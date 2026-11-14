"""Cessna 172M Floatplane.craft: the Cessna 172 airframe (planecraft's build.py on its spec c172float.json: a Spine
fuselage measured off the 172 three-view, B9 wing and tail with flaps, ailerons, elevator and rudder; no engine) made
the 172M on EDO 89-2000 floats:

- its wheels off; its wing and tail weighed as the 172's (FAR's massMultiplier); its flaps and ailerons part of
  their wing strips (one FAR surface with ctrlSurfFrac); its horizontal tail cut to stand in for the downwash
  FAR leaves out (TAIL_SPAN_SCALE, with the stabilizer's incidence in the spec); its elevator part of the stabilizer
  strips; the panels between them FAR control surfaces too (plain_controls);
- the engine on the nose: FARTest_O320E2D (mods/FARTestFloatplane: Lycoming O-320-E2D, 150 hp, on the McCauley
  1A175/ATM8042 80 x 42 seaplane propeller, as AJE models them), on Firespitter's nose engine kept inside the cowling;
- two floats (edo_float.py: NACA 57-B-5 lines at NACA's own size for a 2220 lb twin-float airplane, 16.05 ft x 24.6 in)
  88 in apart centre to centre, base lines level with the fuselage axis, their decks 30 in below the belly (an estimate
  from photographs), each step 0.36 beam aft of the centre of gravity as NACA set it (TN 1524), each on a front strut
  with a rear strut beside it;
- the 172's wing lift struts (LIFT_STRUT), streamlined Spine parts from the lower fuselage to the wing's lower surface.

The centre of gravity is the test's to set (FloatplaneTests: the fuel and the fuselage's CoM put it at the TCDS's
mid-range, 28.4% MAC); its station here, for placing the floats, is that share of the wing's MAC as build.py lays the
wing out (the root panel's chord at its origin).

    python c172_floatplane.py AIRFRAME.craft          writes Cessna 172M Floatplane.craft beside this script
    python c172_floatplane.py AIRFRAME.craft split    writes Cessna 172M Floatplane Split.craft: each float three parts
                                                      (split_hull.py), cut at station 3 and at the step (FLOAT_CUTS)
"""
import json
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "planecraft"))
import craftio as cio  # noqa: E402

import edo_float  # noqa: E402
import split_hull  # noqa: E402
from spine_loft import spine_module  # noqa: E402

IN = 0.0254
Q_FUSE = np.array([0.70710678, 0.0, 0.0, 0.70710678])      # local y forward, z down (the root's)
Q_UP = np.array([0.0, 0.70710678, 0.70710678, 0.0])        # local y forward, z up (x left)
Q_DOWN = np.array([0.0, 0.0, 1.0, 0.0])                  # local y down (half a turn about z)
FLOAT_SPACING = 88 * IN
DECK_BELOW_BELLY = 30 * IN
STEP_AFT_OF_CG = 0.36 * 2.128 * 0.3048       # 0.36 beam over the spray strips
CG_MAC = 0.284
FLOAT_CUTS = ("3", "10 fwd")
# hinged: each float's water rudder (c172_water_rudders.part: a B9 TypeB control surface, yaw only, 17 deg, as rigged in
# the game) at its stern, below its deck: from the float's middle at the deck, craft axes (m)
WATER_RUDDER_AT = (0.0, -0.138, -2.400)                  # split floats: cut at these stations (the bow piece, forebody, afterbody)
MAC = 58.8 * IN                               # the 172's (its leading edge the wing's root leading edge)
ENGINE_S = 0.40                               # m aft of the nose: the engine part inside the cowling
STRUT = (0.10, 0.04)                          # m: a strut's chord and thickness
WING_Z_MIN = -1.0                             # craft z: the wing's panels lie ahead of it, the tail's well aft
# the wing's lift struts as the three-view draws them (planecraft specs/c172float.json): the wing end 2.29 m out (front
# view) at station 1.98 m (side view, from the spinner), the body end at station 1.88, 0.41 m below the body's axis (side
# view), where the drawing's strut meets the lower fuselage: on the Spine's side there (fuselage_side), 1 cm inside it
LIFT_STRUT = dict(wing_y=2.29, wing_s=1.98, body_s=1.88, body_z=-0.41)
SPEC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "planecraft", "specs",
                    "c172float.json")
WING_ROOT_CHORD = 1.634                       # m: the drawing's root chord (build.py's root strip)
# the fuel tank filled to the 172 floatplane's gross weight, 2220 lb (TCDS 3A12), as the craft loads: the airframe empty
# (FloatplaneTests' "mass": 550 kg, the procedural parts' own masses) and LiquidFuel at 5 kg a unit. The tank holds 636 kg;
# full it made 1.19 t. FloatplaneTests sets the weight itself for each run
GROSS_KG = 2220 * 0.45359237
EMPTY_KG = 560.0                              # with the drain valve's 10 kg
# the fuselage's own contents: ProceduralParts' liquid tank has no Structural type (build.py's), so the game falls back
# to its first, Mixed, and fills it (180 kg of LFO in the shell, 0.207 m3 at the 12 mm the game makes it): written empty
FUSELAGE_TANK = dict(tankVolume=0.207292348, mass=0.0225741379, LiquidFuel=16.25, Oxidizer=19.87)
# the craft loads OVER_KG over the gross, in the fuel tank (at the C.G. station: the C.G. stays put; it cannot get over
# the hump), and the drain valve under the belly (action group 1) dumps it: stock, the tank's LiquidFuel (the engine's
# fuel; the demo closes the valve at the gross, by hand: close it after DRAIN_S); RO, the tank's LiquidFuel beside its
# AvGas, the overload alone, so the valve stops at the gross by itself. FloatplaneTests set the weight themselves
OVER_KG = 398 * 0.45359237
# the FTE-1 drain valve (stock ReleaseValve, 10 kg) on the belly at the C.G. station, on the fuel tank: it drains its
# parent at drainRate % of each resource's full amount a second, the overload in DRAIN_S
DRAIN_S = 10.0
# the RO variant ("ro": RealismOverhaul / RP-1 installs, which have no proceduralTankLiquid or probeCoreOcto): the
# fuselage and the fuel tank RO's procedural spaceplane tank, its Fuselage type (RealFuels): the fuselage LiquidFuel
# (1 kg a litre in RO, drainable), the fuel tank AvGas (RO makes the engine's propellant AvGas: RO_AVGAS_KG_L), the probe core the rover body (50 kg, RP-1 avionics for 2 t); the floats and struts stay
# proceduralStructural (RO's has the Structural type). RealFuels keeps a tank's contents in the part's RESOURCE nodes;
# ProceduralParts sets its volume from the shape as it loads. EMPTY_RO_KG: the airframe empty as RO weighs it
# (FloatplaneTests' "mass" record), for the fuel to the gross
RO_TANK = "RO-ProcTankSpaceplane"
# the probe core RP-1's procedural avionics (the user's Concorde CS donor, as planecraft's rp1 profile has it): a small
# cylinder inside the cowling where the probe core was, its avionics for RO_AVIONICS_T of controllable mass
RO_PROBE = "proceduralAvionics"
RO_AVIONICS_DONOR = (os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "planecraft", "donors",
                                  "Real Concorde CS.craft"), "proceduralAvionics_4292737802")
RO_AVIONICS_SIZE = (0.35, 0.30)               # m: diameter, length
RO_AVIONICS_T = 1.5
RO_AVGAS_KG_L = 0.719
EMPTY_RO_KG = 448.0
# the RO parts weigh differently (lighter floats, the RealFuels tanks): the fuel tank this much further forward (m) for
# the C.G. (FloatplaneTests "cg error"; the procedural avionics 16 kg in the nose where the probe core was 60)
RO_TANK_SHIFT = 0.085
RO_MFT = """	MODULE
	{
		name = ModuleFuelTanks
		isEnabled = True
		type = Fuselage
		utilization = 100
		volume = %(volume).3f
		stagingEnabled = True
		EVENTS
		{
		}
		ACTIONS
		{
		}
		UPGRADESAPPLIED
		{
		}
	}
"""
RO_RESOURCE = ("\tRESOURCE\n\t{\n\t\tname = %s\n\t\tamount = %.3f\n\t\tmaxAmount = %.3f\n\t\tflowState = True\n\t\tisTweakable = True\n"
               "\t\thideFlow = False\n\t\tisVisible = True\n\t\tflowMode = Both\n\t}\n")
VALVE_MODULE = """	MODULE
	{
		name = ModuleResourceDrain
		isEnabled = True
		drainRate = 10
		isDraining = False
		flowMode = False
		resourcesToRelease = LiquidFuel,Oxidizer
		stagingEnabled = True
		EVENTS
		{
		}
		ACTIONS
		{
			ToggleResourceDrainAction
			{
				actionGroup = Custom01
			}
		}
		UPGRADESAPPLIED
		{
		}
	}
"""
# the fuel tank (it stands for the fuel and the load) moved forward this far (m) to put the centre of gravity at
# CG_MAC as the craft loads: (target - as built) x the gross / the tank's 537 kg full to the gross, from
# FloatplaneTests' "cg error" record (as built 1.488 m, hinged 1.467, split 1.514 with the plain shift, against 1.499 at
# 28.4 % MAC; the test checks it within 1 cm). The hinged craft's control surfaces are parts of their own, behind the
# hinges, and its CG sits further aft; the split floats' pieces carry their mass further forward. Each 3.1 cm more for
# the drain valve: physicsless, its 10 kg rides at the fuselage's CoM, 1.5 m aft of the CG (hinged: as built 1.483).
# Hinged 2.0 cm more for the water rudders on the floats' sterns (1.0 cm aft), 3.0 cm less for the overload's longer, heavier
# tank (1.6 cm forward); split hinged the split floats' 2.7 cm less
TANK_SHIFT = {"plain": 0.052, "hinged": 0.081, "split": 0.025, "split hinged": 0.054}
# the fuel tank's cylinder inside the cabin (build.py sizes it 0.63 x 2.36 m, 0.1 m above the axis: its front end came
# out through the windshield): TANK_LENGTH long, as wide as keeps its volume, its centre TANK_Y above the fuselage axis
# (the cabin, stations 1.2 - 3.2 m, is 1.07 m wide and 1.2 m tall); its centre along the fuselage, and so the C.G., kept.
# TANK_VOLUME (m3, stock: build.py's 0.73), TANK_Y: the RO variant's AvGas (0.719 kg/L) to the gross over RO's lighter airframe
# (569 kg; 0.73 holds 526), the tank further forward there (RO_TANK_SHIFT): lower, clear of the windshield
# The overload makes the tank longer (its middle, and so its CoM, where it was): stock 0.77 holds the gross and the
# overload (0.73 held 636 kg of 628); RO 1.0 the gross's AvGas (777 L) and the overload's LiquidFuel (181 L)
TANK_LENGTH = {"stock": 1.30 * 0.77 / 0.73, "ro": 1.30 * 1.0 / 0.82}
TANK_VOLUME = {"stock": 0.77, "ro": 1.0}
TANK_Y = {"stock": 0.0, "ro": -0.02}
# FAR weighs a wing by its area (FARWingAerodynamicModel): 1014 kg for these panels as built, where the 172's wing and
# tail are about 115 kg (250 lb): every panel's massMultiplier scaled to that
WING_MASS_SCALE = 115.0 / 1014.0
# FAR puts no wing downwash on a tail (its wing interaction looks one MAC ahead of a surface), so the horizontal tail is
# sized to (1 - d eps / d alpha) of the 172's: the wing's AR 7.44, CL_alpha 2 pi AR / (2 + sqrt(AR^2 + 4)) = 4.81 / rad,
# d eps / d alpha = 2 CL_alpha / (pi AR) = 0.41, the 172's 3.35 m2 (Jane's: 21.56 + 14.53 sq ft) times 0.59 = 1.98 m2
# (TAIL_AREA) against the panels' as built (shrink_tail: 3.26 m2 as the drawing has it): its span cut, its chords kept
# (the elevator's power drops with it)
# The downwash at zero lift slope's other half: at a fuselage alpha of 0 the wing (zero lift at -2.3 deg) still turns
# the flow down 0.41 x 2.3 = 0.94 deg at the tail; on the cut tail that is a stabilizer incidence of -0.94 / 0.59 =
# -1.6 deg (leading edge down; the 172's own taken as the drawing's 0), set in the spec ("incidence": {"tail"}) so
# build.py turns the stabilizer and elevator together
# FAR's lift and control power against JSBSim's c172p (scratchpad planing/c172_calib.py on the Trim test's table):
# FAR deflects a control surface as an angle of attack of deflection x ctrlSurfFrac, so ctrlSurfFrac is the surface's
# effectiveness, not its chord share: the flaps' 0.57 gives c172p's CL increment at 10 deg (0.20; at 0.30, 0.106), the
# elevator's 0.99 its power at constant lift (0.98 / rad; at 0.31, 0.31) (both in mods/FARTestFloatplane). FAR's wing
# lifts 0.071 / deg against c172p's 0.096: the wing's incidence 2.3 -> 7.4 deg (the spec) puts c172p's lift in ground
# effect at the floats' height (h/b 0.29, x1.05) on FAR's over the lift-off attitudes, 7-11 deg (within 0.01 at
# 8-10), at the price of cruise attitude: CL 0.42 at -1.1 deg against c172p's 1.8. Cm at neutral elevator, flaps 10,
# already c172p's within 0.02 (0.033 - 0.028 / deg against 0.051 - 0.028): the tail's incidence kept
TAIL_AREA = 1.98


def spine_nodes(block):
    """the fuselage Spine's nodes: (position, sizeH, sizeV, offsetV), and its length"""
    i, j = cio.module_span(block, "ProceduralShapeSpine")
    mod = block[i:j]
    length = float(re.search(r"(?m)^\t\tlength = (.*)$", mod).group(1))
    nodes = []
    for m in re.finditer(r"SPINE_NODE\n\t\t\{\n(.*?)\t\t\}", mod, re.S):
        g = lambda k: float(re.search(r"(?m)^\t\t\t%s = (.*)$" % k, m.group(1)).group(1))
        nodes.append((g("position"), g("sizeH"), g("sizeV"), g("offsetV")))
    return sorted(nodes), length


def fuselage_at(nodes, length, y0, z):
    """the fuselage's bottom and centre heights (craft y) at craft z, the root lying local z down at height y0"""
    p = z / length + 0.5
    pos = [n[0] for n in nodes]
    size_v = np.interp(p, pos, [n[2] for n in nodes])
    off = np.interp(p, pos, [n[3] for n in nodes])
    return y0 - (off + size_v / 2), y0 - off


def wing_root_le(parts):
    """the wing's root leading edge (craft z) from build.py's right-hand root panel (B9 TypeA lying as Q_RIGHT, root
    mid-chord of its base at the part origin, its leading edge strip ahead of the base)"""
    best = None
    for b in parts:
        if not cio.part_id(b).startswith(("B9.Aero.Wing.Procedural.TypeA", "FARTest.B9.Wing")):
            continue
        pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
        rot = [float(v) for v in re.search(r"(?m)^\trot = (.*)$", b).group(1).split(",")]
        if pos[2] < WING_Z_MIN or abs(rot[0]) < 0.5:   # the wing (the tail lies aft), right-hand (Q_RIGHT)
            continue
        if best is None or pos[0] < best[0][0]:
            best = (pos, b)
    pos, b = best
    g = lambda k: float(re.search(r"(?m)^\t\t%s = (.*)$" % k, b).group(1))
    return pos[2] + g("sharedBaseWidthRoot") / 2 + g("sharedEdgeWidthLeadingRoot")


def structural(donor, name, pid, spine, pos, rot, parent):
    """the fuselage's block as a solid structural Spine part (ProceduralParts' proceduralStructural, whose Structural
    contents weigh 0.05 t/m3; the liquid tank part has no such option and falls back to a fuel tank's)"""
    b = cio.place(donor, name, pid, pos, rot, parent)
    b = re.sub(r"(?m)^\tpart = proceduralTankLiquid_", "\tpart = proceduralStructural_", b, count=1)
    i, j = cio.module_span(b, "ProceduralShapeSpine")
    b = b[:i] + spine + b[j:]
    b = re.sub(r"\tRESOURCE\n\t\{\n.*?\n\t\}\n", "", b, flags=re.S)
    b = cio.drop_lines(b, "link")
    b = cio.setf(b, "autostrutMode", "Root")
    b = cio.setf(b, "rigidAttachment", "True")
    return b


def frame_q(y_axis, fwd=(0.0, 0.0, 1.0)):
    """the rotation taking a part's local y to y_axis and its local z as near fwd as it can (x, y, z, w)"""
    y = np.asarray(y_axis, float) / np.linalg.norm(y_axis)
    z = np.asarray(fwd, float) - np.dot(fwd, y) * y
    z /= np.linalg.norm(z)
    x = np.cross(y, z)
    m = np.column_stack([x, y, z])
    w = math.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    q = np.array([(m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w])
    assert np.allclose(cio.qrot(q, [0, 1, 0]), y, atol=1e-6) and np.allclose(cio.qrot(q, [0, 0, 1]), z, atol=1e-6)
    return q


def fuselage_side(nodes, length, y0, z, y):
    """the fuselage's half-width at craft z and height y: the Spine's elliptical section there"""
    p = z / length + 0.5
    pos = [n[0] for n in nodes]
    w, h, off = (np.interp(p, pos, [n[k] for n in nodes]) for k in (1, 2, 3))
    u = (y - (y0 - off)) / (h / 2)
    return 0.5 * w * math.sqrt(max(0.0, 1 - u * u))


def wing_lower(spec, y0, le_z, x, z):
    """the wing's lower surface's height (craft y) at craft (x, z): build.py's slab (thickness_ratio of the root chord),
    its root chord's quarter point at the spec's height, turned up by its incidence about that point and by its
    dihedral about the centre line"""
    h = spec["heights"]["wing"]
    inc = math.radians(spec["incidence"]["wing"])
    z_q = le_z - 0.25 * WING_ROOT_CHORD
    half = 0.5 * spec["thickness_ratio"] * WING_ROOT_CHORD
    return y0 + h["z"] + math.tan(math.radians(h["dihedral"])) * abs(x) + (z - z_q) * math.tan(inc) - half


def strut_spine(length):
    """a streamlined strut: an ellipse STRUT[0] x STRUT[1], length long along local y"""
    node = ("\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %g\n\t\t\tsizeH = %g\n\t\t\tsizeV = %g\n\t\t\toffsetV = 0\n"
            "\t\t\tsection = Ellipse\n\t\t}\n")
    return ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = %.5f\n\t\thScale = 1\n"
            "\t\tvScale = 1\n\t\tshellMode = False\n\t\tstagingEnabled = True\n" % length +
            node % (0, STRUT[1], STRUT[0]) + node % (1, STRUT[1], STRUT[0]) +
            "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")


ENGINE_BLOCK = """PART
{
	part = %(name)s
	partName = Part
	persistentId = %(pid)d
	pos = %(pos)s
	attPos = 0,0,0
	attPos0 = %(attpos)s
	rot = %(rot)s
	attRot = 0,0,0,1
	attRot0 = %(attrot)s
	mir = 1,1,1
	symMethod = Mirror
	autostrutMode = Root
	rigidAttachment = True
	istg = 0
	resPri = 0
	dstg = 0
	sidx = 0
	sqor = 0
	sepI = 0
	attm = 1
	modCost = 0
	modMass = 0
	modSize = 0,0,0
	srfN = srfAttach,%(parent)s
	EVENTS
	{
	}
	ACTIONS
	{
	}
	PARTDATA
	{
	}
}
"""


def ro_variant(head, parts, fuel_kg, over_kg, fus_pose):
    """the craft's parts as RO has them (RO_TANK, RO_PROBE): the fuselage (parts[0]) empty, the fuel tank fuel_kg of
    AvGas and over_kg of LiquidFuel (1 kg a litre, drainable: the overload the valve dumps); names swapped in every
    reference (links, srfN)"""
    out = []
    for n, b in enumerate(parts):
        pid = cio.part_id(b)
        if pid.startswith("proceduralTankLiquid_"):
            i, j = cio.module_span(b, "TankContentSwitcher")
            vol = float(re.search(r"(?m)^\t\ttankVolume = (.*)$", b[i:j]).group(1)) * 1000.0   # m3 -> L
            b = b[:i] + RO_MFT % dict(volume=vol) + b[j:]
            b = re.sub(r"\tRESOURCE\n\t\{\n.*?\n\t\}\n", "", b, flags=re.S)
            k = b.rstrip().rindex("}")
            if n == 0:
                b = b[:k] + RO_RESOURCE % ("LiquidFuel", 0.0, vol) + b[k:]
            else:
                gas = fuel_kg / RO_AVGAS_KG_L
                b = b[:k] + RO_RESOURCE % ("AvGas", gas, gas) + RO_RESOURCE % ("LiquidFuel", over_kg, over_kg) + b[k:]
        elif pid.startswith("probeCoreOcto"):
            # the probe core's pose and id on RP-1's procedural avionics
            f, dpid = RO_AVIONICS_DONOR
            _, donors = cio.split_parts(open(f, encoding="utf-8").read().replace("\r\n", "\n"))
            donor = next(d for d in donors if cio.part_id(d) == dpid)
            pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
            rot = [float(v) for v in re.search(r"(?m)^\trot = (.*)$", b).group(1).split(",")]
            pers = int(re.search(r"(?m)^\tpersistentId = (\d+)", b).group(1))
            num = re.search(r"_(\d+)$", pid).group(1)
            b = cio.place(donor, "%s_%s" % (RO_PROBE, num), pers, np.array(pos), np.array(rot), fus_pose)
            b = cio.set_mod(b, "ProceduralShapeCylinder", "diameter", "%.3f" % RO_AVIONICS_SIZE[0])
            b = cio.set_mod(b, "ProceduralShapeCylinder", "length", "%.3f" % RO_AVIONICS_SIZE[1])
            b = cio.set_mod(b, "ModuleProceduralAvionics", "controllableMass", "%g" % RO_AVIONICS_T)
        out.append(b)
    text = head + "".join(out)
    text = re.sub(r"proceduralTankLiquid_(\d+)", RO_TANK + r"_\1", text)
    return re.sub(r"probeCoreOcto(?:\.v2)?_(\d+)", RO_PROBE + r"_\1", text)


def panel_area(b):
    """a B9 panel's area: its span times its mean chord (leading edge, base and trailing edge widths, as B9 gives FAR)"""
    g = lambda k_: float(re.search(r"(?m)^\t\t%s = (.*)$" % k_, b).group(1))
    chord = lambda end: sum(g("shared%s%s" % (w, end)) for w in ("EdgeWidthLeading", "BaseWidth", "EdgeWidthTrailing"))
    return g("sharedBaseLength") * 0.5 * (chord("Root") + chord("Tip"))


def shrink_tail(parts, area):
    """The horizontal tail's span (and so its area) cut to area, its chords kept: the B9 panels aft of the wing with a
    mirror partner (the fin has none), their spans, their distances out from the center line and their offsets along
    their parents' spans scaled. Its area: the panels' and a hinged elevator's behind them (its trailing edge widths)."""
    tail = set()
    for b in parts:
        pid = cio.part_id(b)
        pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
        if pid.startswith(("B9.Aero", "FARTest.B9")) and pos[2] < -2 and re.search(r"(?m)^\tsym = ", b):
            tail.add(pid)
    def own_area(b):
        if "TypeB" not in cio.part_id(b):
            return panel_area(b)
        g = lambda k_: float(re.search(r"(?m)^\t\t%s = (.*)$" % k_, b).group(1))
        return g("sharedBaseLength") * 0.5 * (g("sharedEdgeWidthTrailingRoot") + g("sharedEdgeWidthTrailingTip"))
    built = sum(own_area(b) for b in parts if cio.part_id(b) in tail)
    k = area / built
    out = []
    for b in parts:
        pid = cio.part_id(b)
        if pid in tail:
            g = lambda k_: float(re.search(r"(?m)^\t\t%s = (.*)$" % k_, b).group(1))
            b = cio.set_mod(b, "WingProcedural", "sharedBaseLength", "%.6g" % (g("sharedBaseLength") * k))
            pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
            b = cio.setf(b, "pos", cio.fmt([pos[0] * k, pos[1], pos[2]]))
            parent = re.search(r"(?m)^\tsrfN = srfAttach,([^,\n]+)", b).group(1)
            if parent in tail:
                ap = [float(v) for v in re.search(r"(?m)^\tattPos0 = (.*)$", b).group(1).split(",")]
                b = cio.setf(b, "attPos0", cio.fmt([ap[0] * k, ap[1], ap[2]]))
        out.append(b)
    print(f"  horizontal tail: {len(tail)} panels, {built:.2f} m2 as built, span x{k:.3f} to {area:.2f} m2")
    return out


def merge_wing_controls(parts):
    """Each wing flap and aileron (a B9 TypeB behind the hinge on a TypeA strip) made part of its strip: the strip
    FARTest_B9_WingFlap or _WingAileron (mods/FARTestFloatplane: a TypeA with FAR's FARControllableSurface, its ctrlSurfFrac in the part), its trailing edge strip the
    control's chord behind the hinge, the control's FAR settings, ctrlSurfFrac that chord's share. FAR then models one
    wing with a plain flap rather than a wing and a separate small one whose lift acts half a meter behind (the airframe's
    neutral point 49% MAC aft of the C.G. as built). The tail's elevator: merge_tail_elevator."""
    by_id = {cio.part_id(b): i for i, b in enumerate(parts)}
    renames, drop = {}, set()
    for b in parts:
        pid = cio.part_id(b)
        if not pid.startswith("B9.Aero.Wing.Procedural.TypeB") or "\t\tname = FARControllableSurface\n" not in b:
            continue
        parent = re.search(r"(?m)^\tsrfN = srfAttach,([^,\n]+)", b).group(1)
        if not parent.startswith("B9.Aero.Wing.Procedural.TypeA") or parent not in by_id:
            continue
        a = parts[by_id[parent]]
        pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", a).group(1).split(",")]
        if pos[2] < WING_Z_MIN:                               # the wing only (the tail lies aft)
            continue
        g = lambda blk, k: float(re.search(r"(?m)^\t\t%s = (.*)$" % k, blk).group(1))
        trail_r, trail_t = g(b, "sharedEdgeWidthTrailingRoot"), g(b, "sharedEdgeWidthTrailingTip")
        chord_r = g(a, "sharedEdgeWidthLeadingRoot") + g(a, "sharedBaseWidthRoot") + trail_r
        frac = trail_r / chord_r
        a = cio.set_mod(a, "WingProcedural", "sharedEdgeWidthTrailingRoot", "%.6g" % trail_r)
        a = cio.set_mod(a, "WingProcedural", "sharedEdgeWidthTrailingTip", "%.6g" % trail_t)
        # the control's FAR module in place of the strip's wing model, with its chord share
        i, j = cio.module_span(b, "FARControllableSurface")
        ctrl = b[i:j].replace("\t\tname = FARControllableSurface\n",
                              "\t\tname = FARControllableSurface\n\t\tctrlSurfFrac = %.4f\n" % frac, 1)
        i, j = cio.module_span(a, "FARWingAerodynamicModel")
        a = a[:i] + ctrl + a[j:]
        a = re.sub(r"(?m)^\tlink = %s\n" % re.escape(pid), "", a)
        parts[by_id[parent]] = a
        kind = "Flap" if "isFlap = True" in ctrl else "Aileron"
        renames[parent] = parent.replace("B9.Aero.Wing.Procedural.TypeA_", "FARTest.B9.Wing%s_" % kind)
        drop.add(pid)
        print(f"  {parent} + {pid}: chord {chord_r:.3f} m at the root, {frac:.0%} control "
              f"({'flap' if 'isFlap = True' in ctrl else 'aileron'})")
    parts = [b for b in parts if cio.part_id(b) not in drop]
    out = []
    for b in parts:
        for old, new in renames.items():
            b = re.sub(r"\b%s\b" % re.escape(old), new, b)
        out.append(b)
    return out


def merge_tail_elevator(parts):
    """Each elevator (a B9 TypeB with FAR's pitch axis, centered on its span, over the stabilizer strips it is attached
    across: its parent, and the parent's inboard strip where the parent is shorter) made part of those strips, as the
    wing's flaps are: each
    strip FARTest_B9_TailElevator (ctrlSurfFrac in mods/FARTestFloatplane), its trailing edge the elevator's chord over
    its stretch of the span, the elevator's FAR settings. As two panels FAR gave the elevator a third of a plain flap's
    power and less going up than down."""
    by_id = {cio.part_id(b): i for i, b in enumerate(parts)}
    renames, drop = {}, set()
    g = lambda blk, k: float(re.search(r"(?m)^\t\t%s = (.*)$" % k, blk).group(1))
    parent_of = lambda blk: re.search(r"(?m)^\tsrfN = srfAttach,([^,\n]+)", blk).group(1)
    for b in parts:
        pid = cio.part_id(b)
        if not pid.startswith("B9.Aero.Wing.Procedural.TypeB") or "\t\tpitchaxis = 100\n" not in b:
            continue
        outer = parent_of(b)
        assert outer.startswith("B9.Aero.Wing.Procedural.TypeA")
        l_el = g(b, "sharedBaseLength")
        strips = [outer]
        if abs(g(parts[by_id[outer]], "sharedBaseLength") - l_el) > 0.05 * l_el:
            strips.insert(0, parent_of(parts[by_id[outer]]))
            assert strips[0].startswith("B9.Aero.Wing.Procedural.TypeA")
        lens = [g(parts[by_id[sid]], "sharedBaseLength") for sid in strips]
        assert abs(sum(lens) - l_el) < 0.05 * l_el, (lens, l_el)
        c_r, c_t = g(b, "sharedEdgeWidthTrailingRoot"), g(b, "sharedEdgeWidthTrailingTip")
        ends = np.cumsum([0.0] + lens) / sum(lens)
        i, j = cio.module_span(b, "FARControllableSurface")
        ctrl = b[i:j]
        area = el_area = 0.0
        for sid, cr, ct in zip(strips, c_r + (c_t - c_r) * ends[:-1], c_r + (c_t - c_r) * ends[1:]):
            a = parts[by_id[sid]]
            a = cio.set_mod(a, "WingProcedural", "sharedEdgeWidthTrailingRoot", "%.6g" % cr)
            a = cio.set_mod(a, "WingProcedural", "sharedEdgeWidthTrailingTip", "%.6g" % ct)
            chord_r = g(a, "sharedEdgeWidthLeadingRoot") + g(a, "sharedBaseWidthRoot") + cr
            chord_t = g(a, "sharedEdgeWidthLeadingTip") + g(a, "sharedBaseWidthTip") + ct
            area += (chord_r + chord_t) / 2 * g(a, "sharedBaseLength")
            el_area += (cr + ct) / 2 * g(a, "sharedBaseLength")
            m, n = cio.module_span(a, "FARWingAerodynamicModel")
            a = a[:m] + ctrl + a[n:]
            a = re.sub(r"(?m)^\tlink = %s\n" % re.escape(pid), "", a)
            parts[by_id[sid]] = a
            renames[sid] = sid.replace("B9.Aero.Wing.Procedural.TypeA_", "FARTest.B9.TailElevator_")
        drop.add(pid)
        print(f"  {' + '.join(strips)} + {pid}: elevator {el_area / area:.0%} of the strips' area")
    parts = [b for b in parts if cio.part_id(b) not in drop]
    out = []
    for b in parts:
        for old, new in renames.items():
            b = re.sub(r"\b%s\b" % re.escape(old), new, b)
        out.append(b)
    return out


def plain_controls(parts):
    """The wing's and horizontal tail's other panels (no control on them) made FAR control surfaces with no axes,
    FARTest_B9_Plain: FAR's whole-wing aspect ratio groups touching panels of one kind (wing panels, or control
    surfaces), so with the flap, aileron and elevator strips control surfaces and the panels between them wing panels
    each wing came apart into short pieces (the flap strips AR 3.6, the ailerons 5.2, the wing 7.4; the tail 2.5) and
    FAR's induced drag doubled. The fin is left as it is."""
    template = next(b for b in parts if cio.part_id(b).startswith("FARTest.B9.WingAileron"))
    i, j = cio.module_span(template, "FARControllableSurface")
    module = template[i:j]
    for k, v in (("pitchaxis", "0"), ("yawaxis", "0"), ("rollaxis", "0"), ("pitchaxisDueToAoA", "0"), ("brakeRudder", "0"),
                 ("maxdeflect", "0"), ("isFlap", "False"), ("maxdeflectFlap", "0")):
        module = re.sub(r"(?m)^\t\t%s = .*$" % k, "\t\t%s = %s" % (k, v), module)
    renames = {}
    for n, b in enumerate(parts):
        pid = cio.part_id(b)
        if not pid.startswith("B9.Aero.Wing.Procedural.TypeA") or "\t\tname = FARWingAerodynamicModel\n" not in b:
            continue
        pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
        if not (pos[2] >= WING_Z_MIN or (pos[2] < -2 and re.search(r"(?m)^\tsym = ", b))):     # the wing, or the tail (not the fin)
            continue
        m, k = cio.module_span(b, "FARWingAerodynamicModel")
        mass = re.search(r"(?m)^\t\tmassMultiplier = (.*)$", b[m:k]).group(1)
        own = re.sub(r"(?m)^\t\tmassMultiplier = .*$", "\t\tmassMultiplier = %s" % mass, module)
        parts[n] = b[:m] + own + b[k:]
        renames[pid] = pid.replace("B9.Aero.Wing.Procedural.TypeA_", "FARTest.B9.Plain_")
    print(f"  {len(renames)} plain wing and tail panels as FAR control surfaces with no axes")
    out = []
    for b in parts:
        for old, new in renames.items():
            b = re.sub(r"\b%s\b" % re.escape(old), new, b)
        out.append(b)
    return out


# the water rudders' part blocks (left, right), their names, ids, positions and parents to fill in
WATER_RUDDERS = re.findall(r"(?m)^PART\n\{\n.*?^\}\n", open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "c172_water_rudders.part"), encoding="utf-8").read(), re.S)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    flags = set(sys.argv[2:])
    split = "split" in flags
    # hinged: B9's own control surfaces (no FARTest parts); noplain: the controls merged into their strips, the other
    # panels left plain wing panels (for the Components tab's picture of the wing split at the strips)
    hinged, noplain = "hinged" in flags, "noplain" in flags
    ro = "ro" in flags      # RO's parts (ro_variant)
    ship = "Cessna 172M Floatplane" + (" Split" if split else "") + (" Hinged" if hinged else "") + (" NoPlain" if noplain else "") + \
        (" RO" if ro else "")

    text = open(sys.argv[1], encoding="utf-8").read().replace("\r\n", "\n")
    head, parts = cio.split_parts(text)
    head = re.sub(r"(?m)^ship = .*$", "ship = " + ship, head)
    head = re.sub(r"(?m)^description = .*$", "description = The Cessna 172M on EDO 89-2000 floats for FAR's water tests "
                  "(crafts/c172_floatplane.py): Lycoming O-320-E2D on the 80 x 42 seaplane propeller (AJE), NACA 57-B-5 "
                  "floats at the EDO's size" + ("; each float three parts (split_hull.py)" if split else ""), head)

    # the wheels off
    gear = [cio.part_id(b) for b in parts if cio.part_id(b).startswith("KF-")]
    parts = [b for b in parts if cio.part_id(b) not in gear]
    parts = [re.sub(r"(?m)^\tlink = (%s)\n" % "|".join(map(re.escape, gear)), "", b) for b in parts]

    # the wing and tail weighed as a 172's
    def wing_mass(b):
        far = "FARWingAerodynamicModel" if "\t\tname = FARWingAerodynamicModel\n" in b else "FARControllableSurface"
        return cio.set_mod(b, far, "massMultiplier", "%.4f" % WING_MASS_SCALE)
    parts = [wing_mass(b) if cio.part_id(b).startswith("B9.Aero") else b for b in parts]
    if not hinged:
        parts = merge_wing_controls(parts)
        parts = merge_tail_elevator(parts)
        if not noplain:
            parts = plain_controls(parts)
    parts = shrink_tail(parts, TAIL_AREA)

    fus = parts[0]
    fus_name = cio.part_id(fus)
    y0 = float(re.search(r"(?m)^\tpos = (.*)$", fus).group(1).split(",")[1])
    nodes, length = spine_nodes(fus)
    le_z = wing_root_le(parts)
    z_cg = le_z - CG_MAC * MAC
    print(f"fuselage {length:.2f} m, nose at z {length / 2:+.3f}; wing root leading edge at z {le_z:+.3f} "
          f"({length / 2 - le_z:.3f} m aft of the nose); CG ({CG_MAC:.1%} MAC) at z {z_cg:+.3f}")
    fus_pose = (fus_name, np.array([0.0, y0, 0.0]), Q_FUSE)
    new, links = [], []
    pid = 4100000000

    # the engine inside the cowling
    zn = length / 2 - ENGINE_S
    _, yc = fuselage_at(nodes, length, y0, zn)
    epos = np.array([0.0, yc, zn])
    pid += 1
    ename = "FARTest.O320E2D_%d" % pid
    new.append(ENGINE_BLOCK % dict(name=ename, pid=pid, pos=cio.fmt(epos), rot=cio.fmt(Q_FUSE), parent=fus_name,
                                   attpos=cio.fmt(cio.qrot(cio.qinv(Q_FUSE), epos - fus_pose[1])),
                                   attrot=cio.fmt(cio.qmul(cio.qinv(Q_FUSE), Q_FUSE))))
    links.append(ename)

    # the wing's lift struts, from the fuselage's side to 2 cm into the wing's lower surface
    spec = json.load(open(SPEC, encoding="utf-8"))
    ls = LIFT_STRUT
    for side in (1, -1):
        top = np.array([side * ls["wing_y"], 0.0, length / 2 - ls["wing_s"]])
        top[1] = wing_lower(spec, y0, le_z, top[0], top[2]) + 0.02
        bz = length / 2 - ls["body_s"]
        bot = np.array([side * (fuselage_side(nodes, length, y0, bz, y0 + ls["body_z"]) - 0.01), y0 + ls["body_z"], bz])
        d = top - bot
        pid += 1
        sname = "proceduralStructural_%d" % pid
        new.append(structural(fus, sname, pid, strut_spine(float(np.linalg.norm(d))), (top + bot) / 2, frame_q(d),
                              fus_pose))
        links.append(sname)
        if side > 0:
            print(f"lift struts: {np.linalg.norm(d):.3f} m, wing end at x {top[0]:.2f} y {top[1]:.3f} z {top[2]:+.3f}, "
                  f"body end at x {bot[0]:.2f} y {bot[1]:.3f} z {bot[2]:+.3f}")

    # the floats: the step STEP_AFT_OF_CG aft of the CG, the deck DECK_BELOW_BELLY below the belly there
    s = edo_float
    z_step = z_cg - STEP_AFT_OF_CG
    belly, _ = fuselage_at(nodes, length, y0, z_step)
    deck = belly - DECK_BELOW_BELLY
    float_len = s.LENGTH_IN * s.SX
    z_float = z_step + (s.STEP_X_IN / s.LENGTH_IN - 0.5) * float_len      # the spine's middle
    float_spine = spine_module(s.STATIONS, s.LENGTH_IN, s.SX, s.SY, s.SZ, "EDO57B5").replace(
        "\t\tsectionPoints = 96\n", "\t\tsectionPoints = 96\n\t\tshellMode = False\n")
    print(f"floats: {float_len:.3f} m, step at z {z_step:+.3f}, deck at y {deck:.3f} ({belly - deck:.3f} m below the belly "
          f"at {belly:.3f}), keel at the step {deck - 11.40 * s.SZ:.3f}, spine middle at z {z_float:+.3f}")
    for side in (1, -1):
        x = side * FLOAT_SPACING / 2
        # front strut over the float's forward third, rear strut near the step; both from the belly to the deck
        struts = []
        for zs in (z_step + 0.35 * float_len, z_step - 0.05):
            b_belly, _ = fuselage_at(nodes, length, y0, zs)
            top, bot = b_belly + 0.05, deck
            spos = np.array([x * 0.5, (top + bot) / 2, zs])      # the struts lean out to the floats: at mid-height
            pid += 1
            sname = "proceduralStructural_%d" % pid
            lean = math.degrees(math.atan2(x, top - bot))
            rot = cio.qmul(cio.axis_angle([0, 0, 1], lean), Q_DOWN)
            new.append(structural(fus, sname, pid, strut_spine(math.hypot(top - bot, x) + 0.05), spos, rot, fus_pose))
            links.append(sname)
            struts.append((sname, spos, rot))
        pid += 1
        fname = "proceduralStructural_%d" % pid
        fpos = np.array([x, deck, z_float])
        fb = structural(fus, fname, pid, float_spine, fpos, Q_UP, struts[0])
        if split:
            cuts = [1 - x / s.LENGTH_IN for name, x, _, _ in s.STATIONS if name in FLOAT_CUTS]
            pieces = split_hull.split_part(fb, cuts, pid, parent=(struts[0][1], struts[0][2]))
            pid += len(pieces)
            new.extend(b for b, _ in pieces)
            new[-2 - len(pieces)] = cio.add_links(new[-2 - len(pieces)], [fname])   # the front strut carries it
        else:
            new.append(fb)
            new[-3] = cio.add_links(new[-3], [fname])               # the front strut carries the float
        if hinged:
            # the water rudder, on the float's stern piece (a split float's first)
            stern = pieces[0][1] if split else fname
            k_st = next(i for i in range(len(new)) if cio.part_id(new[i]) == stern)
            spos = np.array([float(v) for v in re.search(r"(?m)^\tpos = (.*)$", new[k_st]).group(1).split(",")])
            srot = [float(v) for v in re.search(r"(?m)^\trot = (.*)$", new[k_st]).group(1).split(",")]
            rpos = fpos + np.array(WATER_RUDDER_AT)
            pid += 1
            rname = "B9.Aero.Wing.Procedural.TypeB_%d" % pid
            rb = WATER_RUDDERS[0 if side < 0 else 1]
            for k, v in (("@NAME@", rname), ("@PID@", str(pid)), ("@POS@", cio.fmt(rpos)),
                         ("@ATTPOS@", cio.fmt(cio.qrot(cio.qinv(srot), rpos - spos))), ("@PARENT@", stern)):
                rb = rb.replace(k, v)
            new.append(rb)
            new[k_st] = cio.add_links(new[k_st], [rname])
            print(f"water rudder on {stern} at x {rpos[0]:+.3f} y {rpos[1]:.3f} z {rpos[2]:+.3f}")

    for n, b in enumerate(parts):
        if cio.part_id(b).startswith("proceduralTankLiquid") and n > 0 and "\t\tname = LiquidFuel\n" in b:
            units = (GROSS_KG - EMPTY_KG + OVER_KG) / 5.0
            b = re.sub(r"(\t\tname = LiquidFuel\n\t\tamount = )[^\n]*", r"\g<1>%.3f" % units, b)
            tank_name = cio.part_id(b)
            dz = TANK_SHIFT["split hinged" if split and hinged else "hinged" if hinged else "split" if split else "plain"] +                 (RO_TANK_SHIFT if ro else 0.0)
            pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
            ap = [float(v) for v in re.search(r"(?m)^\tattPos0 = (.*)$", b).group(1).split(",")]
            assert re.search(r"(?m)^\tsrfN = srfAttach,%s," % re.escape(fus_name), b), "the tank on the fuselage"
            tv, ty = TANK_VOLUME["ro" if ro else "stock"], TANK_Y["ro" if ro else "stock"]
            b = cio.setf(b, "pos", cio.fmt([pos[0], y0 + ty, pos[2] + dz]))
            b = cio.setf(b, "attPos0", cio.fmt([ap[0], ap[1] + dz, -ty]))     # the fuselage's local y forward, z down
            i, j = cio.module_span(b, "ProceduralShapeCylinder")
            d0 = float(re.search(r"(?m)^\t\tdiameter = (.*)$", b[i:j]).group(1))
            l0 = float(re.search(r"(?m)^\t\tlength = (.*)$", b[i:j]).group(1))
            tl = TANK_LENGTH["ro" if ro else "stock"]
            d1 = math.sqrt(tv / tl / math.pi * 4)
            b = cio.set_mod(b, "TankContentSwitcher", "tankVolume", "%.4f" % tv)
            b = cio.set_mod(b, "ProceduralShapeCylinder", "diameter", "%.4f" % d1)
            b = cio.set_mod(b, "ProceduralShapeCylinder", "length", "%.4f" % tl)
            print(f"fuel tank: {d0:.3f} x {l0:.3f} m -> {d1:.3f} x {tl:.3f} m, {ty:+.2f} m from the axis")
            parts[n] = b
            print(f"fuel: {units:.1f} units ({units * 5:.0f} kg) for {GROSS_KG:.0f} kg gross and {OVER_KG:.0f} kg over; the tank "
                  f"{dz * 100:.1f} cm forward for the C.G.")
            tank_pose = (tank_name, np.array([pos[0], y0 + ty, pos[2] + dz]),
                         np.array([float(v) for v in re.search(r"(?m)^\trot = (.*)$", b).group(1).split(",")]))

    # the fuselage's LFO written empty (FUSELAGE_TANK) and the drain valve under it, on the fuel tank
    fus = cio.set_mod(parts[0], "TankContentSwitcher", "tankType", "Mixed")
    for k in ("tankVolume", "mass"):
        fus = cio.set_mod(fus, "TankContentSwitcher", k, "%.9g" % FUSELAGE_TANK[k])
    assert "\tRESOURCE\n" not in fus, "build.py's fuselage has no resources"
    res = "".join("\tRESOURCE\n\t{\n\t\tname = %s\n\t\tamount = 0\n\t\tmaxAmount = %g\n\t\tflowState = True\n\t\tisTweakable = True\n"
                  "\t\thideFlow = False\n\t\tisVisible = True\n\t\tflowMode = Both\n\t}\n" % (r, FUSELAGE_TANK[r])
                  for r in ("LiquidFuel", "Oxidizer"))
    k = fus.rstrip().rindex("}")
    parts[0] = fus[:k] + res + fus[k:]
    belly, _ = fuselage_at(nodes, length, y0, z_cg)
    vpos = np.array([0.0, belly + 0.01, z_cg])
    pid += 1
    vname = "ReleaseValve_%d" % pid
    tname, tpos, trot = tank_pose
    vb = ENGINE_BLOCK % dict(name=vname, pid=pid, pos=cio.fmt(vpos), rot=cio.fmt(Q_FUSE), parent=tname,
                             attpos=cio.fmt(cio.qrot(cio.qinv(trot), vpos - tpos)),
                             attrot=cio.fmt(cio.qmul(cio.qinv(trot), Q_FUSE)))
    k = vb.rstrip().rindex("}")
    # the drain rate: the overload out in DRAIN_S (stock: of the tank's LiquidFuel, full at its volume; RO: the
    # overload's LiquidFuel is the resource's whole amount)
    stock_full = TANK_VOLUME["stock"] / 0.73 * 636.0
    rate = 100.0 / DRAIN_S if ro else 100.0 * OVER_KG / stock_full / DRAIN_S
    new.append(vb[:k] + VALVE_MODULE.replace("drainRate = 10", "drainRate = %.2f" % rate) + vb[k:])
    k_t = next(i for i in range(len(parts)) if cio.part_id(parts[i]) == tname)
    parts[k_t] = cio.add_links(parts[k_t], [vname])
    print(f"fuselage empty; {OVER_KG:.0f} kg over in the fuel tank ({(GROSS_KG + OVER_KG) / 0.45359237:.0f} lb loaded); "
          f"drain valve at y {vpos[1]:.3f} z {z_cg:+.3f} on the tank, {rate:.2f} %/s")

    parts[0] = cio.add_links(parts[0], links)
    if ro:
        out = ro_variant(head, parts + new, GROSS_KG - EMPTY_RO_KG, OVER_KG, fus_pose)
        print(f"RO: {RO_TANK} fuselage (empty) and fuel tank ({GROSS_KG - EMPTY_RO_KG:.0f} kg AvGas, {OVER_KG:.0f} kg "
              f"LiquidFuel over), {RO_PROBE}")
    else:
        out = head + "".join(parts + new)
    path = os.path.join(here, ship + ".craft")
    open(path, "w", encoding="utf-8", newline="\n").write(out)
    print(f"wrote {path}: {len(parts) + len(new)} parts")


if __name__ == "__main__":
    main()
