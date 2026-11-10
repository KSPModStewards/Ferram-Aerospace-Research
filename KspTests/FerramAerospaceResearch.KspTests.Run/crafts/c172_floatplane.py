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
  with a rear strut beside it.

The centre of gravity is the test's to set (FloatplaneTests: the fuel and the fuselage's CoM put it at the TCDS's
mid-range, 28.4% MAC); its station here, for placing the floats, is that share of the wing's MAC as build.py lays the
wing out (the root panel's chord at its origin).

    python c172_floatplane.py AIRFRAME.craft          writes Cessna 172M Floatplane.craft beside this script
    python c172_floatplane.py AIRFRAME.craft split    writes Cessna 172M Floatplane Split.craft: each float three parts
                                                      (split_hull.py), cut at station 3 and at the step (FLOAT_CUTS)
"""
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
FLOAT_CUTS = ("3", "10 fwd")                  # split floats: cut at these stations (the bow piece, forebody, afterbody)
MAC = 58.8 * IN                               # the 172's (its leading edge the wing's root leading edge)
ENGINE_S = 0.40                               # m aft of the nose: the engine part inside the cowling
STRUT = (0.10, 0.04)                          # m: a strut's chord and thickness
# FAR weighs a wing by its area (FARWingAerodynamicModel): 1014 kg for these panels as built, where the 172's wing and
# tail are about 115 kg (250 lb): every panel's massMultiplier scaled to that
WING_MASS_SCALE = 115.0 / 1014.0
# FAR puts no wing downwash on a tail (its wing interaction looks one MAC ahead of a surface), so the horizontal tail is
# sized to (1 - d eps / d alpha) of the 172's: the wing's AR 7.44, CL_alpha 2 pi AR / (2 + sqrt(AR^2 + 4)) = 4.81 / rad,
# d eps / d alpha = 2 CL_alpha / (pi AR) = 0.41, the 172's 3.35 m2 (Jane's: 21.56 + 14.53 sq ft) times 0.59 = 1.98 m2
# against the drawing's 2.70 (FAR's area): its span x0.73, its chords kept (the elevator's power drops with it)
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
TAIL_SPAN_SCALE = 1.98 / 2.70


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
        if pos[1] < 7.0 or abs(rot[0]) < 0.5:          # the wing (the tail sits lower), right-hand (Q_RIGHT)
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


def shrink_tail(parts, k):
    """The horizontal tail's span (and so its area) times k, its chords kept: the B9 panels aft of the wing with a mirror
    partner (the fin has none), their spans, their distances out from the center line and their offsets along their
    parents' spans scaled."""
    tail = set()
    for b in parts:
        pid = cio.part_id(b)
        pos = [float(v) for v in re.search(r"(?m)^\tpos = (.*)$", b).group(1).split(",")]
        if pid.startswith(("B9.Aero", "FARTest.B9")) and pos[2] < -2 and re.search(r"(?m)^\tsym = ", b):
            tail.add(pid)
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
    print(f"  horizontal tail: {len(tail)} panels, span x{k:.3f}")
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
        if pos[1] < 7.0:                                      # the wing only (the tail sits lower)
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
    """Each elevator (a B9 TypeB with FAR's pitch axis, centered on its span, over the two stabilizer strips it is
    attached across: its parent and the parent's inboard strip) made part of those strips, as the wing's flaps are: each
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
        inner = parent_of(parts[by_id[outer]])
        assert outer.startswith("B9.Aero.Wing.Procedural.TypeA") and inner.startswith("B9.Aero.Wing.Procedural.TypeA")
        a_in, a_out = parts[by_id[inner]], parts[by_id[outer]]
        l_in, l_out, l_el = g(a_in, "sharedBaseLength"), g(a_out, "sharedBaseLength"), g(b, "sharedBaseLength")
        assert abs(l_in + l_out - l_el) < 0.05 * l_el, (l_in, l_out, l_el)
        c_r, c_t = g(b, "sharedEdgeWidthTrailingRoot"), g(b, "sharedEdgeWidthTrailingTip")
        c_mid = c_r + (c_t - c_r) * l_in / (l_in + l_out)
        i, j = cio.module_span(b, "FARControllableSurface")
        ctrl = b[i:j]
        area = el_area = 0.0
        for sid, cr, ct in ((inner, c_r, c_mid), (outer, c_mid, c_t)):
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
        print(f"  {inner} + {outer} + {pid}: elevator {el_area / area:.0%} of the strips' area")
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
        if not (pos[1] >= 7.0 or (pos[2] < -2 and re.search(r"(?m)^\tsym = ", b))):     # the wing, or the tail (not the fin)
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


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    split = len(sys.argv) > 2 and sys.argv[2] == "split"
    ship = "Cessna 172M Floatplane Split" if split else "Cessna 172M Floatplane"
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
    parts = merge_wing_controls(parts)
    parts = merge_tail_elevator(parts)
    parts = plain_controls(parts)
    parts = shrink_tail(parts, TAIL_SPAN_SCALE)

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

    parts[0] = cio.add_links(parts[0], links)
    out = head + "".join(parts + new)
    path = os.path.join(here, ship + ".craft")
    open(path, "w", encoding="utf-8", newline="\n").write(out)
    print(f"wrote {path}: {len(parts) + len(new)} parts")


if __name__ == "__main__":
    main()
