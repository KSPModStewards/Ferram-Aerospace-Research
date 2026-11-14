"""Makes Test Submarine.craft: DARPA SUBOFF's fully appended configuration (AFF-8: hull, sail and four stern
appendages; crafts/suboff.py's hull from Groves et al.'s equations) scaled to 8 m long, as a submarine to drive: the
FAR test electric motor (mods/FARTestFloatplane/BoatMotor.cfg) on its tail, a probe core inside it, its stern
appendages B9 procedural control surfaces (all of each turning, 35 deg either way: rudders up and down, diving planes
left and right, which also roll it against each other) and its sail one held at zero, and ore ballast inside it
(BALLAST) trimming it as built. Roddy's Table 1 planforms (a
NACA 0020's 20% thickness): the appendages' span 0.438 ft, root chord 0.704 ft, tip 0.500 ft, their root's trailing
edge 13.146 ft from the nose (Groves); the sail's leading edge 3.033 ft from the nose, chord 1.208 ft, span 0.729 ft.
SubTests.Sub_Dive drives it and turns it against SUBOFF's turning circles.

    python test_sub.py            Test Submarine.craft
    python test_sub.py prop       Test Submarine Prop.craft: as a player would build it with an electric propeller,
                                  Firespitter's (FSPROpellerElectric, a pusher on its tail: reversed) in place of the test
                                  motor, stock batteries (Z-100) inside it for its power, its ballast solved again for the
                                  heavier motor aft (SubTests.Demo_Submarine's craft=prop)

Craft frame: x right, y up, z forward (the SPH's). The hull lies as Submerged Body.craft has it: its middle at y 3, its
local y forward and local z down (Q_FUSE); the Spine runs from its tail (z -4) to its nose (z +4)."""
import math
import os
import re
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import suboff  # noqa: E402  (puts planecraft on the path through underwater)
import craftio as cio  # noqa: E402
from test_boat import set_field, child  # noqa: E402

SCALE = 8.0 / suboff.LENGTH                       # model (4.356 m) to 8 m
L = suboff.LENGTH * SCALE
HULL_POS = np.array([0.0, 3.0, 0.0])
Q_FUSE = suboff.Q_FUSE
FT = suboff.FT
# The stern appendages' span scaled: the rudders FIN_SPAN_SCALE, the diving planes PLANE_SPAN_SCALE (SUBOFF's: 1, 1).
# SUBOFF's are small, its fully appended hull a little unstable in both planes (Roddy's conclusions: stability indices
# Gv -1.16, Gh -0.44). (Enlarged 2 and 3 while FAR's numerical limiter held a light foil's force in the water under a
# kN: much too stable, turning too sharply on its planes, once it no longer did)
FIN_SPAN_SCALE, PLANE_SPAN_SCALE = 1.0, 1.0
APP_SPAN, APP_CHORD, APP_TE = 0.438 * FT * SCALE * FIN_SPAN_SCALE, (0.704 * FT * SCALE, 0.500 * FT * SCALE), 13.146284 * FT * SCALE
PLANE_SPAN = APP_SPAN * PLANE_SPAN_SCALE / FIN_SPAN_SCALE
SAIL_LE, SAIL_CHORD, SAIL_SPAN = 3.033 * FT * SCALE, 1.208 * FT * SCALE, 0.729 * FT * SCALE
INSET = 0.01                                       # m: a root inside the hull
MOTOR_POS = np.array([0.0, 3.0, -L / 2 - 0.1])     # on the tail, its thrust along the hull's axis
PROBE_POS = np.array([0.0, 3.0, 0.6])              # inside the hull, on its axis, ahead of the middle
MAX_DEFLECT = 35


# the mass SubTests dives and turns it at: 99.5 % of its neutral mass (wholly under: 4.199 t, 41.20 kN, at
# FIN_SPAN_SCALE and PLANE_SPAN_SCALE 1), less its dry mass without the ballast (0.652 t; both measured in SubTests):
# the ballast's ore and the hull's fuel. Its trim: a little light, so it rides awash and dives on its planes
FUEL_CAPACITY = 720.0   # its tank's, ~4.13 kL
NEUTRAL_T, DRY_T = 4.199, 0.652
MASS_T = 0.995 * NEUTRAL_T
LOAD_T = MASS_T - DRY_T

# Its trim. Without ballast (SubTests, trim as built) its CG is CG_BARE ahead of the hull's middle (the hull's fuel
# and its tank's dry mass at the part's origin, the middle; the motor on the tail), its center of buoyancy CB_AHEAD
# (the SUBOFF nose fuller than its tail; larger fins aft move it back), and its CG on its axis (no righting moment
# wholly under).
# Ore ballast inside the hull (Procedural Parts' ore tank: 139.664 units a kL at 10 kg, 0.17 t dry a kL; FAR: a part
# sealed inside another adds no buoyancy) in place of as much fuel: a cone filling the nose (its moment ahead) and a
# cylinder low along the bottom (its CG down), solved for TRIM_MOMENT ahead and its CG BG below its axis. A real
# submarine's: a few % of its diameter (here 3-4 cm), from lead along its keel and trim tanks fore and aft
CG_BARE, CB_AHEAD = -0.069, 0.299
TRIM_MOMENT = (CB_AHEAD - CG_BARE) * MASS_T     # t m
BG = 0.04                       # m
ORE_T_KL = 139.664 * 0.01 + 0.17
MARGIN = 0.02                   # m: the ballast inside the hull's skin
KEEL_D = 0.40                   # m: the keel ballast's diameter, along the bottom
NOSE_AFT = 1.85                 # m from the nose: the nose cone's wide end (the parallel middle body from 1.87 m)
BALLAST_PIDS = (9008, 9009)
# the prop variant's forward trim tank: as a real submarine's, the water (here ore) that trims a heavy stern held
# forward. Its motor on the tail is too heavy for the nose's cone alone (the hull's own fuel, the rest of the ballast,
# sits at its middle: no moment); a cylinder filling the parallel body behind the cone, as long as the moment needs,
# the keel's ballast behind it
TRIM_PID = 9014
# the hull's tank: KSP has no ballast tanks, so its fuel is the ballast. Without a RESOURCE node Procedural Parts fills
# it, far heavier than the water it displaces; on load it keeps a saved amount as it is (TankContentSwitcher)
RESOURCE_BLOCK = ("\tRESOURCE\n\t{\n\t\tname = LiquidFuel\n\t\tamount = %.1f\n\t\tmaxAmount = %.1f\n\t\tflowState = True\n"
                  "\t\tisTweakable = True\n\t\thideFlow = False\n\t\tisVisible = True\n\t\tflowMode = Both\n\t}\n")


def with_fuel(hull, units):
    """the hull's part block with its LiquidFuel (units, 5 kg each) saved, and as its capacity Procedural Parts' own (a
    LiquidFuel tank: 0.1089 t dry a kL, 1600 units a tonne of that: FUEL_CAPACITY), which it recomputes on load (and
    should the two differ, scales the amount by)"""
    assert "\tRESOURCE\n" not in hull
    k = hull.rstrip().rindex("}")
    return hull[:k] + RESOURCE_BLOCK % (units, FUEL_CAPACITY) + hull[k:]


def radius_m(x_m):
    """the scaled hull's radius x m from its nose"""
    return suboff.radius_ft(x_m / SCALE / FT) * FT * SCALE


def frustum(x0, x1, r0, r1):
    """a cone x0 to x1 m from the nose, radius r0 to r1: its volume (m3) and centroid (m from the nose)"""
    l = x1 - x0
    s = r0 * r0 + r0 * r1 + r1 * r1
    return math.pi * l / 3 * s, x0 + l * (r0 * r0 + 2 * r0 * r1 + 3 * r1 * r1) / (4 * s)


# the prop variant: Firespitter's electric propeller on the tail in place of the test motor (MOTOR_T at MOTOR_POS), its
# origin PROP_Z (its mount at the tail's end, the prop aft of it), as light as a player would make it for a submarine:
# its motor's length slider at 0 (0.2 t, 20 kN) and two blades (0.01 t each: PROP_T); BATTERIES Z-100s (BATTERY_T each,
# 100 ElectricCharge: its draw at full power 2 a second) and the probe core (PROBE_T) in the nose, as trim weight
MOTOR_T, PROP_T, PROP_Z = 0.08, 0.22, -L / 2 + 0.05
BATTERY_T, BATTERIES = 0.005, (2.0, 2.15, 2.3, 2.45)
PROBE_T, PROP_PROBE_POS = 0.1, np.array([0.0, 3.0, 2.9])
PROP_KEEL_D = 0.45              # (its keel shorter, behind the trim tank)


def trim_for(prop):
    """(dry mass without ballast, CG_BARE, TRIM_MOMENT, LOAD_T) for the craft, or for its prop variant (the motor's mass
    and place changed, the batteries added: its CG without ballast moved aft)"""
    if not prop:
        return DRY_T, CG_BARE, TRIM_MOMENT, LOAD_T
    moment = (DRY_T * CG_BARE - MOTOR_T * MOTOR_POS[2] + PROP_T * PROP_Z + BATTERY_T * sum(BATTERIES) +
              PROBE_T * (PROP_PROBE_POS[2] - PROBE_POS[2]))
    dry = DRY_T - MOTOR_T + PROP_T + BATTERY_T * len(BATTERIES)
    cg = moment / dry
    return dry, cg, (CB_AHEAD - cg) * MASS_T, MASS_T - dry


def solve_ballast(trim_moment=None, trim_tank=False, nose_aft=NOSE_AFT, keel_d=KEEL_D):
    """the keel cylinder (its length, middle) and nose cone (its narrow end) for BG and TRIM_MOMENT: the keel's weight
    below the axis for BG (its CG at MASS_T), as far forward as the parallel body lets it; the cone's narrow end moved
    forward until the two give the moment ahead (the fuel they replace sits at the middle). trim_tank: the cone full
    (its narrow end at 0.25 m) and a forward trim tank behind it as long as the rest of the moment needs (the keel's
    ballast behind that)"""
    mass = MASS_T
    trim_moment = TRIM_MOMENT if trim_moment is None else trim_moment
    y_keel = radius_m(L / 2) - MARGIN - keel_d / 2          # its axis below the hull's
    m_keel = BG * mass / y_keel
    vol_keel = m_keel / ORE_T_KL
    l_keel = vol_keel / (math.pi * keel_d ** 2 / 4)
    z_keel = L / 2 - nose_aft - l_keel / 2                     # its fore end at the nose ballast's
    assert z_keel - l_keel / 2 > -L / 2 + 2.0, "the keel fits the parallel body"
    need = trim_moment - m_keel * z_keel
    r1 = radius_m(nose_aft) - MARGIN

    def cone(x0):
        r0 = radius_m(x0) - MARGIN
        vol, xc = frustum(x0, nose_aft, r0, r1)
        # inside the hull all along
        assert all(r0 + (r1 - r0) * (x - x0) / (nose_aft - x0) <= radius_m(x) - MARGIN + 1e-6
                   for x in [x0 + (nose_aft - x0) * j / 40 for j in range(41)]), x0
        return r0, vol, xc, vol * ORE_T_KL * (L / 2 - xc)

    lo, hi = 0.25, nose_aft - 0.1
    if trim_tank and cone(lo)[3] < need:   # (the cone alone short of it)
        r0, vol_cone, xc, m_cone_moment = cone(lo)
        r_t = radius_m(L / 2) - MARGIN
        area = math.pi * r_t * r_t

        def moment(l_t):
            z_t = L / 2 - nose_aft - l_t / 2
            z_k = L / 2 - nose_aft - l_t - l_keel / 2
            return m_cone_moment + area * l_t * ORE_T_KL * z_t + m_keel * z_k

        # (its moment peaks: past it the keel goes aft faster than the tank adds) the shortest that balances it
        steps = [j * 0.02 for j in range(int(L / 2 / 0.02))]
        b_ = next((s for s in steps if moment(s) >= trim_moment), None)
        assert b_ is not None, "a trim tank balances it (at most %.2f t m of %.2f)" % (max(map(moment, steps)), trim_moment)
        a = max(0.0, b_ - 0.02)
        for _ in range(60):
            mid = (a + b_) / 2
            a, b_ = (mid, b_) if moment(mid) < trim_moment else (a, mid)
        l_t = a
        z_keel = L / 2 - nose_aft - l_t - l_keel / 2
        assert z_keel - l_keel / 2 > -L / 2 + 2.0, "the keel fits the parallel body"
        return dict(nose_aft=nose_aft, keel_d=keel_d, x0=lo, r0=r0, r1=r1, vol_cone=vol_cone, m_cone=vol_cone * ORE_T_KL,
                    z_cone=L / 2 - xc, l_keel=l_keel, vol_keel=vol_keel, m_keel=m_keel, z_keel=z_keel, y_keel=-y_keel,
                    l_trim=l_t, r_trim=r_t, vol_trim=area * l_t, m_trim=area * l_t * ORE_T_KL, z_trim=L / 2 - nose_aft - l_t / 2)
    assert cone(lo)[3] >= need, "the nose holds enough"
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if cone(mid)[3] > need else (lo, mid)
    r0, vol_cone, xc, moment = cone(lo)
    return dict(nose_aft=nose_aft, keel_d=keel_d, x0=lo, r0=r0, r1=r1, vol_cone=vol_cone, m_cone=vol_cone * ORE_T_KL,
                z_cone=L / 2 - xc, l_keel=l_keel, vol_keel=vol_keel, m_keel=m_keel, z_keel=z_keel, y_keel=-y_keel, m_trim=0.0)


def ballast_parts(donor, hull_name, b):
    """the two ore tanks, the Procedural Parts liquid tank (donor) made an ore one, full, surface-attached inside the
    hull; along its axis (Q_FUSE: local y forward, so the cone's top, its narrow end, to the nose)"""
    def ore(pid, pos, shape, fields, vol):
        t = cio.place(donor, "proceduralTankOre_%d" % pid, pid, pos, Q_FUSE, (hull_name, HULL_POS, Q_FUSE))
        t = cio.set_mod(t, "ProceduralPart", "shapeName", shape)
        for key, val in fields:
            t = cio.set_mod(t, "ProceduralShape" + shape, key, "%.4f" % val)
        t = cio.set_mod(t, "TankContentSwitcher", "tankType", "Ore")
        t = cio.set_mod(t, "TankContentSwitcher", "tankVolume", "%.5f" % vol)
        units = vol * 139.664
        t, n = re.subn(r"\tRESOURCE\n\t\{\n\t\tname = LiquidFuel\n\t\tamount = [^\n]*\n\t\tmaxAmount = [^\n]*\n",
                       "\tRESOURCE\n\t{\n\t\tname = Ore\n\t\tamount = %.3f\n\t\tmaxAmount = %.3f\n" % (units, units), t)
        assert n == 1, "the donor's LiquidFuel"
        return t
    l_cone = b["nose_aft"] - b["x0"]
    cone = ore(BALLAST_PIDS[0], HULL_POS + np.array([0.0, 0.0, L / 2 - (b["x0"] + b["nose_aft"]) / 2]), "Cone",
               (("topDiameter", 2 * b["r0"]), ("bottomDiameter", 2 * b["r1"]), ("length", l_cone)), b["vol_cone"])
    keel = ore(BALLAST_PIDS[1], HULL_POS + np.array([0.0, b["y_keel"], b["z_keel"]]), "Cylinder",
               (("diameter", b["keel_d"]), ("length", b["l_keel"])), b["vol_keel"])
    if b["m_trim"] <= 0:
        return [cone, keel]
    trim = ore(TRIM_PID, HULL_POS + np.array([0.0, 0.0, b["z_trim"]]), "Cylinder",
               (("diameter", 2 * b["r_trim"]), ("length", b["l_trim"])), b["vol_trim"])
    return [cone, keel, trim]


def mat_q(x, y):
    """the rotation (x, y, z, w) taking local x and y to the craft's x and y given"""
    x, y = np.array(x, float), np.array(y, float)
    z = np.cross(x, y)
    m = np.column_stack([x, y, z])
    w = np.sqrt(max(0.0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    qx = np.copysign(np.sqrt(max(0.0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2, m[2, 1] - m[1, 2])
    qy = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2, m[0, 2] - m[2, 0])
    qz = np.copysign(np.sqrt(max(0.0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2, m[1, 0] - m[0, 1])
    q = np.array([qx, qy, qz, w])
    assert np.allclose(cio.qrot(q, (1, 0, 0)), x) and np.allclose(cio.qrot(q, (0, 1, 0)), y), (x, y, q)
    return q


def surface(donor, pid, out, hull_name, x_le, span, chord, axes, maxdeflect):
    """a B9 control surface standing out of the hull along out: its hinge (its leading edge, the origin, at
    mid-span) x_le m from the nose at the root, its span (local x) from just inside the hull, its chord aft (-y);
    axes its FAR pitch, yaw and roll responses"""
    out = np.array(out, float)
    r = radius_m(x_le)
    pos = HULL_POS + np.array([0.0, 0.0, L / 2 - x_le]) + out * (r - INSET + (span + INSET) / 2)
    b = cio.place(donor, "B9.Aero.Wing.Procedural.TypeB_%d" % pid, pid, pos, mat_q(out, (0, 0, 1)), (hull_name, HULL_POS, Q_FUSE))
    edge = 0.02 * SCALE
    pitch, yaw, roll = axes
    for key, val in (("sharedBaseLength", span + INSET), ("sharedBaseOffsetRoot", 0), ("sharedBaseOffsetTip", 0),
                     ("sharedBaseWidthRoot", chord[0] - edge), ("sharedBaseWidthTip", chord[1] - edge),
                     ("sharedEdgeWidthTrailingRoot", edge), ("sharedEdgeWidthTrailingTip", edge),
                     ("sharedEdgeWidthLeadingRoot", edge), ("sharedEdgeWidthLeadingTip", edge),
                     ("sharedBaseThicknessRoot", 0.2 * chord[0]), ("sharedBaseThicknessTip", 0.2 * chord[1]),
                     ("aeroStatMeanAerodynamicChord", sum(chord) / 2), ("aeroStatSemispan", span),
                     ("aeroStatMidChordSweep", 0), ("aeroStatTaperRatio", chord[1] / chord[0]), ("maxdeflect", maxdeflect),
                     ("pitchaxis", pitch), ("yawaxis", yaw), ("rollaxis", roll)):
        b = set_field(b, key, "%g" % val)
    return b


def main(prop=False):
    src = open(os.path.join(here, "SUBOFF Bare Hull.craft"), encoding="utf-8").read()
    head, parts = cio.split_parts(src)
    assert len(parts) == 1
    ship = "Test Submarine Prop" if prop else "Test Submarine"
    head = head.replace("ship = SUBOFF Bare Hull", "ship = " + ship)
    head = re.sub(r"(?m)^description = .*$", "description = A test submarine for FAR's water model: DARPA SUBOFF (AFF-8: hull, "
                  "sail, four stern appendages) scaled to 8 m, an electric motor on its tail (FARTest_BoatMotor), its stern "
                  "appendages B9 control surfaces (rudders, diving planes) and a probe core", head)
    hull = parts[0]
    # the Spine scaled: its length and every node's size
    hull = re.sub(r"(?m)^(\t\tlength = )(.*)$", lambda m: m.group(1) + "%g" % L, hull, count=1)
    hull = re.sub(r"(?m)^(\t\t\tsize[HV] = )(.*)$", lambda m: m.group(1) + "%g" % (float(m.group(2)) * SCALE), hull)
    hull_name = cio.part_id(hull)
    _, donors = cio.split_parts(open(os.path.join(here, "Cessna 172M Floatplane Hinged.craft"), encoding="utf-8").read())
    probe = next(b for b in donors if cio.part_id(b).startswith("probeCoreOcto"))
    probe = cio.place(probe, "probeCoreOcto_9001", 9001, PROP_PROBE_POS if prop else PROBE_POS, Q_FUSE, (hull_name, HULL_POS, Q_FUSE))
    dry, cg_bare, trim_moment, load_t = trim_for(prop)
    extra = []
    if prop:
        # the propeller turned to face aft (about its x: its prop, ahead of its mount, behind the tail), its thrust
        # reversed (Firespitter's pusher switch, saved): it pushes the hull ahead
        q_aft = cio.qmul(Q_FUSE, (1.0, 0.0, 0.0, 0.0))
        motor = child("FSPROpellerElectric_9002", 9002, np.array([0.0, 3.0, PROP_Z]), q_aft, hull_name)
        motor = motor.replace("\tPARTDATA\n\t{\n\t}\n", "\tPARTDATA\n\t{\n\t}\n\tMODULE\n\t{\n\t\tname = FSpropellerTweak\n"
                              "\t\tisEnabled = True\n\t\tbladeNumber = 2\n\t\tengineLengthSlider = 0\n\t\tEVENTS\n\t\t{\n\t\t}\n"
                              "\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n", 1)
        motor = motor.replace("\tPARTDATA\n\t{\n\t}\n", "\tPARTDATA\n\t{\n\t}\n\tMODULE\n\t{\n\t\tname = FSswitchEngineThrustTransform\n"
                              "\t\tisEnabled = True\n\t\tisReversed = True\n\t\tstagingEnabled = True\n\t\tEVENTS\n\t\t{\n\t\t}\n"
                              "\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n", 1)
        assert "isReversed = True" in motor
        extra = [child("batteryPack_%d" % (9010 + i), 9010 + i, np.array([0.0, 3.0, z]), Q_FUSE, hull_name)
                 for i, z in enumerate(BATTERIES)]
    else:
        motor = child("FARTest.BoatMotor_9002", 9002, MOTOR_POS, Q_FUSE, hull_name)
    donor = next(b for b in donors if cio.part_id(b).startswith("B9.Aero.Wing.Procedural.TypeB") and "\t\tyawaxis = 100\n" in b)
    x_app = APP_TE - APP_CHORD[0]
    # the rudders yaw; the planes pitch, and roll it against each other; the sail is held
    apps = [surface(donor, 9003, (0, 1, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (0, 100, 0), MAX_DEFLECT),
            surface(donor, 9004, (0, -1, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (0, 100, 0), MAX_DEFLECT),
            surface(donor, 9005, (1, 0, 0), hull_name, x_app, PLANE_SPAN, APP_CHORD, (100, 0, 50), MAX_DEFLECT),
            surface(donor, 9006, (-1, 0, 0), hull_name, x_app, PLANE_SPAN, APP_CHORD, (100, 0, 50), MAX_DEFLECT),
            surface(donor, 9007, (0, 1, 0), hull_name, SAIL_LE, SAIL_SPAN, (SAIL_CHORD, SAIL_CHORD), (0, 0, 0), 0)]
    b = solve_ballast(trim_moment, trim_tank=prop, keel_d=PROP_KEEL_D if prop else KEEL_D)
    tank = next(d for d in donors if cio.part_id(d).startswith("proceduralTankLiquid") and "shapeName = Cylinder" in d)
    ballast = ballast_parts(tank, hull_name, b)
    fuel_units = (load_t - b["m_cone"] - b["m_keel"] - b["m_trim"]) / 0.005
    assert 0 < fuel_units < FUEL_CAPACITY
    ids = ["probeCoreOcto_9001", cio.part_id(motor)] + ["B9.Aero.Wing.Procedural.TypeB_%d" % i for i in range(9003, 9008)]
    ids += [cio.part_id(x) for x in ballast] + [cio.part_id(e) for e in extra]
    links = "".join("\tlink = %s\n" % c for c in ids)
    hull, n = re.subn(r"(?m)^\tMODULE$", lambda m: links + m.group(0), hull, count=1)
    assert n == 1, "the hull's first module"
    out = os.path.join(here, ship + ".craft")
    open(out, "w", encoding="utf-8", newline="\n").write(head + with_fuel(hull, fuel_units) + probe + motor + "".join(apps) +
                                                       "".join(ballast) + "".join(extra))
    print("wrote %s: %.2f m long, %.3f m across; appendages span %.3f m, chord %.3f/%.3f m; sail %.3f by %.3f m"
          % (out, L, 2 * radius_m(L / 2), APP_SPAN, APP_CHORD[0], APP_CHORD[1], SAIL_CHORD, SAIL_SPAN))
    print("trim: dry %.3f t without ballast, its CG %+.3f m ahead of the middle; trim moment %.3f t m" % (dry, cg_bare, trim_moment))
    if b["m_trim"] > 0:
        print("trim tank: %.2f m long behind the nose cone, %.3f m across, %.3f t at z %+.3f" % (b["l_trim"], 2 * b["r_trim"], b["m_trim"], b["z_trim"]))
    print("ballast: nose cone %.2f-%.2f m from the nose, %.3f-%.3f m across, %.3f t at z %+.3f; keel %.3f m by %.2f m at "
          "y %+.3f z %+.3f, %.3f t; fuel %.1f units (%.3f t)"
          % (b["x0"], b["nose_aft"], 2 * b["r0"], 2 * b["r1"], b["m_cone"], b["z_cone"], b["keel_d"], b["l_keel"], b["y_keel"],
             b["z_keel"], b["m_keel"], fuel_units, fuel_units * 0.005))


if __name__ == "__main__":
    main(prop=len(sys.argv) > 1 and sys.argv[1] == "prop")
