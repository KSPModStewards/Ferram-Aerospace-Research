"""Makes Test Submarine Big.craft: Test Submarine (crafts/test_sub.py, DARPA SUBOFF's fully appended hull) at a real
submarine's size, 40 m long and 4.66 m across (a coastal boat's), driven as a player would build it: Firespitter's
electric propeller (FS1PRE: its motor's length slider at 0, 20 kN), a pusher on the stern, its mount, motor and casing
sunk into the hull and only its spinner and blades behind it (the 3.2 m propeller about 0.7 of the hull's diameter).
The stern is cut where it narrows to the casing (CUT_X) and closed with a short bevel to the spinner. Stock
batteries (Z-1K) and its probe core in the nose. Its ballast: test_sub's, scaled (ore in a nose cone and along the
keel, the hull's own fuel the rest), and two trim tanks (LiquidFuel, half full) fore and aft that SubTests'
demo pumps between to trim it level once it is in the water, as a real boat's trim pumps do (its mass, at this
size and with a motor so much lighter than the test one's, too far from test_sub's to trust the scaled ballast).

    python test_sub_big.py

Craft frame: as test_sub's (the hull's middle at y 3, local y forward)."""
import math
import os
import re
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import suboff  # noqa: E402
import underwater  # noqa: E402
import craftio as cio  # noqa: E402
import test_sub as ts  # noqa: E402
from test_boat import child  # noqa: E402

LP = 40.0                               # m: the whole SUBOFF form's length (its stern cut short of it)
S = LP / suboff.LENGTH                  # from the tank model
K = LP / 8.0                            # from Test Submarine
FT = suboff.FT
# the propeller (Measure_PropGeometry, its part frame, y aft from its mount at slider 0): mount plate and motor
# 1.2 m across to 0.6 m, casing 1.0 m across to 0.7 m, blades 0.8 - 1.0 m (3.2 m across), spinner 0.4 m to 1.2 m
MOUNT_X = 36.65                         # m from the nose: its mount (the hull 0.81 m in radius: the casing inside)
CUT_X = MOUNT_X + 0.72                  # the stern cut just aft of its casing (radius 0.58 m)
END_X, END_D = CUT_X + 0.06, 0.5        # a bevel to the spinner
PROP_T = 0.22                           # t: its motor at slider 0 and two blades
PROBE_X, PROBE_T = 5.0, 0.1
BATTERY_XS, BATTERY_T = (3.0, 3.6, 4.2, 4.8), 0.05
FIN_GUESS_T = 0.4                       # t: each B9 appendage, roughly (the demo trims what this misses)
# t m: its CG as built that much ahead of where these put it (Demo_SubmarineBig, trim tanks even: 0.477 m ahead of its
# center of buoyancy at 502 t), taken off the ballast's moment
MEASURED_NOSE_HEAVY = 0.477 * 502
# the trim tanks (LiquidFuel, half full: SubTests' demo moves it between them), above the keel's ballast
TRIM_FORE_X, TRIM_AFT_X, TRIM_LEN, TRIM_D, TRIM_Y = 14.0, 25.0, 8.0, 2.0, 0.8
TRIM_PIDS = (9021, 9022)
LF_UNITS_KL = ts.FUEL_CAPACITY / 4.13   # Procedural Parts' LiquidFuel: units a kL
TANK_DRY_T_KL = 0.1089


def radius(x):
    """the full-size hull's radius x m from its nose"""
    return suboff.radius_ft(x / S / FT) * FT * S


def z_of(x):
    """craft z of a point x m from the nose: the hull's middle (its END_X long Spine's) at z 0"""
    return END_X / 2 - x


def hull_nodes():
    """the Spine's nodes (position from the tail 0 to the nose 1, diameter): SUBOFF's stations to the cut, then the bevel"""
    xs = [x * FT * S for x in suboff.stations()]
    xs = [x for x in xs if x < CUT_X - 0.05] + [CUT_X]
    nodes = [(round(1 - x / END_X, 6), round(max(suboff.MIN_D, 2 * radius(x)), 5)) for x in reversed(xs)]
    return [(0.0, END_D)] + nodes


def volume_and_centroid():
    xs = np.linspace(0, CUT_X, 8001)
    a = np.array([math.pi * radius(x) ** 2 for x in xs])
    v = np.trapezoid(a, xs)
    return v, np.trapezoid(a * xs, xs) / v


def lf_tank(donor, pid, x, hull_name, units):
    """a Procedural Parts LiquidFuel cylinder (TRIM_D by TRIM_LEN) inside the hull, its axis along it TRIM_Y above the
    hull's, holding units of its capacity"""
    vol = math.pi * TRIM_D ** 2 / 4 * TRIM_LEN
    cap = vol * LF_UNITS_KL
    t = cio.place(donor, "proceduralTankLiquid_%d" % pid, pid, ts.HULL_POS + np.array([0.0, TRIM_Y, z_of(x)]), ts.Q_FUSE,
                  (hull_name, ts.HULL_POS, ts.Q_FUSE))
    t = cio.set_mod(t, "ProceduralPart", "shapeName", "Cylinder")
    t = cio.set_mod(t, "ProceduralShapeCylinder", "diameter", "%.4f" % TRIM_D)
    t = cio.set_mod(t, "ProceduralShapeCylinder", "length", "%.4f" % TRIM_LEN)
    t = cio.set_mod(t, "TankContentSwitcher", "tankVolume", "%.5f" % vol)
    t, n = re.subn(r"\tRESOURCE\n\t\{\n\t\tname = LiquidFuel\n\t\tamount = [^\n]*\n\t\tmaxAmount = [^\n]*\n",
                   "\tRESOURCE\n\t{\n\t\tname = LiquidFuel\n\t\tamount = %.1f\n\t\tmaxAmount = %.1f\n" % (units, cap), t)
    assert n == 1, "the donor's LiquidFuel"
    return t, cap


def main():
    nodes = hull_nodes()
    underwater.BODY_LENGTH, underwater.BODY_NODES = END_X, nodes
    _, src = underwater.body()
    head, parts = cio.split_parts(src)
    head = head.replace("ship = Submerged Body", "ship = Test Submarine Big")
    head = re.sub(r"(?m)^description = .*$", "description = A submarine at a real one's size for FAR's water model: DARPA "
                  "SUBOFF's fully appended hull at 40 m, a Firespitter electric propeller sunk into its stern, trim tanks fore "
                  "and aft (crafts/test_sub_big.py)", head)
    hull = parts[0]
    hull_name = cio.part_id(hull)

    # test_sub's helpers at this size: its scale and length, the ballast's margins and BG scaled
    ts.SCALE, ts.L = S, END_X
    ts.INSET, ts.MARGIN, ts.KEEL_D, ts.BG = 0.01 * K, 0.02 * K, 0.40 * K, 0.04 * K
    nose_aft = ts.NOSE_AFT * K

    vol, x_cb = volume_and_centroid()
    neutral = 0.96 * vol                # (Test Submarine's measured over its form's volume)
    ts.MASS_T = 0.995 * neutral
    tank_dry = TANK_DRY_T_KL * vol
    fin_x = 13.146284 * FT * S
    trim_cap = math.pi * TRIM_D ** 2 / 4 * TRIM_LEN * LF_UNITS_KL
    trim_t = 2 * 0.5 * trim_cap * 0.005
    dry_items = [(tank_dry, 0.0), (PROP_T, z_of(MOUNT_X + 0.4)), (PROBE_T, z_of(PROBE_X)),
                 (BATTERY_T * len(BATTERY_XS), z_of(sum(BATTERY_XS) / len(BATTERY_XS))),
                 (4 * FIN_GUESS_T, z_of(fin_x)), (FIN_GUESS_T, z_of(3.033 * FT * S)),
                 (trim_t / 2, z_of(TRIM_FORE_X)), (trim_t / 2, z_of(TRIM_AFT_X))]
    dry = sum(m for m, _ in dry_items)
    cg_bare = sum(m * z for m, z in dry_items) / dry
    cb = z_of(x_cb)
    trim_moment = (cb - cg_bare) * ts.MASS_T - MEASURED_NOSE_HEAVY
    b = ts.solve_ballast(trim_moment, nose_aft=nose_aft, keel_d=ts.KEEL_D)
    fuel_t = ts.MASS_T - dry - b["m_cone"] - b["m_keel"]
    assert fuel_t > 0, "room for the hull's fuel"
    hull_cap = vol * LF_UNITS_KL
    hull = hull[:hull.rstrip().rindex("}")] + ts.RESOURCE_BLOCK % (fuel_t / 0.005, hull_cap) + hull[hull.rstrip().rindex("}"):]

    _, donors = cio.split_parts(open(os.path.join(here, "Cessna 172M Floatplane Hinged.craft"), encoding="utf-8").read())
    probe = next(d for d in donors if cio.part_id(d).startswith("probeCoreOcto"))
    probe = cio.place(probe, "probeCoreOcto_9001", 9001, ts.HULL_POS + np.array([0.0, 0.0, z_of(PROBE_X)]), ts.Q_FUSE,
                      (hull_name, ts.HULL_POS, ts.Q_FUSE))
    # the propeller turned to face aft, its thrust reversed (Firespitter's pusher switch), its motor at its shortest
    q_aft = cio.qmul(ts.Q_FUSE, (1.0, 0.0, 0.0, 0.0))
    prop = child("FARTestSubMotor_9002", 9002, ts.HULL_POS + np.array([0.0, 0.0, z_of(MOUNT_X)]), q_aft, hull_name)
    modules = ""
    for name, fields in (("FSswitchEngineThrustTransform", "\t\tisReversed = True\n"),
                         ("FSpropellerTweak", "\t\tbladeNumber = 2\n\t\tengineLengthSlider = 0\n")):
        modules += ("\tMODULE\n\t{\n\t\tname = %s\n\t\tisEnabled = True\n%s\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n"
                    "\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n" % (name, fields))
    prop = prop.replace("\tPARTDATA\n\t{\n\t}\n", "\tPARTDATA\n\t{\n\t}\n" + modules, 1)
    assert "isReversed = True" in prop
    batteries = [child("batteryBank_%d" % (9010 + i), 9010 + i, ts.HULL_POS + np.array([0.0, 0.0, z_of(x)]), ts.Q_FUSE, hull_name)
                 for i, x in enumerate(BATTERY_XS)]

    donor = next(d for d in donors if cio.part_id(d).startswith("B9.Aero.Wing.Procedural.TypeB") and "\t\tyawaxis = 100\n" in d)
    app_span, app_chord, app_te = 0.438 * FT * S, (0.704 * FT * S, 0.500 * FT * S), 13.146284 * FT * S
    sail_le, sail_chord, sail_span = 3.033 * FT * S, 1.208 * FT * S, 0.729 * FT * S
    x_app = app_te - app_chord[0]
    apps = [ts.surface(donor, 9003, (0, 1, 0), hull_name, x_app, app_span, app_chord, (0, 100, 0), ts.MAX_DEFLECT),
            ts.surface(donor, 9004, (0, -1, 0), hull_name, x_app, app_span, app_chord, (0, 100, 0), ts.MAX_DEFLECT),
            ts.surface(donor, 9005, (1, 0, 0), hull_name, x_app, app_span, app_chord, (100, 0, 50), ts.MAX_DEFLECT),
            ts.surface(donor, 9006, (-1, 0, 0), hull_name, x_app, app_span, app_chord, (100, 0, 50), ts.MAX_DEFLECT),
            ts.surface(donor, 9007, (0, 1, 0), hull_name, sail_le, sail_span, (sail_chord, sail_chord), (0, 0, 0), 0)]

    tank = next(d for d in donors if cio.part_id(d).startswith("proceduralTankLiquid") and "shapeName = Cylinder" in d)
    ballast = ts.ballast_parts(tank, hull_name, b)
    trims = [lf_tank(tank, pid, x, hull_name, 0.5 * trim_cap)[0] for pid, x in zip(TRIM_PIDS, (TRIM_FORE_X, TRIM_AFT_X))]

    children = [probe, prop] + apps + ballast + trims + batteries
    links = "".join("\tlink = %s\n" % cio.part_id(c) for c in children)
    hull, n = re.subn(r"(?m)^\tMODULE$", lambda m: links + m.group(0), hull, count=1)
    assert n == 1, "the hull's first module"
    out = os.path.join(here, "Test Submarine Big.craft")
    open(out, "w", encoding="utf-8", newline="\n").write(head + hull + "".join(children))
    print("wrote %s: %.2f m long (its form's %.0f m, the stern cut at %.2f m, %.2f m across there), %.2f m across; "
          "volume %.0f m3, its center of buoyancy %+.2f m from the middle" % (out, END_X, LP, CUT_X, 2 * radius(CUT_X),
                                                                         2 * radius(LP / 2), vol, cb))
    print("mass %.0f t (neutral ~%.0f): dry %.1f t, its CG %+.2f m; ballast: nose cone %.2f-%.2f m, %.1f t; keel %.2f m by "
          "%.1f m, %.1f t; the hull's fuel %.1f t; trim tanks %.1f t each of %.1f, %.1f m apart"
          % (ts.MASS_T, neutral, dry, cg_bare, b["x0"], b["nose_aft"], b["m_cone"], b["keel_d"], b["l_keel"], b["m_keel"],
             fuel_t, trim_t / 2, trim_cap * 0.005, TRIM_AFT_X - TRIM_FORE_X))
    print("propeller mount %.2f m from the nose (hull %.2f m across), its blades %.2f - %.2f m aft of the cut"
          % (MOUNT_X, 2 * radius(MOUNT_X), MOUNT_X + 0.8 - CUT_X, MOUNT_X + 1.0 - CUT_X))


if __name__ == "__main__":
    main()
