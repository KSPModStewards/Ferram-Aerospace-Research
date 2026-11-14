"""Makes SUBOFF Bare Hull.craft: the DARPA SUBOFF bare hull at model scale (DTRC Model 5470: 4.356 m long, 0.508 m
across) as one CustomProceduralParts Spine (Ellipse sections), its radius from Groves, Huang and Chang's equations
(DTRC/SHD-1298-01, 1989, Table 1): the bow 0 to 3.333 ft, the parallel middle body to 10.646 ft, the afterbody to the
aft perpendicular at 13.979 ft and its cap to 14.292 ft. For UnderwaterTests.Tow_SuboffBareHull against Roddy's
captive-model derivatives (DTRC/SHD-1298-08, 1990). And SUBOFF Hull Fins.craft: the hull with its four stern
appendages (Roddy's configuration 5), for Tow_SuboffHullFins: B9 procedural control surfaces (test_sub.py's, cut
down) at 0, 90, 180 and 270 deg, Roddy's Table 1 planform (span 0.438 ft from the hull, root chord 0.704 ft, tip
0.500 ft, a NACA 0020's 20% thickness), their trailing edge at the root 13.146 ft from the nose (Groves), held at
zero (no control axes: a captive test's fixed fins). And SUBOFF Hull Sail.craft: the hull with its sail (Roddy's
configuration 4, Table 1: its leading edge 3.033 ft from the nose, chord 1.208 ft, span 0.729 ft), a B9 control surface
held at zero, for Tow_SuboffHullSail.

    python suboff.py

Built from underwater.py's body (Planing Box.craft's tank part): its middle at craft y 3, local y forward."""
import os
import re
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import underwater  # noqa: E402
from test_boat import set_field  # noqa: E402
import craftio as cio  # noqa: E402

FT = 0.3048
R_MAX = 5.0 / 6.0                 # ft
X_BOW, X_AFT, X_AP, X_END = 3.333333, 10.645833, 13.979167, 14.291667   # ft from the nose
LENGTH = X_END * FT               # 4.356 m overall
LBP = X_AP * FT                   # 4.261 m: Roddy's reference length
MIN_D = 0.01                      # m: a Spine node's least diameter (the nose's and the cap's points)


def radius_ft(x):
    """Groves et al. Table 1: the hull's radius (ft) at x ft from the nose"""
    if x <= 0:
        return 0.0
    if x < X_BOW:
        a = 0.3 * x - 1
        v = 1.126395101 * x * a ** 4 + 0.442874707 * x * x * a ** 3 + 1 - a ** 4 * (1.2 * x + 1)
        return R_MAX * max(v, 0) ** (1 / 2.1)
    if x <= X_AFT:
        return R_MAX
    if x <= X_AP:
        rh, k0, k1 = 0.1175, 10.0, 44.6244
        xi = (X_AP - x) / 3.333333
        v = (rh * rh + rh * k0 * xi ** 2 + (20 - 20 * rh * rh - 4 * rh * k0 - k1 / 3) * xi ** 3 +
             (-45 + 45 * rh * rh + 6 * rh * k0 + k1) * xi ** 4 + (36 - 36 * rh * rh - 4 * rh * k0 - k1) * xi ** 5 +
             (-10 + 10 * rh * rh + rh * k0 + k1 / 3) * xi ** 6)
        return R_MAX * max(v, 0) ** 0.5
    if x < X_END:
        return 0.1175 * R_MAX * max(0.0, 1 - (3.2 * x - 44.733333) ** 2) ** 0.5
    return 0.0


def stations():
    """x (ft) from the nose: cosine-spaced on the bow and the afterbody, the cap's points, the middle body's ends"""
    bow = [X_BOW * (1 - np.cos(np.pi / 2 * i / 14)) for i in range(15)]
    aft = [X_AFT + (X_AP - X_AFT) * (1 - np.cos(np.pi * i / 14)) / 2 for i in range(15)]
    cap = [X_AP + (X_END - X_AP) * f for f in (0.4, 0.7, 0.9, 1.0)]
    return sorted(set(round(x, 6) for x in bow + aft + cap))


HULL_POS = np.array([0.0, 3.0, 0.0])
Q_FUSE = underwater.Q_FUSE
FIN_SPAN = 0.438 * FT
FIN_CHORD = (0.704 * FT, 0.500 * FT)       # root, tip
FIN_TE = 13.146284 * FT                    # m from the nose: the root's trailing edge
FIN_INSET = 0.005                          # m: the root inside the hull


def craft_z(x_m):
    """craft z of a point x m from the nose (the hull's middle at z 0, its nose at +LENGTH/2)"""
    return LENGTH / 2 - x_m


def fin(donor, pid, out, hull_name):
    """a B9 control surface as a SUBOFF stern appendage: its span (local x) along out from the hull, its hinge (its
    leading edge, the origin, at mid-span) the root chord ahead of the root's trailing edge, its chord aft along -y"""
    from test_sub import mat_q   # (test_sub imports this module)
    out = np.array(out, float)
    x_le = FIN_TE - FIN_CHORD[0]
    r = radius_ft(x_le / FT) * FT
    pos = HULL_POS + np.array([0.0, 0.0, craft_z(x_le)]) + out * (r - FIN_INSET + (FIN_SPAN + FIN_INSET) / 2)
    b = cio.place(donor, "B9.Aero.Wing.Procedural.TypeB_%d" % pid, pid, pos, mat_q(out, (0, 0, 1)), (hull_name, HULL_POS, Q_FUSE))
    edge = 0.02
    for key, val in (("sharedBaseLength", FIN_SPAN + FIN_INSET), ("sharedBaseOffsetRoot", 0), ("sharedBaseOffsetTip", 0),
                     ("sharedBaseWidthRoot", FIN_CHORD[0] - edge), ("sharedBaseWidthTip", FIN_CHORD[1] - edge),
                     ("sharedEdgeWidthTrailingRoot", edge), ("sharedEdgeWidthTrailingTip", edge),
                     ("sharedEdgeWidthLeadingRoot", edge), ("sharedEdgeWidthLeadingTip", edge),
                     ("sharedBaseThicknessRoot", 0.2 * FIN_CHORD[0]), ("sharedBaseThicknessTip", 0.2 * FIN_CHORD[1]),
                     ("aeroStatMeanAerodynamicChord", sum(FIN_CHORD) / 2), ("aeroStatSemispan", FIN_SPAN),
                     ("aeroStatMidChordSweep", 0), ("aeroStatTaperRatio", FIN_CHORD[1] / FIN_CHORD[0]), ("maxdeflect", 0),
                     ("pitchaxis", 0), ("yawaxis", 0), ("rollaxis", 0)):
        b = set_field(b, key, "%g" % val)
    return b


SAIL_LE, SAIL_CHORD, SAIL_SPAN = 3.033 * FT, 1.208 * FT, 0.729 * FT    # Roddy Table 1


def sail(donor, pid, hull_name):
    """the sail (Roddy's Table 1: its leading edge 3.033 ft from the nose, chord 1.208 ft, span 0.729 ft, a NACA 0020's
    thickness) as a B9 control surface held at zero, standing up off the hull's top"""
    from test_sub import mat_q   # (test_sub imports this module)
    out = np.array((0.0, 1.0, 0.0))
    r = radius_ft(SAIL_LE / FT) * FT
    pos = HULL_POS + np.array([0.0, 0.0, craft_z(SAIL_LE)]) + out * (r - FIN_INSET + (SAIL_SPAN + FIN_INSET) / 2)
    b = cio.place(donor, "B9.Aero.Wing.Procedural.TypeB_%d" % pid, pid, pos, mat_q(out, (0, 0, 1)), (hull_name, HULL_POS, Q_FUSE))
    edge = 0.02
    for key, val in (("sharedBaseLength", SAIL_SPAN + FIN_INSET), ("sharedBaseOffsetRoot", 0), ("sharedBaseOffsetTip", 0),
                     ("sharedBaseWidthRoot", SAIL_CHORD - edge), ("sharedBaseWidthTip", SAIL_CHORD - edge),
                     ("sharedEdgeWidthTrailingRoot", edge), ("sharedEdgeWidthTrailingTip", edge),
                     ("sharedEdgeWidthLeadingRoot", edge), ("sharedEdgeWidthLeadingTip", edge),
                     ("sharedBaseThicknessRoot", 0.2 * SAIL_CHORD), ("sharedBaseThicknessTip", 0.2 * SAIL_CHORD),
                     ("aeroStatMeanAerodynamicChord", SAIL_CHORD), ("aeroStatSemispan", SAIL_SPAN),
                     ("aeroStatMidChordSweep", 0), ("aeroStatTaperRatio", 1), ("maxdeflect", 0),
                     ("pitchaxis", 0), ("yawaxis", 0), ("rollaxis", 0)):
        b = set_field(b, key, "%g" % val)
    return b


def with_appendages(src, ship, description, fins, with_sail):
    """the bare hull's craft with its four stern appendages and/or its sail on it"""
    head, parts = cio.split_parts(src)
    assert len(parts) == 1
    head = head.replace("ship = SUBOFF Bare Hull", "ship = " + ship)
    head = re.sub(r"(?m)^description = .*$", "description = " + description, head)
    hull = parts[0]
    hull_name = cio.part_id(hull)
    _, donors = cio.split_parts(open(os.path.join(here, "Cessna 172M Floatplane Hinged.craft"), encoding="utf-8").read())
    donor = next(b for b in donors if cio.part_id(b).startswith("B9.Aero.Wing.Procedural.TypeB") and "\t\tyawaxis = 100\n" in b)
    blocks, ids = [], []
    if fins:
        for i, o in enumerate([(0, 1, 0), (1, 0, 0), (0, -1, 0), (-1, 0, 0)]):
            blocks.append(fin(donor, 9101 + i, o, hull_name))
            ids.append(9101 + i)
    if with_sail:
        blocks.append(sail(donor, 9105, hull_name))
        ids.append(9105)
    links = "".join("\tlink = B9.Aero.Wing.Procedural.TypeB_%d\n" % i for i in ids)
    hull, n = re.subn(r"(?m)^\tMODULE$", lambda m: links + m.group(0), hull, count=1)
    assert n == 1, "the hull's first module"
    return head + hull + "".join(blocks)


def main():
    # the equations' joins: the bow and afterbody meet the middle body at R_MAX, the afterbody ends at r_h
    assert abs(radius_ft(X_BOW - 1e-9) - R_MAX) < 1e-3 and abs(radius_ft(X_AFT + 1e-9) - R_MAX) < 1e-3
    assert abs(radius_ft(X_AP) - 0.1175 * R_MAX) < 1e-3, radius_ft(X_AP)
    nodes = []
    for x in reversed(stations()):            # the Spine runs from its tail (position 0) to its nose (1)
        d = max(MIN_D, 2 * radius_ft(x) * FT)
        nodes.append((round(1 - x / X_END, 6), round(d, 5)))
    # volume of the true body (for the test's hand check): integral of pi r^2
    xs = np.linspace(0, X_END, 4001)
    vol = np.trapezoid([np.pi * (radius_ft(x) * FT) ** 2 for x in xs], xs * FT)
    print("SUBOFF: %d nodes, %.3f m long, %.3f m across, volume %.4f m3 (its equations, integrated)"
          % (len(nodes), LENGTH, 2 * R_MAX * FT, vol))
    underwater.BODY_LENGTH, underwater.BODY_NODES = LENGTH, nodes
    _, src = underwater.body()
    src = src.replace("ship = Submerged Body", "ship = SUBOFF Bare Hull")
    src = re.sub(r"(?m)^description = .*$", "description = The DARPA SUBOFF bare hull at model scale (DTRC Model 5470, "
                 "4.356 m by 0.508 m) from Groves et al.'s equations: one ProceduralParts Spine (CustomProceduralParts), "
                 "Ellipse sections, for FAR's underwater tests against Roddy's captive-model derivatives", src)
    out = os.path.join(here, "SUBOFF Bare Hull.craft")
    open(out, "w", encoding="utf-8", newline="\n").write(src)
    print("wrote " + out)
    for ship, desc, fins, with_sail in (
            ("SUBOFF Hull Fins", "The DARPA SUBOFF hull with its four stern appendages (Roddy's configuration 5): B9 "
             "control surfaces held at zero, for FAR's underwater tests", True, False),
            ("SUBOFF Hull Sail", "The DARPA SUBOFF hull with its sail (Roddy's configuration 4): a B9 control surface held "
             "at zero, for FAR's underwater tests", False, True),
            ("SUBOFF Fully Appended", "The DARPA SUBOFF hull with its sail and four stern appendages (Roddy's "
             "configurations 1 and 2, AFF-8): B9 control surfaces held at zero, for FAR's underwater tests", True, True)):
        out = os.path.join(here, ship + ".craft")
        open(out, "w", encoding="utf-8", newline="\n").write(with_appendages(src, ship, desc, fins, with_sail))
        print("wrote " + out)


if __name__ == "__main__":
    main()
