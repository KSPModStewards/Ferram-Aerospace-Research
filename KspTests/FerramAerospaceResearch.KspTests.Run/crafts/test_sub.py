"""Makes Test Submarine.craft: DARPA SUBOFF's fully appended configuration (AFF-8: hull, sail and four stern
appendages; crafts/suboff.py's hull from Groves et al.'s equations) scaled to 8 m long, as a submarine to drive: the
FAR test electric motor (mods/FARTestFloatplane/BoatMotor.cfg) on its tail, a probe core inside it, its stern
appendages B9 procedural control surfaces (all of each turning, 35 deg either way: rudders up and down, diving planes
left and right, which also roll it against each other) and its sail one held at zero. Roddy's Table 1 planforms (a
NACA 0020's 20% thickness): the appendages' span 0.438 ft, root chord 0.704 ft, tip 0.500 ft, their root's trailing
edge 13.146 ft from the nose (Groves); the sail's leading edge 3.033 ft from the nose, chord 1.208 ft, span 0.729 ft.
SubTests.Sub_Dive drives it and turns it against SUBOFF's turning circles.

    python test_sub.py

Craft frame: x right, y up, z forward (the SPH's). The hull lies as Submerged Body.craft has it: its middle at y 3, its
local y forward and local z down (Q_FUSE); the Spine runs from its tail (z -4) to its nose (z +4)."""
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
APP_SPAN, APP_CHORD, APP_TE = 0.438 * FT * SCALE, (0.704 * FT * SCALE, 0.500 * FT * SCALE), 13.146284 * FT * SCALE
SAIL_LE, SAIL_CHORD, SAIL_SPAN = 3.033 * FT * SCALE, 1.208 * FT * SCALE, 0.729 * FT * SCALE
INSET = 0.01                                       # m: a root inside the hull
MOTOR_POS = np.array([0.0, 3.0, -L / 2 - 0.1])     # on the tail, its thrust along the hull's axis
PROBE_POS = np.array([0.0, 3.0, 0.6])              # inside the hull, on its axis, ahead of the middle
MAX_DEFLECT = 35


def radius_m(x_m):
    """the scaled hull's radius x m from its nose"""
    return suboff.radius_ft(x_m / SCALE / FT) * FT * SCALE


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


def main():
    src = open(os.path.join(here, "SUBOFF Bare Hull.craft"), encoding="utf-8").read()
    head, parts = cio.split_parts(src)
    assert len(parts) == 1
    head = head.replace("ship = SUBOFF Bare Hull", "ship = Test Submarine")
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
    probe = cio.place(probe, "probeCoreOcto_9001", 9001, PROBE_POS, Q_FUSE, (hull_name, HULL_POS, Q_FUSE))
    motor = child("FARTest.BoatMotor_9002", 9002, MOTOR_POS, Q_FUSE, hull_name)
    donor = next(b for b in donors if cio.part_id(b).startswith("B9.Aero.Wing.Procedural.TypeB") and "\t\tyawaxis = 100\n" in b)
    x_app = APP_TE - APP_CHORD[0]
    # the rudders yaw; the planes pitch, and roll it against each other; the sail is held
    apps = [surface(donor, 9003, (0, 1, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (0, 100, 0), MAX_DEFLECT),
            surface(donor, 9004, (0, -1, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (0, 100, 0), MAX_DEFLECT),
            surface(donor, 9005, (1, 0, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (100, 0, 50), MAX_DEFLECT),
            surface(donor, 9006, (-1, 0, 0), hull_name, x_app, APP_SPAN, APP_CHORD, (100, 0, 50), MAX_DEFLECT),
            surface(donor, 9007, (0, 1, 0), hull_name, SAIL_LE, SAIL_SPAN, (SAIL_CHORD, SAIL_CHORD), (0, 0, 0), 0)]
    ids = ["probeCoreOcto_9001", "FARTest.BoatMotor_9002"] + ["B9.Aero.Wing.Procedural.TypeB_%d" % i for i in range(9003, 9008)]
    links = "".join("\tlink = %s\n" % c for c in ids)
    hull, n = re.subn(r"(?m)^\tMODULE$", lambda m: links + m.group(0), hull, count=1)
    assert n == 1, "the hull's first module"
    out = os.path.join(here, "Test Submarine.craft")
    open(out, "w", encoding="utf-8", newline="\n").write(head + hull + probe + motor + "".join(apps))
    print("wrote %s: %.2f m long, %.3f m across; appendages span %.3f m, chord %.3f/%.3f m; sail %.3f by %.3f m"
          % (out, L, 2 * radius_m(L / 2), APP_SPAN, APP_CHORD[0], APP_CHORD[1], SAIL_CHORD, SAIL_SPAN))


if __name__ == "__main__":
    main()
