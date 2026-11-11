"""Makes Test Boat.craft: the Planing Hull (8 m by 2 m, 2 m deep, its bow raked) with a probe core on its deck, the FAR
test electric motor (mods/FARTestFloatplane/BoatMotor.cfg) under its stern and a spade rudder hanging below the stern
ahead of it (a B9 procedural control surface, the Cessna's rudder cut down: FAR's control surface, all of it turning,
0.6 m deep, 0.5 m chord at the hull and 0.4 m at its foot, 0.27 m2, 30 deg either way): a boat to drive about the water
by hand or in BoatTests.

    python test_boat.py

Craft frame: x right, y up, z forward (the SPH's). The hull lies as Planing Hull.craft has it: its center at y 3, its
local y forward and local z down (Q_FUSE), so its keel is at y 2 and its transom at z -4."""
import os
import re
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, "..", "..", "..", "..", "planecraft"))
import craftio as cio  # noqa: E402
from underwater import FIN_BLOCK, Q_FUSE  # noqa: E402

HULL_POS = np.array([0.0, 3.0, 0.0])
KEEL_Y = 2.0
PROBE_POS = np.array([0.0, 4.15, 1.0])          # on the deck, ahead of the middle
MOTOR_POS = np.array([0.0, KEEL_Y - 0.05, -3.7])  # under the hull at the stern, its thrust line just below the keel
# the rudder: a B9 control surface's origin is its hinge (its leading edge) at mid-span, its span along its local x,
# its chord aft of the hinge along its local -y
RUDDER_SPAN, RUDDER_CHORD = 0.6, (0.5, 0.4)      # root (at the hull), tip
RUDDER_POS = np.array([0.0, KEEL_Y - 0.02 - RUDDER_SPAN / 2, -2.7])  # its leading edge 0.8 m ahead of its trailing one, ahead of the motor
Q_RUDDER = np.array([0.5, -0.5, -0.5, 0.5])     # local x down (its root at the hull), local y forward


def child(name, pid, pos, rot, parent):
    return FIN_BLOCK % dict(name=name, pid=pid, pos=cio.fmt(pos), rot=cio.fmt(rot), parent=parent,
                            attpos=cio.fmt(cio.qrot(cio.qinv(Q_FUSE), pos - HULL_POS)),
                            attrot=cio.fmt(cio.qmul(cio.qinv(Q_FUSE), rot)))


RUDDER_ID = "B9.Aero.Wing.Procedural.TypeB_9003"


def set_field(block, key, val):
    block, n = re.subn(r"(?m)^(\t\t%s = ).*$" % re.escape(key), lambda m: m.group(1) + val, block, count=1)
    assert n == 1, key
    return block


def spade_rudder(donors, hull_name):
    """the Cessna's rudder (its B9 control surface, yawing) cut down to a boat's spade rudder, hanging below the hull"""
    assert np.allclose(cio.qrot(Q_RUDDER, (1, 0, 0)), (0, -1, 0)) and np.allclose(cio.qrot(Q_RUDDER, (0, 1, 0)), (0, 0, 1))
    b = next(b for b in donors if cio.part_id(b).startswith("B9.Aero.Wing.Procedural.TypeB") and "\t\tyawaxis = 100\n" in b)
    b = cio.place(b, RUDDER_ID, 9003, RUDDER_POS, Q_RUDDER, (hull_name, HULL_POS, Q_FUSE))
    # a control surface's chord is its base and trailing edge widths (its rounded leading edge, 0.24 m, ahead of the hinge
    # line, not counted); its span centered on the hinge's middle
    edge = 0.12
    for key, val in (("sharedBaseLength", RUDDER_SPAN), ("sharedBaseOffsetRoot", 0), ("sharedBaseOffsetTip", 0),
                     ("sharedBaseWidthRoot", RUDDER_CHORD[0] - edge), ("sharedBaseWidthTip", RUDDER_CHORD[1] - edge),
                     ("sharedEdgeWidthTrailingRoot", edge), ("sharedEdgeWidthTrailingTip", edge),
                     ("sharedBaseThicknessRoot", 0.06), ("sharedBaseThicknessTip", 0.05),
                     ("aeroStatMeanAerodynamicChord", sum(RUDDER_CHORD) / 2), ("aeroStatSemispan", RUDDER_SPAN),
                     ("aeroStatMidChordSweep", 0), ("aeroStatTaperRatio", 1), ("maxdeflect", 30)):
        b = set_field(b, key, "%g" % val)
    return b


def main():
    src = open(os.path.join(here, "Planing Hull.craft"), encoding="utf-8-sig").read().replace("\r\n", "\n")
    head, parts = cio.split_parts(src)
    assert len(parts) == 1
    head = head.replace("ship = Planing Hull", "ship = Test Boat")
    head = "\n".join("description = A test boat for FAR's water model: the Planing Hull with an electric motor under its stern "
                     "(FARTest_BoatMotor, 10 kN), a spade rudder (a B9 control surface) and a probe core to steer it"
                     if line.startswith("description = ") else line for line in head.split("\n"))
    hull = parts[0]
    hull_name = cio.part_id(hull)
    _, donors = cio.split_parts(open(os.path.join(here, "Cessna 172M Floatplane Hinged.craft"), encoding="utf-8").read())
    probe = next(b for b in donors if cio.part_id(b).startswith("probeCoreOcto"))
    probe = cio.place(probe, "probeCoreOcto_9001", 9001, PROBE_POS, Q_FUSE, (hull_name, HULL_POS, Q_FUSE))
    motor = child("FARTest.BoatMotor_9002", 9002, MOTOR_POS, Q_FUSE, hull_name)
    rudder = spade_rudder(donors, hull_name)
    # the links at the part's level, before its first module (Planing Box.craft has no part-level EVENTS, where
    # craftio's add_links would put them)
    links = "".join("\tlink = %s\n" % c for c in ("probeCoreOcto_9001", "FARTest.BoatMotor_9002", RUDDER_ID))
    hull, n = re.subn(r"(?m)^\tMODULE$", lambda m: links + m.group(0), hull, count=1)
    assert n == 1, "the hull's first module"
    out = os.path.join(here, "Test Boat.craft")
    open(out, "w", encoding="utf-8", newline="\n").write(head + hull + probe + motor + rudder)
    print("wrote " + out)


if __name__ == "__main__":
    main()
