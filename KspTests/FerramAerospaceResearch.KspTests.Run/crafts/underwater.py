"""Makes the underwater test crafts (FAR's UnderwaterTests):

    python underwater.py body     Submerged Body.craft: Planing Box.craft's tank part with a CustomProceduralParts Spine
                                  body of revolution (Ellipse sections), 8 m long, 1.2 m across, a rounded nose and a
                                  tapered tail: a submarine's hull, held under the water
    python underwater.py rudder   Rudder Rig.craft: a probe core with a stock tail fin (FAR's control surface: its
                                  rudder) hanging below it, its span down and its chord fore and aft: a rudder or keel,
                                  held under the water and in the air

Craft frame: x right, y up, z forward (the SPH's); the probe core lies as the C172's root (local y forward)."""
import os
import re
import sys

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, "..", "..", "..", "..", "planecraft"))
import craftio as cio  # noqa: E402

Q_FUSE = np.array([0.70710678, 0.0, 0.0, 0.70710678])      # local y forward, z down (a root lying in the SPH)
# the tail fin: its span (local +x, away from its attach node) down, its chord (local y) forward
Q_FIN = np.array([0.5, -0.5, -0.5, 0.5])
BODY_LENGTH = 8.0
# Spine nodes from the tail (0) to the nose (1): diameter (m)
BODY_NODES = [(0.0, 0.15), (0.1, 0.6), (0.25, 1.1), (0.35, 1.2), (0.75, 1.2), (0.9, 1.0), (0.97, 0.7), (1.0, 0.3)]
FIN_DROP = 0.25    # m: the fin's root below the probe core's center


def body():
    src = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
    src = src.replace("ship = Planing Box", "ship = Submerged Body")
    src = re.sub(r"(?m)^description = .*$", "description = A streamlined body of revolution for FAR's underwater tests: one "
                 "ProceduralParts Spine (CustomProceduralParts), Ellipse sections, 8 m long, 1.2 m across", src)
    src = src.replace("shapeName = Polygon", "shapeName = Spine")
    nodes = "".join("\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %g\n\t\t\tsizeH = %g\n\t\t\tsizeV = %g\n\t\t\tsection = Ellipse\n\t\t}\n"
                    % (p, d, d) for p, d in BODY_NODES)
    spine = ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = %g\n\t\thScale = 1\n"
             "\t\tvScale = 1\n\t\tstagingEnabled = True\n" % BODY_LENGTH + nodes +
             "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")
    start = src.index("\tMODULE\n\t{\n\t\tname = ProceduralShapePolygon")
    end = src.index("\tMODULE\n\t{\n\t\tname = TankContentSwitcher")
    src = src[:start] + spine + src[end:]
    return "Submerged Body.craft", src


FIN_BLOCK = """PART
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
	autostrutMode = Off
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


def rudder():
    head = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
    head = head[:head.index("\nPART\n") + 1]
    head = head.replace("ship = Planing Box", "ship = Rudder Rig")
    head = re.sub(r"(?m)^description = .*$", "description = A stock tail fin (its rudder FAR's control surface) under a probe "
                  "core, its span down: a rudder or keel for FAR's underwater tests", head)
    _, donors = cio.split_parts(open(os.path.join(here, "Cessna 172M Floatplane Hinged.craft"), encoding="utf-8").read())
    probe = next(b for b in donors if cio.part_id(b).startswith("probeCoreOcto"))
    ppos = np.array([0.0, 10.0, 0.0])
    root = cio.place(probe, "probeCoreOcto_1", 1001, ppos, Q_FUSE)
    root = cio.drop_lines(root, "srfN")
    fpos = ppos + np.array([0.0, -FIN_DROP, 0.0])
    assert np.allclose(cio.qrot(Q_FIN, (1, 0, 0)), (0, -1, 0)) and np.allclose(cio.qrot(Q_FIN, (0, 1, 0)), (0, 0, 1))
    fin = FIN_BLOCK % dict(name="tailfin_2", pid=1002, pos=cio.fmt(fpos), rot=cio.fmt(Q_FIN), parent="probeCoreOcto_1",
                           attpos=cio.fmt(cio.qrot(cio.qinv(Q_FUSE), fpos - ppos)), attrot=cio.fmt(cio.qmul(cio.qinv(Q_FUSE), Q_FIN)))
    root = cio.add_links(root, ["tailfin_2"])
    return "Rudder Rig.craft", head + root + fin


if __name__ == "__main__":
    what = sys.argv[1] if len(sys.argv) > 1 else ""
    for name, text in ([body()] if what == "body" else [rudder()] if what == "rudder" else [body(), rudder()]):
        open(os.path.join(here, name), "w", encoding="utf-8", newline="\n").write(text)
        print("wrote " + name)
