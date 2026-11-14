"""Makes the Spine planing hulls from Planing Box.craft: the same tank part with a CustomProceduralParts Spine shape
instead of the square prism, 2 m wide, 8 m long, 2 m deep from the transom to 70% of the length, shallower at the bow
(the bow node is centered, so the bottom rakes up to it).

    python planing_hull.py      Planing Hull.craft: Rectangle sections, 0.5 m deep at the bow (top and bottom raked
                                alike, so it floats the same either way up)
    python planing_hull.py v    Planing V Hull.craft: a 20 deg V bottom with vertical sides (a custom section embedded
                                in the craft), 0.6 m deep at the bow
"""
import math
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
v_hull = len(sys.argv) > 1 and sys.argv[1] == "v"
src = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
if v_hull:
    ship, out = "Planing V Hull", "Planing V Hull.craft"
    desc = ("A 20 deg V-bottomed hull with a raked bow for towing on the water: one ProceduralParts Spine "
            "(CustomProceduralParts), its section embedded, 2 m beam, 8 m long, 2 m deep to 5.6 m, 0.6 m at the bow")
    section, bow_depth = "VHull20", 0.6
    chine = -1 + math.tan(math.radians(20))   # unit box: half-beam 1, keel at -1
    custom = ("\t\tCROSS_SECTION\n\t\t{\n\t\t\tname = VHull20\n" +
              "".join("\t\t\tv = %g,%g\n" % p for p in [(0, -1), (1, round(chine, 5)), (1, 1), (-1, 1), (-1, round(chine, 5))]) +
              "\t\t}\n")
else:
    ship, out = "Planing Hull", "Planing Hull.craft"
    desc = ("A flat-bottomed hull with a raked bow for towing on the water: one ProceduralParts Spine "
            "(CustomProceduralParts), Rectangle sections, 2 m beam, 8 m long, 2 m deep to 5.6 m, 0.5 m at the bow")
    section, bow_depth, custom = "Rectangle", 0.5, ""

src = src.replace("ship = Planing Box", "ship = " + ship)
src = src.replace(
    "description = A flat-bottomed box hull for towing on the water: one ProceduralParts square prism, 2 m beam, 8 m long",
    "description = " + desc)
src = src.replace("shapeName = Polygon", "shapeName = Spine")

NODES = [(0, 2, 2), (0.7, 2, 2), (1, 2, bow_depth)]
nodes = "".join(
    "\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %g\n\t\t\tsizeH = %g\n\t\t\tsizeV = %g\n\t\t\tsection = %s\n\t\t}\n" % (n + (section,))
    for n in NODES)
spine = ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = 8\n\t\thScale = 1\n"
         "\t\tvScale = 1\n\t\tstagingEnabled = True\n" + custom + nodes +
         "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")
start = src.index("\tMODULE\n\t{\n\t\tname = ProceduralShapePolygon")
end = src.index("\tMODULE\n\t{\n\t\tname = TankContentSwitcher")
src = src[:start] + spine + src[end:]
open(os.path.join(here, out), "w", encoding="utf-8", newline="\n").write(src)
print("wrote " + out)
