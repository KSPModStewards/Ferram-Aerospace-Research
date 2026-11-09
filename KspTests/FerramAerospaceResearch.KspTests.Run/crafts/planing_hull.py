"""Makes Planing Hull.craft from Planing Box.craft: the same tank part with a CustomProceduralParts Spine shape instead of
the square prism, Rectangle sections 2 m wide, 2 m deep from the transom to 70% of the length, 0.5 m deep at the bow (the
rake is top and bottom alike, so the hull floats the same either way up).

    python planing_hull.py
"""
import os

here = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
src = src.replace("ship = Planing Box", "ship = Planing Hull")
src = src.replace(
    "description = A flat-bottomed box hull for towing on the water: one ProceduralParts square prism, 2 m beam, 8 m long",
    "description = A flat-bottomed hull with a raked bow for towing on the water: one ProceduralParts Spine "
    "(CustomProceduralParts), Rectangle sections, 2 m beam, 8 m long, 2 m deep to 5.6 m, 0.5 m at the bow")
src = src.replace("shapeName = Polygon", "shapeName = Spine")

NODES = [(0, 2, 2), (0.7, 2, 2), (1, 2, 0.5)]
nodes = "".join(
    "\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %g\n\t\t\tsizeH = %g\n\t\t\tsizeV = %g\n\t\t\tsection = Rectangle\n\t\t}\n" % n
    for n in NODES)
spine = ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = 8\n\t\thScale = 1\n"
         "\t\tvScale = 1\n\t\tstagingEnabled = True\n" + nodes +
         "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")
start = src.index("\tMODULE\n\t{\n\t\tname = ProceduralShapePolygon")
end = src.index("\tMODULE\n\t{\n\t\tname = TankContentSwitcher")
src = src[:start] + spine + src[end:]
open(os.path.join(here, "Planing Hull.craft"), "w", encoding="utf-8", newline="\n").write(src)
print("wrote Planing Hull.craft")
