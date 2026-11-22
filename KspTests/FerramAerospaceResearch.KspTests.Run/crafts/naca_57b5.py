"""Makes NACA 57-B-5.craft: one float of the Cessna 172 floatplane (edo_float.py: NACA model 57-B-5's lines, TN 716,
at the 172's float size, 2.128 ft beam over the spray strips) as one CustomProceduralParts Spine on its own, to tow it
as NACA towed the model free to trim (NacaHullTests.Tow_Naca57B5FreeToTrim).

The Spine runs along the base line (the deck at the centre line), bow at its top end; its middle is 47 model inches
from the bow, the step 52.

    python naca_57b5.py      writes NACA 57-B-5.craft beside this script
"""
import os

import edo_float
from spine_loft import spine_module


def main():
    s = edo_float
    here = os.path.dirname(os.path.abspath(__file__))
    src = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
    ship = "NACA 57-B-5"
    src = src.replace("ship = Planing Box", "ship = " + ship)
    src = src.replace(
        "description = A flat-bottomed box hull for towing on the water: one ProceduralParts square prism, 2 m beam, 8 m long",
        "description = NACA float model 57-B-5 (TN 716) at the Cessna 172 floatplane's float size (2.128 ft beam over "
        "the spray strips): one ProceduralParts Spine (CustomProceduralParts), its stations embedded as custom sections")
    src = src.replace("shapeName = Polygon", "shapeName = Spine")
    spine = spine_module(s.STATIONS, s.LENGTH_IN, s.SX, s.SY, s.SZ, "NACA57B5")
    start = src.index("\tMODULE\n\t{\n\t\tname = ProceduralShapePolygon")
    end = src.index("\tMODULE\n\t{\n\t\tname = TankContentSwitcher")
    src = src[:start] + spine + src[end:]
    open(os.path.join(here, ship + ".craft"), "w", encoding="utf-8", newline="\n").write(src)
    print("wrote %s.craft: %.3f m long, beam over the spray strips %.4f m, %d stations"
          % (ship, s.LENGTH_IN * s.SX, 12.47 * s.SX, len(s.STATIONS)))


if __name__ == "__main__":
    main()
