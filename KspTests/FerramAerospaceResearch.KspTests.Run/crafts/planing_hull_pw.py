"""Makes Planing Hull PW.craft: Planing Hull.craft (planing_hull.py: one Spine, Rectangle sections, 2 m beam, 8 m long,
2 m deep to 5.6 m from the transom, 0.5 m deep at the bow, top and bottom raked alike) built of ten flat B9 hull plates
instead (b9_plates.py): its faces are flat, so the plates are a true replica of it: the bottom and the deck (each its
flat run and its rake to the bow), each side (its rectangle and its trapezoid to the bow), the transom, the bow.

The root is a small Spine tank of Planing Hull's own part (Rectangle, BALLAST m square, BALLAST_LENGTH long) at the hull's
middle, turned as Planing Hull's Spine is (its y the bow), so the tow reads the hull in the same frame and fills its
liquid fuel to the hull's mass; the plates are surface-attached in a tree from it.

    python planing_hull_pw.py     writes Planing Hull PW.craft beside this script
"""
import os
import re

import b9_plates as b9

here = os.path.dirname(os.path.abspath(__file__))
FRAME_POS = (0.0, 3.0, 0.0)
FRAME_ROT = (0.707106829, 0.0, 0.0, 0.707106829)
L, B, D, BOW = 8.0, 2.0, 2.0, 0.5       # length, beam, depth, bow depth (m)
KNUCKLE = 0.7                           # the bow's rake starts at this share of the length from the transom
BALLAST, BALLAST_LENGTH = 1.2, 3.2
MASS_MULTIPLIER = 0.3                   # FAR's wing mass x this (the hull reaches 4 t in its tank's fuel)


def plates():
    """The hull's faces as plates, in the Spine's frame: x across (+-B/2), y along (transom -L/2, bow +L/2), z up."""
    y0, yk, y1 = -L / 2, -L / 2 + KNUCKLE * L, L / 2
    h, hb, w = D / 2, BOW / 2, B / 2
    out = []
    for s in (1, -1):
        # bottom (s = -1) and deck (s = 1): chords across, root at the transom
        out.append(b9.plate((-w, y0, s * h), (w, y0, s * h), (-w, yk, s * h), (w, yk, s * h), (0, y0 + 1, 0)))
        out.append(b9.plate((-w, yk, s * h), (w, yk, s * h), (-w, y1, s * hb), (w, y1, s * hb), (0, yk, 0)))
        # sides: chords up and down
        out.append(b9.plate((s * w, y0, -h), (s * w, y0, h), (s * w, yk, -h), (s * w, yk, h), (0, y0 + 1, 0)))
        out.append(b9.plate((s * w, yk, -h), (s * w, yk, h), (s * w, y1, -hb), (s * w, y1, hb), (0, yk, 0)))
    # transom and bow: chords up and down, span across
    out.append(b9.plate((-w, y0, -h), (-w, y0, h), (w, y0, -h), (w, y0, h), (0, 0, 0)))
    out.append(b9.plate((-w, y1, -hb), (-w, y1, hb), (w, y1, -hb), (w, y1, hb), (0, 0, 0)))
    return [p for p in out if p]


def main():
    src = open(os.path.join(here, "Planing Hull.craft"), encoding="utf-8-sig").read().replace("\r\n", "\n")
    m = re.search(r"(?ms)^PART\n\{\n.*?^\}\n", src)
    head = src[:m.start()]
    head = head.replace("ship = Planing Hull", "ship = Planing Hull PW")
    head = re.sub(r"(?m)^description = .*$",
                  "description = Planing Hull (one Spine: 2 m beam, 8 m long, 2 m deep, raked to 0.5 m at the bow) built "
                  "of ten flat B9 hull plates round a small Spine tank (its fuel the hull's mass)", head)
    root = m.group(0)
    root_id = 4294200000
    root = re.sub(r"(?m)^\tpart = .*$", "\tpart = proceduralTankLiquid_%d" % root_id, root)
    root = re.sub(r"(?m)^(\t\tlength = )8$", r"\g<1>%g" % BALLAST_LENGTH, root)
    root = re.sub(r"(?ms)(\t\tSPINE_NODE\n\t\t\{\n.*?\t\t\}\n)+",
                  "".join("\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %g\n\t\t\tsizeH = %g\n\t\t\tsizeV = %g\n\t\t\tsection = Rectangle\n\t\t}\n"
                          % (p, BALLAST, BALLAST) for p in (0, 1)), root, count=1)
    assert "length = %g" % BALLAST_LENGTH in root
    pls = plates()
    blocks, kids = b9.b9_parts(here, pls, "proceduralTankLiquid_%d" % root_id, (0.0, 0.0, 0.0), root_id + 1,
                               FRAME_POS, FRAME_ROT, MASS_MULTIPLIER)
    root = root.replace("\tmodSize = 0,0,0\n", "\tmodSize = 0,0,0\n" + "".join("\tlink = %s\n" % k for k in kids), 1)
    open(os.path.join(here, "Planing Hull PW.craft"), "w", encoding="utf-8", newline="\n").write(head + root + "".join(blocks))
    area = sum(p["length"] * (p["root"] + p["tip"]) / 2 for p in pls)
    print("wrote Planing Hull PW.craft: %d plates, %.2f m2" % (len(pls), area))


if __name__ == "__main__":
    main()
