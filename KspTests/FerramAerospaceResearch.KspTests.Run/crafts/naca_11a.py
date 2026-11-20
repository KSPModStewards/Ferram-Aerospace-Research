"""Makes NACA 11-A.craft: the flying-boat hull of NACA TN 470 (Parkinson 1933, "A Complete Tank Test of a Model of a
Flying-Boat Hull - N.A.C.A. Model No. 11-A") as one CustomProceduralParts Spine, from the offsets of its Table I,
scaled to a 2 m beam (the model's is 17 in).

Each station is a custom section embedded in the craft: the keel, the bottom through the buttocks and waterlines the
table gives (forward of station 6; straight from there aft, as the table says), the main chine, the cove and upper
chine aft of the step, then the side straight up to the base line (the hull's top here: the table gives no topsides
forward of station 12). The step is two stations 0.1 in apart, the sternpost likewise. The spine runs along the base
line, bow at its top end.

    python naca_11a.py           writes NACA 11-A.craft beside this script (2 m beam)
    python naca_11a.py shell     writes NACA 11-A Shell.craft: the 2 m hull with its deck caved in, a SHELL_WALL wall (each
                                 section but the bow's, the stern's and the sternpost's the wall alone, solid 0.1 in from them): an open hull, one closed mesh
    python naca_11a.py model     writes NACA 11-A Model.craft: the tank model's own size (17 in beam), to tow it at the
                                 tank's Reynolds number
    import naca_11a              STATIONS (x from F.P. in; keel depth below the base line; half-section points as
                                 (half-breadth, depth below the base line), keel first, base line last), SCALE
"""
import math
import os
import re
import sys

SHELL_WALL = 0.02       # m, the shell's wall (python naca_11a.py shell)
SHELL_DENSITY = 2.15    # t/m3, FARTestBallast's dryDensity (mods/FARTestFloatplane/ProceduralBallast.cfg)
BEAM_M = 2.0
MODEL_BEAM_IN = 17.0
SCALE = BEAM_M / MODEL_BEAM_IN * 1.0      # m per model inch (the 2 m hull)
LENGTH_IN = 96.0

# (name, x from F.P. in, keel depth in, [(half-breadth, depth)] from the keel out to the main chine and above)
_RAW = [
    ("FP", 0.0, 4.00, [(0.25, 4.00)]),
    ("1/2", 2.40, 10.43, [(0.23, 9.5), (0.68, 8.0), (1.34, 6.5), (1.5, 6.25), (2.25, 5.27)]),
    ("1", 4.80, 11.80, [(0.38, 11.0), (1.14, 9.5), (1.5, 8.89), (2.09, 8.0), (3.0, 6.97), (3.55, 6.5), (3.81, 6.33)]),
    ("1 1/2", 7.20, 12.44, [(1.04, 11.0), (1.5, 10.38), (2.24, 9.5), (3.0, 8.68), (3.80, 8.0), (4.5, 7.51), (5.03, 7.19)]),
    ("2", 9.60, 12.83, [(0.31, 12.5), (1.5, 11.30), (1.79, 11.0), (3.0, 9.88), (3.49, 9.5), (4.5, 8.77), (5.84, 8.0),
                        (6.00, 7.93)]),
    ("3", 14.40, 13.29, [(1.14, 12.5), (1.5, 12.25), (3.0, 11.27), (3.41, 11.0), (4.5, 10.37), (6.0, 9.56), (6.12, 9.5),
                         (7.25, 9.00)]),
    ("4", 19.20, 13.48, [(1.5, 12.70), (1.88, 12.5), (3.0, 11.92), (4.5, 11.19), (4.89, 11.0), (6.0, 10.49), (7.94, 9.63)]),
    ("5", 24.00, 13.58, [(1.5, 12.92), (2.42, 12.5), (3.0, 12.25), (4.5, 11.62), (5.92, 11.0), (6.0, 10.97), (7.5, 10.33),
                         (8.28, 9.99)]),
    ("6", 28.80, 13.66, [(8.43, 10.17)]),
    ("7", 33.60, 13.75, [(8.49, 10.24)]),
    ("8", 38.40, 13.83, [(8.50, 10.32)]),
    ("9", 43.20, 13.92, [(8.50, 10.40)]),
    ("10 fwd", 48.00, 14.00, [(8.50, 10.48)]),
    ("10 aft", 48.10, 13.44, [(8.50, 9.92)]),
    ("11", 52.80, 12.97, [(8.50, 9.45)]),
    # main chine, cove (straight up from it), upper chine
    ("12", 57.60, 12.51, [(8.10, 9.16), (8.10, 8.23), (8.40, 8.10)]),
    ("13", 62.40, 12.04, [(6.97, 9.16), (6.97, 7.57), (8.11, 7.09)]),
    ("14", 67.20, 11.58, [(5.07, 9.48), (5.07, 7.21), (7.58, 6.17)]),
    ("15", 72.00, 11.11, [(2.59, 10.04), (2.59, 7.11), (6.77, 5.38)]),
    # the upper chine at the sternpost is between stations 15 and 16
    ("sternpost", 76.00, 10.74, [(0.20, 10.66), (0.20, 7.16), (5.94, 4.77)]),
    ("sternpost aft", 76.10, 7.24, [(5.94, 4.77)]),
    ("16", 76.80, 7.04, [(5.78, 4.65)]),
    ("17", 81.60, 5.91, [(4.61, 4.00)]),
    ("18", 86.40, 4.77, [(3.31, 3.40)]),
    ("19", 91.20, 3.64, [(1.90, 2.85)]),
    ("20", 96.00, 2.50, [(0.40, 2.33)]),
]

# half-sections: keel, the points, then the side straight up to the base line
STATIONS = [(name, x, keel, [(0.0, keel)] + pts + [(pts[-1][0], 0.0)]) for name, x, keel, pts in _RAW]


def unit_section(keel, half):
    """The full section in the Spine's unit box, counter-clockwise from the keel: x = half-breadth / the widest,
    y = +1 at the base line, -1 at the keel."""
    wide = max(p[0] for p in half)
    right = [(p[0] / wide, 1 - 2 * p[1] / keel) for p in half]
    left = [(-x, y) for x, y in reversed(right[1:])]
    return right + left, wide


def shell_section(keel, half, t):
    """The section as a hull with its deck caved in (an open boat's): the outer line from the deck's edge on one side
    round the keel to the other's, then back along the inner wall, the outer line moved in by t (model in; mitred at the
    corners), so the section is the wall alone. In the Spine's unit box as unit_section's, counter-clockwise."""
    # (x the half-breadth, y up from the base line)
    outer = [(-p[0], -p[1]) for p in reversed(half)] + [(p[0], -p[1]) for p in half[1:]]
    dedup = [outer[0]]
    for p in outer[1:]:
        if abs(p[0] - dedup[-1][0]) + abs(p[1] - dedup[-1][1]) > 1e-6:
            dedup.append(p)
    outer = dedup

    def normal(a, b):
        # the inside of the path (the hollow): the direction a to b turned 90 deg counter-clockwise
        dx, dy = b[0] - a[0], b[1] - a[1]
        n = math.hypot(dx, dy)
        return (-dy / n, dx / n)

    segs = [normal(a, b) for a, b in zip(outer, outer[1:])]
    inner = []
    for i, p in enumerate(outer):
        if i == 0:
            n = segs[0]
            q = (p[0] + t * n[0], p[1] + t * n[1])
        elif i == len(outer) - 1:
            n = segs[-1]
            q = (p[0] + t * n[0], p[1] + t * n[1])
        else:
            n1, n2 = segs[i - 1], segs[i]
            k = t / max(0.2, 1 + n1[0] * n2[0] + n1[1] * n2[1])
            q = (p[0] + k * (n1[0] + n2[0]), p[1] + k * (n1[1] + n2[1]))
        inner.append(q)
    inner[0] = (inner[0][0], 0.0)
    inner[-1] = (inner[-1][0], 0.0)
    poly = outer + list(reversed(inner))
    area = 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, poly[1:] + poly[:1]))
    if area < 0:
        poly.reverse()
    wide = max(p[0] for p in half)
    return [(x / wide, 1 + 2 * y / keel) for x, y in poly], wide, abs(area)


def main():
    args = sys.argv[1:]
    model = "model" in args
    shell = "shell" in args
    scale = 0.0254 if model else SCALE
    ship = ("NACA 11-A Model" if model else "NACA 11-A") + (" Shell" if shell else "")
    size = "the tank model's own size, 17 in beam" if model else "scaled to a 2 m beam"
    if shell:
        size += (", its deck caved in: a %.0f mm wall (each section but the bow's and the stern's the wall alone), an "
                 "open hull" % (SHELL_WALL * 1000))
    here = os.path.dirname(os.path.abspath(__file__))
    src = open(os.path.join(here, "Planing Box.craft"), encoding="utf-8-sig").read()
    src = src.replace("ship = Planing Box", "ship = " + ship)
    src = src.replace(
        "description = A flat-bottomed box hull for towing on the water: one ProceduralParts square prism, 2 m beam, 8 m long",
        "description = The NACA Model 11-A flying-boat hull (TN 470) from its Table I offsets, " + size + ": one "
        "ProceduralParts Spine (CustomProceduralParts), its stations embedded as custom sections")
    src = src.replace("shapeName = Polygon", "shapeName = Spine")
    length = LENGTH_IN * scale
    # the Spine holds a node's own size to at least 0.1 m: the model's nodes are written 10x and the part scaled 0.1
    part_scale = 0.1 if model else 1.0
    node = scale / part_scale
    sections, nodes = [], []
    wall = []    # (x in, the section's wall area in2) for the shell's volume
    # (the sternpost's two stations solid too: its main bottom a post 0.2 in wide, too narrow for a wall each side, a
    # bulkhead there)
    stations = [(name, x, keel, half, shell and 0 < i < len(STATIONS) - 1 and not name.startswith("sternpost"))
                for i, (name, x, keel, half) in enumerate(STATIONS)]
    if shell:
        # where a wall station meets a solid one, the wall's own section solid 0.1 in toward it (as the step's two
        # stations): the Spine matches two sections' points by arc length, and a wall's outline (out and back) lofted
        # into a solid one crosses its own walls all the way between them, a hole through the hull
        ends = []
        for (n0, x0, k0, h0, s0), (n1, x1, k1, h1, s1) in zip(stations, stations[1:]):
            if s0 and not s1:
                ends.append((n0 + " bulkhead", x0 + 0.1, k0, h0, False))
            elif s1 and not s0:
                ends.append((n1 + " bulkhead", x1 - 0.1, k1, h1, False))
        stations = sorted(stations + ends, key=lambda st: st[1])
    for i, (name, x, keel, half, walled) in enumerate(stations):
        if walled:
            pts, wide, a = shell_section(keel, half, SHELL_WALL / scale)
            wall.append((x, a))
        else:
            pts, wide = unit_section(keel, half)
            if shell:
                full = [(p[0], -p[1]) for p in half] + [(-p[0], -p[1]) for p in reversed(half[1:])]
                wall.append((x, abs(0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(full, full[1:] + full[:1])))))
        sec = "NACA11A_%02d" % i
        sections.append("\t\tCROSS_SECTION\n\t\t{\n\t\t\tname = %s\n" % sec +
                        "".join("\t\t\tv = %.5f,%.5f\n" % p for p in pts) + "\t\t}\n")
        nodes.append("\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %.6f\n\t\t\tsizeH = %.5f\n\t\t\tsizeV = %.5f\n"
                     "\t\t\toffsetV = %.5f\n\t\t\tsection = %s\n\t\t}\n"
                     % (1 - x / LENGTH_IN, 2 * wide * node, keel * node, -keel / 2 * node, sec))
    nodes.reverse()   # stern (position 0) first
    spine = ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = %.5f\n\t\thScale = %g\n"
             "\t\tvScale = %g\n\t\tsectionPoints = 96\n\t\tstagingEnabled = True\n" % (length, part_scale, part_scale) +
             "".join(sections) + "".join(nodes) +
             "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")
    start = src.index("\tMODULE\n\t{\n\t\tname = ProceduralShapePolygon")
    end = src.index("\tMODULE\n\t{\n\t\tname = TankContentSwitcher")
    src = src[:start] + spine + src[end:]
    if shell:
        # the wall weighs nothing as a fuel tank (31 kg): the test mods' FARTestBallast tank type
        # (mods/FARTestFloatplane/ProceduralBallast.cfg: SHELL_DENSITY t/m3), the hull ~3.2 t (its ends solid)
        src, n = re.subn(r"(?m)^(\t\ttankType = ).*$", r"\g<1>FARTestBallast", src)
        assert n == 1, "tankType"
        src = re.sub(r"(?ms)^\tRESOURCE\n\t\{\n.*?^\t\}\n", "", src)
        vol = sum((b[0] - a[0]) * (a[1] + b[1]) / 2 for a, b in zip(wall, wall[1:])) * scale ** 3
        print("the shell's wall: %.3f m3, %.2f t at %g t/m3" % (vol, vol * SHELL_DENSITY, SHELL_DENSITY))
    open(os.path.join(here, ship + ".craft"), "w", encoding="utf-8", newline="\n").write(src)
    print("wrote %s.craft: %.3f m long, %d stations" % (ship, length, len(STATIONS)))


if __name__ == "__main__":
    main()
