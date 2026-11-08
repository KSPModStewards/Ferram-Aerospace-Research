"""Twins of the stock Aeris 4A for FAR's tests: parts swapped for procedural ones of the same shape, in the same places,
rotations, symmetry and attachments.

    python aeris_twin.py wings STOCK.craft TEMPLATE.craft "Aeris 4A PW.craft"
    python aeris_twin.py tanks STOCK.craft TEMPLATE.craft "Aeris 4A PP.craft"

STOCK is the game's Ships/SPH/Aeris 4A.craft. TEMPLATE is any craft with the procedural part (its modules are the
template; the shape fields are set here).

wings: the main wing panels (structuralWing, deltaWing) as B9 procedural TypeA panels of the planform FAR gives the
stock parts (FerramAerospaceResearch.cfg: MAC 2.25, taper 0.091 for both; b_2 1.875, mid-chord sweep 44.99 deg for the
structural wing; b_2 3.75, 26.57 deg for the delta wing). With zero-width edges a B9 chord is its base width, its
mid-chord sweep atan(offsetTip / length), and its root mid-chord at the part origin, as FAR has the stock wings' (no
rootMidChordOffsetFromOrig). B9 builds its panel along the other side of the part's span axis from the stock model, so
each panel is turned 180 deg about its chord (local y): it reaches outboard, its leading edge still forward.

tanks: the fuel tanks (fuelTank.long, the FL-T800: 1.25 m x 3.75 m; fuelTank, the FL-T400: 1.25 m x 1.875 m) as
ProceduralParts cylinders of their size (ProceduralParts' axis is the part's y, as the stock tanks').
"""
import math, re, sys

mode, stock_path, template_path, out_path = sys.argv[1:5]
B9 = "B9.Aero.Wing.Procedural.TypeA"
PP = "proceduralTankLiquid"
PLANFORM = {"structuralWing": (1.875, 44.99), "deltaWing": (3.75, 26.57)}
MAC, TAPER = 2.25, 0.091
TANKS = {"fuelTank.long": (1.25, 3.75), "fuelTank": (1.25, 1.875)}


def blocks(text):
    """(start, end) of each top-level PART { ... } block."""
    out, i = [], 0
    while True:
        m = re.search(r"(?m)^PART\r?\n\{", text[i:])
        if not m:
            return out
        s = i + m.start()
        depth, j = 0, s + m.end() - m.start() - 1
        while True:
            c = text[j]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    out.append((s, j + 1))
                    break
            j += 1
        i = j + 1


def top_lines(block):
    """The block's own key = value lines (depth 1), in order."""
    lines, depth = [], 0
    for ln in block.splitlines():
        t = ln.strip()
        if t == "{":
            depth += 1
            continue
        if t == "}":
            depth -= 1
            continue
        if depth == 1 and "=" in t:
            lines.append(t)
    return lines


def sub_nodes(block, keep):
    """The block's MODULE nodes whose name is in keep, verbatim, in order."""
    out, lines, i, depth = [], block.splitlines(), 0, 0
    while i < len(lines):
        t = lines[i].strip()
        if t == "{":
            depth += 1
        elif t == "}":
            depth -= 1
        elif depth == 1 and t in ("MODULE", "RESOURCE") and lines[i + 1].strip() == "{":
            start, d, j = i, 0, i + 1
            while True:
                tj = lines[j].strip()
                d += tj == "{"
                d -= tj == "}"
                if d == 0:
                    break
                j += 1
            node = "\n".join(lines[start:j + 1])
            name = re.search(r"name = (\S+)", node).group(1)
            if t == "MODULE" and name in keep:
                out.append(node)
            i = j
        i += 1
    return out


def set_fields(node, values):
    for k, v in values.items():
        node, n = re.subn(r"(?m)^(\s*%s = ).*$" % k, lambda mm: mm.group(1) + (v if isinstance(v, str) else "%.6g" % v), node)
        assert n == 1, k
    return node


def flip_y(line):
    """rot = x,y,z,w turned 180 deg about the part's own y axis: q * j."""
    x, y, z, w = (float(v) for v in line.split("=", 1)[1].split(","))
    # Hamilton product (w, x, y, z) of q and j = (0, 0, 1, 0): (-y, -z, w, x)
    nw, nx, ny, nz = -y, -z, w, x
    return "rot = %.9g,%.9g,%.9g,%.9g" % (nx, ny, nz, nw)


def wing(kind, modules):
    semispan, sweep = PLANFORM[kind]
    root = 2 * MAC / (1 + TAPER)
    values = {"sharedBaseLength": semispan, "sharedBaseWidthRoot": root, "sharedBaseWidthTip": TAPER * root,
              "sharedBaseOffsetRoot": 0, "sharedBaseOffsetTip": semispan * math.tan(math.radians(sweep)),
              "sharedBaseThicknessRoot": 0.12, "sharedBaseThicknessTip": 0.04,
              "sharedEdgeWidthLeadingRoot": 0, "sharedEdgeWidthLeadingTip": 0,
              "sharedEdgeWidthTrailingRoot": 0, "sharedEdgeWidthTrailingTip": 0}
    return [set_fields(n, values) if "name = WingProcedural" in n else n for n in modules]


def tank(kind, modules):
    diameter, length = TANKS[kind]
    out = []
    for n in modules:
        if "name = ProceduralPart" in n:
            n = set_fields(n, {"shapeName": "Cylinder"})
        elif "name = ProceduralShapeCylinder" in n:
            n = set_fields(n, {"diameter": diameter, "length": length})
        out.append(n)
    return out


if mode == "wings":
    NEW, KINDS, KEEP, MAKE, TURN = B9, PLANFORM, {"WingProcedural", "FARWingAerodynamicModel", "GeometryPartModule", "FARAeroPartModule"}, wing, True
    SHIP, DESC = "Aeris 4A PW", "The Aeris 4A with its main wings as B9 procedural panels of the stock parts' planform"
elif mode == "tanks":
    NEW, KINDS, KEEP, MAKE, TURN = PP, TANKS, {"ProceduralPart", "ProceduralShapeCylinder", "TankContentSwitcher", "GeometryPartModule", "FARAeroPartModule"}, tank, False
    SHIP, DESC = "Aeris 4A PP", "The Aeris 4A with its fuel tanks as ProceduralParts cylinders of the stock tanks' size"
else:
    sys.exit("mode: wings or tanks")

stock = open(stock_path, encoding="utf-8-sig").read()
tmpl_text = open(template_path, encoding="utf-8-sig").read()
tmpl = next(tmpl_text[s:e] for s, e in blocks(tmpl_text) if re.search(r"part = " + re.escape(NEW) + "_", tmpl_text[s:e]))
modules = sub_nodes(tmpl, KEEP)
assert len(modules) == len(KEEP), [re.search(r"name = (\S+)", n).group(1) for n in modules]

renames, pieces, last = {}, [], 0
for s, e in blocks(stock):
    block = stock[s:e]
    m = re.search(r"part = (\S+)_(\d+)\s", block)
    kind, pid = m.group(1), m.group(2)
    if kind not in KINDS:
        continue
    keys = [ln for ln in top_lines(block) if not ln.startswith("part =")]
    if TURN:
        keys = [flip_y(ln) if ln.startswith("rot =") else ln for ln in keys]
    new = "PART\n{\n\tpart = %s_%s\n" % (NEW, pid) + "".join("\t%s\n" % ln for ln in keys) + "\n".join(MAKE(kind, modules)) + "\n}"
    renames["%s_%s" % (kind, pid)] = "%s_%s" % (NEW, pid)
    pieces.append((s, e, new))

out = ""
for s, e, new in pieces:
    out += stock[last:s] + new
    last = e
out += stock[last:]
for old, new in renames.items():
    out = re.sub(r"(?<![\w.])%s(?![\w.])" % re.escape(old), new, out)
out = re.sub(r"(?m)^ship = .*$", "ship = " + SHIP, out, count=1)
out = re.sub(r"(?m)^description = .*$", "description = " + DESC, out, count=1)
open(out_path, "w", encoding="utf-8", newline="\n").write(out)
print("replaced", ", ".join(sorted(renames)))
