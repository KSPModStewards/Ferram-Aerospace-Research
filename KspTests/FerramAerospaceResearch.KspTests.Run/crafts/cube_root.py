"""The stock Kerbal 1 on a cubic strut root, for FAR's tests of the voxel model's axes.

    python cube_root.py STOCK.craft y "Kerbal 1 Cube Root.craft"
    python cube_root.py STOCK.craft x "Kerbal 1 Cube Root X.craft"
    python cube_root.py STOCK.craft z "Kerbal 1 Cube Root Z.craft"
    python cube_root.py STOCK.craft y30 "Kerbal 1 Cube Root Y30.craft"
    python cube_root.py STOCK.craft x60 "Kerbal 1 Cube Root X60.craft"
    python cube_root.py STOCK.craft z60 "Kerbal 1 Cube Root Z60.craft"

STOCK is the game's Ships/VAB/Kerbal 1.craft. Its parachute, on the pod's top node, gives way to a Cubic Octagonal
Strut (strutCube) on that node, made the craft's root. FAR voxelizes an editor craft in its root part's frame, and the
rocket's axis is the axis of that frame the stack lies along: y with the strut as the game places it, x with the strut
turned 90 deg about the world z axis, z with it turned -90 deg about the world x axis (the root's +x, +z up). The strut
is turned about its own centre, and a cube turned by 90 deg is the same cube, so the three crafts have one shape and
differ only in the frame FAR builds the voxel model in.

y30, x60 and z60 turn the strut 30 deg and 60 deg about the world z axis and -60 deg about the world x axis: the
stack lies between two of the root's axes, nearer y, x and z, so FAR's axis is none of the root's and the voxel
model is cut at a slant to its grid (a cube turned by these angles is not the same cube: the strut is 0.26 m).
"""
import re, sys

stock_path, axis, out_path = sys.argv[1:4]
ROT = {"y": "0,0,0,1", "x": "0,0,0.707106769,0.707106769", "z": "-0.707106769,0,0,0.707106769",
       "y30": "0,0,0.258819045,0.965925826", "x60": "0,0,0.5,0.866025404", "z60": "-0.5,0,0,0.866025404"}[axis]
CUBE_ID = "strutCube_4294000001"
HALF = 0.1287344          # strutCube's node offsets: top and bottom at +-0.1287344 along its y

text = open(stock_path, encoding="utf-8", newline="").read()
nl = "\r\n" if "\r\n" in text else "\n"


def blocks(t):
    """(start, end) of each top-level PART { ... } block."""
    out, i = [], 0
    while True:
        m = re.search(r"(?m)^PART\r?\n\{", t[i:])
        if not m:
            return out
        s = i + m.start()
        depth, j = 0, s + m.end() - m.start() - 1
        while True:
            c = t[j]
            if c == "{":
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0:
                    out.append((s, j + 1))
                    break
            j += 1
        i = j + 1


parts = blocks(text)
pod = next(b for b in parts if re.search(r"(?m)^\tpart = mk1pod\.v2_", text[b[0]:b[1]]))
chute = next(b for b in parts if re.search(r"(?m)^\tpart = parachuteSingle_", text[b[0]:b[1]]))
pod_text = text[pod[0]:pod[1]]
pod_id = re.search(r"(?m)^\tpart = (\S+)", pod_text).group(1)
chute_id = re.search(r"(?m)^\tpart = (\S+)", text[chute[0]:chute[1]]).group(1)
assert text.count(chute_id) == 3, "the parachute is named only by its own block and the pod's link and node"
px, py, pz = (float(v) for v in re.search(r"(?m)^\tpos = (\S+)", pod_text).group(1).split(","))
top = float(re.search(r"(?m)^\tattN = top," + re.escape(chute_id) + r"_0\|([-\d.]+)\|0", pod_text).group(1))

# the pod: its parachute link gone, its top node on the strut
new_pod = re.sub(r"(?m)^\tlink = " + re.escape(chute_id) + r"\r?\n", "", pod_text)
new_pod = new_pod.replace("attN = top," + chute_id + "_", "attN = top," + CUBE_ID + "_")

cube = nl.join([
    "PART", "{",
    "\tpart = " + CUBE_ID,
    "\tpartName = Part",
    f"\tpos = {px:.7g},{py + top + HALF:.9g},{pz:.7g}",
    "\tattPos = 0,0,0",
    "\tattPos0 = 0,0,0",
    "\trot = " + ROT,
    "\tattRot = 0,0,0,1",
    "\tattRot0 = 0,0,0,1",
    "\tmir = 1,1,1",
    "\tsymMethod = Radial",
    "\tautostrutMode = Off",
    "\trigidAttachment = False",
    "\tistg = -1",
    "\tresPri = 0",
    "\tdstg = 0",
    "\tsidx = -1",
    "\tsqor = -1",
    "\tsepI = -1",
    "\tattm = 0",
    "\tsameVesselCollision = False",
    "\tmodCost = 0",
    "\tmodMass = 0",
    "\tmodSize = 0,0,0",
    "\tlink = " + pod_id,
    f"\tattN = bottom,{pod_id}_0|{-HALF:.7g}|0",
    "\tEVENTS", "\t{", "\t}",
    "\tACTIONS", "\t{", "\t}",
    "\tPARTDATA", "\t{", "\t}",
    "}",
])

# the strut first (a craft's first part is its root), the parachute's block gone, the pod rewritten
first = parts[0][0]
out = text[:first] + cube + nl
rest = text[first:]
off = -first
chunks, i = [], 0
for s, e in parts:
    s += off; e += off
    chunks.append(rest[i:s])
    if (s - off, e - off) == chute:
        # drop the block and the line break after it
        j = e
        while j < len(rest) and rest[j] in "\r\n":
            j += 1
        i = j
        continue
    chunks.append(new_pod if (s - off, e - off) == pod else rest[s:e])
    i = e
chunks.append(rest[i:])
out += "".join(chunks)
name = "Kerbal 1 Cube Root" + ("" if axis == "y" else " " + axis.upper())
out = re.sub(r"(?m)^ship = .*$", "ship = " + name, out, count=1)
assert chute_id not in out
open(out_path, "w", encoding="utf-8", newline="").write(out)
print(out_path, "root", CUBE_ID, "rot", ROT)
