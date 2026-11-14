"""A one-part hull split into several parts along its length, the same hull: the pieces of a ProceduralParts Spine
(cut at spine positions, a node added at a cut between two nodes of the same section) or Polygon prism (cut anywhere),
each its own part in the hull's place, each surface-attached to the one ahead of it, rigid and autostrutted to the root.
FAR's water forces should treat the split hull as the one-part hull (KspTests: the *Split tow tests against the
one-part ones).

    python split_hull.py SRC.craft OUT.craft SHIP CUTS     CUTS: spine positions (0 stern, 1 bow), comma separated
    import split_hull                                      split_spine_module / split_polygon_module, the pieces'
                                                           module texts and their middles' offsets along the hull
"""
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "planecraft"))
import craftio as cio  # noqa: E402


def _nodes(module):
    out = []
    for m in re.finditer(r"\t\tSPINE_NODE\n\t\t\{\n(.*?)\t\t\}\n", module, re.S):
        d = dict(re.findall(r"(?m)^\t\t\t(\w+) = (.*)$", m.group(1)))
        out.append(d)
    return out


def _node_text(d):
    return "\t\tSPINE_NODE\n\t\t{\n" + "".join("\t\t\t%s = %s\n" % kv for kv in d.items()) + "\t\t}\n"


def _at(nodes, p):
    """the node at spine position p: an existing one, or one between two of the same section, its sizes lerped"""
    for d in nodes:
        if abs(float(d["position"]) - p) < 1e-6:
            return dict(d)
    for a, b in zip(nodes, nodes[1:]):
        pa, pb = float(a["position"]), float(b["position"])
        if pa < p < pb:
            if a.get("section") != b.get("section"):
                raise ValueError("cut at %g between nodes of different sections (%s, %s): cut at a node" %
                                 (p, a.get("section"), b.get("section")))
            t = (p - pa) / (pb - pa)
            d = dict(a)
            for k in ("sizeH", "sizeV", "offsetV", "offsetH"):
                if k in a or k in b:
                    d[k] = "%.6g" % (float(a.get(k, 0)) + t * (float(b.get(k, 0)) - float(a.get(k, 0))))
            d["position"] = "%.6g" % p
            return d
    raise ValueError("cut %g outside the spine" % p)


def split_spine_module(module, cuts):
    """[(module text, offset of the piece's middle from the whole's along the spine toward the bow, m)], stern first"""
    nodes = sorted(_nodes(module), key=lambda d: float(d["position"]))
    length = float(re.search(r"(?m)^\t\tlength = (.*)$", module).group(1))
    head = module[:module.index("\t\tSPINE_NODE\n")]
    tail = module[module.rindex("\t\t}\n", 0, module.index("\t\tEVENTS\n")) + 4:]
    bounds = [0.0] + sorted(cuts) + [1.0]
    pieces = []
    for pa, pb in zip(bounds, bounds[1:]):
        inner = [dict(d) for d in nodes if pa + 1e-6 < float(d["position"]) < pb - 1e-6]
        ends = [_at(nodes, pa)] + inner + [_at(nodes, pb)]
        for d in ends:
            d["position"] = "%.6f" % ((float(d["position"]) - pa) / (pb - pa))
        text = re.sub(r"(?m)^\t\tlength = .*$", "\t\tlength = %.6f" % (length * (pb - pa)), head, count=1)
        pieces.append((text + "".join(_node_text(d) for d in ends) + tail, ((pa + pb) / 2 - 0.5) * length))
    return pieces


def split_polygon_module(module, cuts):
    length = float(re.search(r"(?m)^\t\tlength = (.*)$", module).group(1))
    bounds = [0.0] + sorted(cuts) + [1.0]
    return [(re.sub(r"(?m)^\t\tlength = .*$", "\t\tlength = %.6f" % (length * (pb - pa)), module, count=1),
             ((pa + pb) / 2 - 0.5) * length) for pa, pb in zip(bounds, bounds[1:])]


def split_part(block, cuts, pid0, parent=None):
    """The part block as pieces: [(block, name)], stern first; the first piece takes the block's place (its
    attachment to parent, if any, kept), the others surface-attached to the piece ahead of them."""
    name = cio.part_id(block)
    if not re.search(r"(?m)^\tEVENTS$", block):
        # a part with no part-level nodes (an older craft): the links and attachment go before them, not in a module
        k = re.search(r"(?m)^\tMODULE$", block).start()
        block = block[:k] + "\tEVENTS\n\t{\n\t}\n\tACTIONS\n\t{\n\t}\n\tPARTDATA\n\t{\n\t}\n" + block[k:]
    kind ="ProceduralShapeSpine" if "\t\tname = ProceduralShapeSpine\n" in block else "ProceduralShapePolygon"
    i, j = cio.module_span(block, kind)
    pieces = (split_spine_module if kind == "ProceduralShapeSpine" else split_polygon_module)(block[i:j], cuts)
    pos = np.array([float(v) for v in re.search(r"(?m)^\tpos = (.*)$", block).group(1).split(",")])
    rot = np.array([float(v) for v in re.search(r"(?m)^\trot = (.*)$", block).group(1).split(",")])
    axis = cio.qrot(rot, np.array([0.0, 1.0, 0.0]))       # the spine's bow end is the part's +y
    out = []
    prev = None
    for n, (module, offset) in enumerate(pieces):
        b = block[:i] + module + block[j:]
        b = cio.drop_lines(b, "link")
        b = cio.setf_add(b, "autostrutMode", "Root")
        b = cio.setf_add(b, "rigidAttachment", "True")
        p = pos + axis * offset
        if n == 0:
            pname = name
            b = cio.setf(b, "pos", cio.fmt(p))
            if parent is None:
                b = cio.setf(b, "attPos0", cio.fmt(p))
            else:
                ppos, prot = parent
                b = cio.setf(b, "attPos0", cio.fmt(cio.qrot(cio.qinv(prot), p - np.array(ppos))))
        else:
            # stack-attached to the piece astern of it (its bottom node on that one's top), as a hull's sections are:
            # FAR takes a surface-attached part's axis from its attach direction, across the hull
            pname = re.sub(r"_\d+$", "_%d" % (pid0 + n), name)
            b = re.sub(r"(?m)^\tpart = \S+", "\tpart = %s" % pname, b, count=1)
            for key, val in (("persistentId", str(pid0 + n)), ("pos", cio.fmt(p)), ("attPos", "0,0,0"), ("attRot", "0,0,0,1"),
                             ("attPos0", cio.fmt(cio.qrot(cio.qinv(rot), p - prev[1]))), ("attRot0", "0,0,0,1"), ("attm", "0")):
                b = cio.setf_add(b, key, val)
            for key in ("link", "attN", "sym", "srfN"):
                b = cio.drop_lines(b, key)
            b = b.replace("\tEVENTS\n", "\tattN = bottom,%s\n\tEVENTS\n" % prev[0], 1)
        prev = (pname, p)
        out.append([b, pname])
    for k in range(len(out) - 1):
        out[k][0] = cio.add_links(out[k][0], [out[k + 1][1]])
        out[k][0] = out[k][0].replace("\tEVENTS\n", "\tattN = top,%s\n\tEVENTS\n" % out[k + 1][1], 1)
    return [tuple(x) for x in out]


def main():
    src, dst, ship, cuts = sys.argv[1], sys.argv[2], sys.argv[3], [float(c) for c in sys.argv[4].split(",")]
    text = open(src, encoding="utf-8-sig").read().replace("\r\n", "\n")
    head, parts = cio.split_parts(text)
    assert len(parts) == 1, "a one-part hull"
    head = re.sub(r"(?m)^ship = .*$", "ship = " + ship, head)
    head = re.sub(r"(?m)^(description = .*)$", lambda m: m.group(1) + "; split into %d parts at spine positions %s "
                  "(crafts/split_hull.py)" % (len(cuts) + 1, ", ".join("%g" % c for c in cuts)), head)
    pieces = split_part(parts[0], cuts, 4200000000)
    open(dst, "w", encoding="utf-8", newline="\n").write(head + "".join(b for b, _ in pieces))
    print("wrote %s: %d parts" % (dst, len(pieces)))


if __name__ == "__main__":
    main()
