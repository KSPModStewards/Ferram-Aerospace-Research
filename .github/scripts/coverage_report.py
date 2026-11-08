"""FAR's line coverage from AltCover/OpenCover coverage.xml files made by one instrumentation (the unit tests' and each
in-game KSP instance's): visit counts summed point by point, per folder, plus the largest methods never run.

    python coverage_report.py ROOT OUT.md REPORT.xml [REPORT.xml ...]

ROOT is the last path part before the repository's own folders ("Ferram-Aerospace-Research/" for a checkout of that
name): source paths are shown from after its last occurrence.
"""
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

root, out, reports = sys.argv[1], sys.argv[2], sys.argv[3:]
if not reports:
    sys.exit("no coverage reports")


def load(path):
    doc = ET.parse(path).getroot()
    files = {f.get("uid"): f.get("fullPath").replace("\\", "/") for f in doc.iter("File")}
    methods = []
    for m in doc.iter("Method"):
        pts = m.findall("./SequencePoints/SequencePoint")
        if pts:
            name = m.findtext("Name") or "?"
            methods.append((name, files.get(pts[0].get("fileid"), "?"), [int(p.get("vc", "0")) for p in pts]))
    return methods


base = load(reports[0])
counts = [list(v) for _, _, v in base]
for r in reports[1:]:
    other = load(r)
    if len(other) != len(base):
        sys.exit(f"{r}: {len(other)} methods, {len(base)} in {reports[0]}: not the same instrumentation")
    for i, (_, _, v) in enumerate(other):
        counts[i] = [a + b for a, b in zip(counts[i], v)]


def rel(p):
    i = p.rfind(root)
    return p[i + len(root):] if i >= 0 else p


tot, hit, mtot, mhit = defaultdict(int), defaultdict(int), defaultdict(int), defaultdict(int)
unrun = []
for (name, path, _), vc in zip(base, counts):
    folder = "/".join(rel(path).split("/")[:-1]) or "."
    tot[folder] += len(vc)
    hit[folder] += sum(1 for c in vc if c > 0)
    mtot[folder] += 1
    if any(vc):
        mhit[folder] += 1
    else:
        unrun.append((len(vc), rel(path), name))

T, H = sum(tot.values()), sum(hit.values())
lines = [f"# FAR coverage ({len(reports)} report(s))", "",
         f"**{H} / {T} lines ({100 * H / T:.1f}%)**, {sum(mhit.values())} / {sum(mtot.values())} methods", "",
         "| folder | lines | % | methods |", "|---|---|---|---|"]
for k in sorted(tot, key=lambda k: -tot[k]):
    lines.append(f"| {k} | {hit[k]} / {tot[k]} | {100 * hit[k] / tot[k]:.0f}% | {mhit[k]} / {mtot[k]} |")
lines += ["", "## Largest methods never run", "", "| lines | file | method |", "|---|---|---|"]
for n, f, name in sorted(unrun, reverse=True)[:40]:
    short = name.split("::")[-1] if "::" in name else name
    lines.append(f"| {n} | {f} | `{short[:110]}` |")
open(out, "w", encoding="utf-8").write("\n".join(lines) + "\n")
print("\n".join(lines[:3]))
