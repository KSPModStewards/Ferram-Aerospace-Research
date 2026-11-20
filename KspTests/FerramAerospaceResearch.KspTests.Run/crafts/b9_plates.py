"""B9 procedural wing panels as flat hull plates, for crafts built in a Spine's frame (planing_hull_pw.py).

A plate is a TypeA panel with zero-width edges: a trapezoid slab THICKNESS thick, its root and tip chords parallel.
plate() fits one to a quad (root chord r0-r1, tip chord t0-t1) with its outer face on the quad and the slab moved in
toward a point inside the hull. B9's part frame is not its mesh's: the part's x is the mesh's -x (the tip at part +x),
its y the mesh's -z (the chord's leading edge forward), its z the mesh's -y (fitted to a dump of plates in KSP).
b9_parts() writes the PART blocks, surface-attached as a tree grown from the root (attach_tree: every joint short,
between neighbours; one hub holding many plates metres off flung them apart at launch), each a FAR hull plate
(FARWingAerodynamicModel.hullPlate).
"""
import math
import os
import re

THICKNESS = 0.02    # m
B9 = "B9.Aero.Wing.Procedural.TypeA"


# --- vectors and quaternions (x, y, z, w) ---
def add(a, b): return tuple(x + y for x, y in zip(a, b))
def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def mul(a, k): return tuple(x * k for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return mul(a, 1 / norm(a))


def qmul(p, q):
    px, py, pz, pw = p
    qx, qy, qz, qw = q
    return (pw * qx + px * qw + py * qz - pz * qy, pw * qy - px * qz + py * qw + pz * qx,
            pw * qz + px * qy - py * qx + pz * qw, pw * qw - px * qx - py * qy - pz * qz)


def qrot(q, v):
    x, y, z, w = qmul(qmul(q, (v[0], v[1], v[2], 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return (x, y, z)


def qfrom(ex, ey, ez):
    """The rotation taking the local axes onto ex, ey, ez (columns of a proper rotation matrix)."""
    m00, m01, m02 = ex[0], ey[0], ez[0]
    m10, m11, m12 = ex[1], ey[1], ez[1]
    m20, m21, m22 = ex[2], ey[2], ez[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, s / 4)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1 + m00 - m11 - m22) * 2
        return (s / 4, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, s / 4, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1 + m22 - m00 - m11) * 2
    return ((m02 + m20) / s, (m12 + m21) / s, s / 4, (m10 - m01) / s)


def plate(r0, r1, t0, t1, inside):
    """A plate from its root chord r0-r1 and tip chord t0-t1 (the Spine's frame), the slab moved toward inside by half
    its thickness. None when both chords are ~0. (Chords not quite parallel take their mean direction.)"""
    lr, lt = norm(sub(r1, r0)), norm(sub(t1, t0))
    if max(lr, lt) < 1e-3:
        return None
    if lt > lr:     # (B9's root is at least 1 cm: the wider chord the root)
        r0, r1, t0, t1, lr, lt = t0, t1, r0, r1, lt, lr
    dr = unit(sub(r1, r0))
    dt = unit(sub(t1, t0)) if lt > 1e-6 else dr
    if dot(dr, dt) < 0:
        dt = mul(dt, -1)
        t0, t1 = t1, t0
    u = unit(add(dr, dt))
    rc, tc = mul(add(r0, r1), 0.5), mul(add(t0, t1), 0.5)
    d = sub(tc, rc)
    off = dot(d, u)
    span = sub(d, mul(u, off))
    length = norm(span)
    if length < 1e-4:
        return None
    ex = mul(span, -1 / length)        # mesh: the root at x = 0, the tip at x = -length
    ez = u
    ey = cross(ez, ex)
    n = ey if dot(sub(inside, rc), ey) > 0 else mul(ey, -1)
    origin = add(rc, mul(n, THICKNESS / 2))
    return dict(origin=origin, axes=(ex, ey, ez), length=length, root=max(lr, 0.01), tip=lt, offsetTip=off)


def attach_tree(pls, root_pos):
    """Each plate's parent (-1: the root): grown from the root, the nearest plate to the tree joined to its nearest
    member each time."""
    pts = [pl["origin"] for pl in pls]
    n = len(pts)
    parent = [None] * n
    best = [(norm(sub(p, root_pos)), -1) for p in pts]
    for _ in range(n):
        i = min((k for k in range(n) if parent[k] is None), key=lambda k: best[k][0])
        parent[i] = best[i][1]
        for k in range(n):
            if parent[k] is None:
                d = norm(sub(pts[k], pts[i]))
                if d < best[k][0]:
                    best[k] = (d, i)
    return parent


def template(crafts_dir):
    """A B9 TypeA PART block (Aeris 4A PW's), its links, symmetry and attachment dropped."""
    src = open(os.path.join(crafts_dir, "Aeris 4A PW.craft"), encoding="utf-8-sig").read()
    m = re.search(r"(?ms)^PART\r?\n\{\r?\n\tpart = B9\.Aero\.Wing\.Procedural\.TypeA_4294809824\r?\n.*?^\}\r?\n", src)
    return re.sub(r"(?m)^\t(link|sym|srfN|attN) = .*\n", "", m.group(0).replace("\r\n", "\n"))


def b9_parts(crafts_dir, pls, root_name, root_pos, first_id, frame_pos, frame_rot, mass_multiplier=1.0):
    """The plates' PART blocks (in the frame at frame_pos / frame_rot in the craft) and the root's children's names."""
    tmpl = template(crafts_dir)
    names = ["%s_%d" % (B9, first_id + i) for i in range(len(pls))]
    parent = attach_tree(pls, root_pos)
    children = {}
    for i, p in enumerate(parent):
        children.setdefault(p, []).append(names[i])
    blocks = []
    for i, pl in enumerate(pls):
        ex, ey, ez = pl["axes"]
        q = qmul(frame_rot, qfrom(mul(ex, -1), mul(ez, -1), mul(ey, -1)))
        pos = add(frame_pos, qrot(frame_rot, pl["origin"]))
        b = tmpl.replace("TypeA_4294809824", "TypeA_%d" % (first_id + i))
        for k, val in (("pos", "%.6f,%.6f,%.6f" % pos), ("attPos", "0,0,0"), ("attPos0", "%.6f,%.6f,%.6f" % pos),
                       ("rot", "%.9g,%.9g,%.9g,%.9g" % q), ("attRot", "0,0,0,1"), ("attRot0", "%.9g,%.9g,%.9g,%.9g" % q)):
            b, n = re.subn(r"(?m)^\t%s = .*$" % k, "\t%s = %s" % (k, val), b)
            assert n == 1, k
        b = b.replace("\tsepI = 1\n", "\tsepI = 0\n")
        flag = "\t\tname = FARWingAerodynamicModel\n\t\tisEnabled = True\n"
        assert b.count(flag) == 1
        b = b.replace(flag, flag + "\t\thullPlate = True\n")
        b, n = re.subn(r"(?m)^(\t\tmassMultiplier = ).*$", lambda mm: mm.group(1) + "%g" % mass_multiplier, b)
        assert n == 1, "massMultiplier"
        up = root_name if parent[i] < 0 else names[parent[i]]
        links = "".join("\tlink = %s\n" % c for c in children.get(i, []))
        b = b.replace("\tmodSize = 0,0,0\n", "\tmodSize = 0,0,0\n%s\tsrfN = srfAttach,%s\n" % (links, up), 1)
        values = {"sharedBaseLength": pl["length"], "sharedBaseWidthRoot": pl["root"], "sharedBaseWidthTip": pl["tip"],
                  "sharedBaseOffsetRoot": 0, "sharedBaseOffsetTip": pl["offsetTip"],
                  "sharedBaseThicknessRoot": THICKNESS, "sharedBaseThicknessTip": THICKNESS}
        for k, v in values.items():
            b, n = re.subn(r"(?m)^(\t\t%s = ).*$" % k, lambda mm: mm.group(1) + "%.6g" % v, b)
            assert n == 1, k
        blocks.append(b)
    return blocks, children.get(-1, [])
