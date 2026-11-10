"""Hull lines to a CustomProceduralParts Spine, and the hand statics of the same loft.

A hull is a list of stations (name, x along the hull from its bow, keel depth below the base line, half-section) in
any one unit; a half-section runs from the keel (0, keel) out and up to the base line, as (half-breadth, depth below
the base line) points. The Spine runs along the base line, bow at its top end (position 1); each station is a custom
section embedded in the craft, in the Spine's unit box (x = half-breadth / the widest, y = +1 at the base line, -1 at
the keel).

Scales: sx along the hull, sy across, sz in depth (m per unit), so a model's lines can be stretched to another size.
"""
import math

from scipy.optimize import brentq


def unit_section(keel, half):
    """The full section in the Spine's unit box, counter-clockwise from the keel, and its widest half-breadth."""
    wide = max(p[0] for p in half)
    right = [(p[0] / wide, 1 - 2 * p[1] / keel) for p in half]
    left = [(-x, y) for x, y in reversed(right[1:])]
    return right + left, wide


def spine_module(stations, length_units, sx, sy, sz, prefix, points=96, part_scale=1.0):
    """The ProceduralShapeSpine MODULE text. part_scale: the Spine's h/vScale, the node sizes written 1/part_scale
    larger (the Spine holds a node's own size to at least 0.1 m)."""
    sections, nodes = [], []
    for i, (name, x, keel, half) in enumerate(stations):
        pts, wide = unit_section(keel, half)
        sec = "%s_%02d" % (prefix, i)
        sections.append("\t\tCROSS_SECTION\n\t\t{\n\t\t\tname = %s\n" % sec +
                        "".join("\t\t\tv = %.5f,%.5f\n" % p for p in pts) + "\t\t}\n")
        nodes.append("\t\tSPINE_NODE\n\t\t{\n\t\t\tposition = %.6f\n\t\t\tsizeH = %.5f\n\t\t\tsizeV = %.5f\n"
                     "\t\t\toffsetV = %.5f\n\t\t\tsection = %s\n\t\t}\n"
                     % (1 - x / length_units, 2 * wide * sy / part_scale, keel * sz / part_scale,
                        -keel / 2 * sz / part_scale, sec))
    nodes.reverse()   # stern (position 0) first
    return ("\tMODULE\n\t{\n\t\tname = ProceduralShapeSpine\n\t\tisEnabled = True\n\t\tlength = %.5f\n\t\thScale = %g\n"
            "\t\tvScale = %g\n\t\tsectionPoints = %d\n\t\tstagingEnabled = True\n"
            % (length_units * sx, part_scale, part_scale, points) +
            "".join(sections) + "".join(nodes) +
            "\t\tEVENTS\n\t\t{\n\t\t}\n\t\tACTIONS\n\t\t{\n\t\t}\n\t\tUPGRADESAPPLIED\n\t\t{\n\t\t}\n\t}\n")


class Statics:
    """The loft as the Spine builds it (unit outlines paired along their length and lerped with the sizes), sliced
    every dx units; volume and floating attitude. Lengths in the stations' unit times the scales (m)."""

    NPT = 81

    def __init__(self, stations, sx, sy, sz, dx=0.1):
        self.sx, self.sy, self.sz = sx, sy, sz
        self.st = [(x, keel, *self._resample(half, keel)) for _, x, keel, half in stations]
        self.slices = []
        for i in range(len(stations) - 1):
            x0, x1 = stations[i][1], stations[i + 1][1]
            m = max(1, int(math.ceil((x1 - x0) / dx)))
            for k in range(m):
                x = x0 + (x1 - x0) * (k + 0.5) / m
                self.slices.append((x, (x1 - x0) / m, self.section(x)))

    def _resample(self, half, keel):
        wide = max(p[0] for p in half)
        u = [(p[0] / wide, 1 - 2 * p[1] / keel) for p in half]
        seg = [math.dist(u[i], u[i + 1]) for i in range(len(u) - 1)]
        total = sum(seg)
        out, i, acc = [], 0, 0.0
        for k in range(self.NPT):
            s = total * k / (self.NPT - 1)
            while i < len(seg) - 1 and acc + seg[i] < s:
                acc += seg[i]
                i += 1
            t = 0 if seg[i] == 0 else min(1, (s - acc) / seg[i])
            out.append((u[i][0] + t * (u[i + 1][0] - u[i][0]), u[i][1] + t * (u[i + 1][1] - u[i][1])))
        return out, wide

    def section(self, x):
        """Half-section at x as (half-breadth, depth below the base line) in the stations' unit."""
        for i in range(len(self.st) - 1):
            a, b = self.st[i], self.st[i + 1]
            if a[0] <= x <= b[0]:
                t = (x - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0
                keel = a[1] + t * (b[1] - a[1])
                wide = a[3] + t * (b[3] - a[3])
                pts = [(pa[0] + t * (pb[0] - pa[0]), pa[1] + t * (pb[1] - pa[1])) for pa, pb in zip(a[2], b[2])]
                return [(p[0] * wide, keel / 2 * (1 - p[1])) for p in pts]
        raise ValueError(x)

    @staticmethod
    def _area_below(half, zw):
        poly = half + [(0.0, 0.0)]
        clipped = []
        for i in range(len(poly)):
            p, q = poly[i], poly[(i + 1) % len(poly)]
            pin, qin = p[1] >= zw, q[1] >= zw
            if pin:
                clipped.append(p)
            if pin != qin:
                t = (zw - p[1]) / (q[1] - p[1])
                clipped.append((p[0] + t * (q[0] - p[0]), zw))
        a = 0.0
        for i in range(len(clipped)):
            p, q = clipped[i], clipped[(i + 1) % len(clipped)]
            a += p[0] * q[1] - q[0] * p[1]
        return abs(a)            # both halves: 2 * |shoelace| / 2

    def volume(self):
        """m3."""
        return sum(self._area_below(h, 0.0) * dx for _, dx, h in self.slices) * self.sx * self.sy * self.sz

    def displaced(self, zw_ref, x_ref, tau_deg):
        """Volume (m3) and its centroid x (units) under water whose surface is zw_ref (units of depth) below the base
        line at x_ref, trim tau bow up (in the scaled hull: the water rises toward the stern by tan(tau) per metre)."""
        t = math.tan(math.radians(tau_deg))
        vol = mom = 0.0
        for x, dx, half in self.slices:
            zw = zw_ref + (x_ref - x) * self.sx * t / self.sz
            a = self._area_below(half, zw) * dx
            vol += a
            mom += a * x
        return vol * self.sx * self.sy * self.sz, (mom / vol if vol else 0)

    def float_at(self, volume_m3, x_cg, x_ref):
        """Trim (deg) and water depth below the base line at x_ref (units) at which it displaces volume_m3 with its
        centre of buoyancy at x_cg (units from the bow)."""
        def f(tau):
            zw = brentq(lambda z: self.displaced(z, x_ref, tau)[0] - volume_m3, -50, 50)
            return self.displaced(zw, x_ref, tau)[1] - x_cg, zw
        tau = brentq(lambda t: f(t)[0], -15, 15)
        return tau, f(tau)[1]
