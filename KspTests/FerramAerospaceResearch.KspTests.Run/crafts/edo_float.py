"""A float for the Cessna 172 floatplane (EDO 89-2000, whose lines aren't public): NACA float model 57-B-5 (NACA TN
716, Parkinson & Olson 1939: Table IV for its lengthened bow, Table II, model 57-B, from station 5 aft; inches, depth
below the base line at the deck), scaled as NACA sized it for a light twin-float airplane.

57-B-5: 94 in long (bow 10 in ahead of 57-B's F.P.), 12 in chine beam (12.47 over the spray strips), 11.40 in deep
at the step, 25 deg deadrise at the step, step 52 in (55%) from the bow, step depth 0.85 in, afterbody keel 7 deg.

NACA TN 1524 (Parkinson 1948) put a 2500 lb seaplane ("B") on 57-B-5 floats 16.70 ft long, 2.215 ft over the spray
strips: a gross load coefficient 1.80 per float. The same coefficient at the 172's 2220 lb (beam as weight^1/3):
2.128 ft over the spray strips, a uniform scale of 2.048 on the model: 16.05 ft long, 24.6 in chine beam, 23.4 in
deep (EDO 89-2000: 16 ft 2 in to 17 ft long and 24 in deep as reported; 2000 lb displacement each).

    import edo_float     STATIONS (x from the bow, in), SX, SY, SZ (m per inch), LENGTH_IN, STEP_X_IN
"""
import math

IN = 0.0254
LENGTH_IN = 94.0
SCALE = 2.128 * 12.0 / 12.47          # model inches per model inch at the 172's float size
SX = SY = SZ = SCALE * IN
STEP_X_IN = 52.0
BOW = 10.0          # 57-B-5's bow is 10 in ahead of the table's F.P.


def deck_arc(chine_hb, chine_depth, radius, n=6):
    """Rounded deck from the chine up to the centre line: a circle of the given radius centred on the centre line
    `radius` below the base line (it passes through the chine point: the table's radius and chine agree)."""
    a0 = math.asin(min(1.0, chine_hb / radius))
    return [(radius * math.sin(a0 * (1 - k / n)), radius - radius * math.cos(a0 * (1 - k / n))) for k in range(1, n + 1)]


def _st(name, x, keel, buttocks, chine_depth, chine_hb, radius=None):
    """keel; buttocks (depths at 1.2, 2.4, 3.6, 4.8 in out, None where not given); chine; then the deck: an arc of the
    deck radius, or the side straight up to the base line."""
    pts = [(0.0, keel)]
    for b, d in zip((1.2, 2.4, 3.6, 4.8), buttocks):
        if d is not None and b < chine_hb:
            pts.append((b, d))
    pts.append((chine_hb, chine_depth))
    if radius:
        pts += deck_arc(chine_hb, chine_depth, radius)
        pts[-1] = (1e-3, 0.0)     # the arc meets the centre line at the base line
    else:
        pts.append((chine_hb, 0.0))
    return (name, x + BOW, keel, pts)


N = None
STATIONS = [
    # 57-B-5 bow (Table IV); the bow point a small square at the deck
    ("bow", -10.0 + BOW, 0.49, [(0.0, 0.49), (0.05, 0.49), (0.05, 0.0)]),
    _st("a", -8.75, 3.80, (2.10, 1.21, N, N), 1.11, 2.70, 3.84),
    _st("b", -7.50, 5.06, (3.41, 2.24, 1.72, N), 1.71, 3.67, 4.79),
    _st("c", -5.00, 6.82, (5.30, 3.98, 3.08, N), 2.83, 4.74, 5.39),
    _st("d", -2.50, 8.09, (6.66, 5.42, 4.47, 3.92), 3.88, 5.30, 5.56),
    _st("1/4", 1.05, 9.35, (8.09, 6.95, 6.02, 5.35), 5.18, 5.70, 5.73),
    _st("1", 4.20, 10.16, (8.98, 7.90, 7.02, 6.37), 6.14, 5.86),
    _st("2", 8.40, 10.88, (9.80, 8.83, 8.05, 7.48), 7.10, 5.95),
    _st("3", 12.60, 11.27, (10.35, 9.52, 8.80, 8.21), 7.77, 5.98),
    _st("4", 16.80, 11.39, (10.64, 9.95, 9.29, 8.73), 8.27, 6.00),
    # 57-B (Table II) from station 5; straight bottoms from 6 aft
    _st("5", 21.00, 11.40, (10.78, 10.18, 9.59, 9.02), 8.53, 6.00),
    _st("6", 25.20, 11.40, (N, N, N, N), 8.60, 6.00),
    _st("7", 29.40, 11.40, (N, N, N, N), 8.60, 6.00),
    _st("8", 33.60, 11.40, (N, N, N, N), 8.62, 5.97),
    _st("9", 37.80, 11.40, (N, N, N, N), 8.67, 5.86),
    _st("10 fwd", 42.00, 11.40, (N, N, N, N), 8.74, 5.70),
    _st("10 aft", 42.05, 10.55, (N, N, N, N), 7.89, 5.70),
    _st("11", 46.20, 10.03, (N, N, N, N), 7.47, 5.49),
    _st("12", 50.40, 9.52, (N, N, N, N), 7.07, 5.24),
    _st("13", 54.60, 9.00, (N, N, N, N), 6.70, 4.93),
    _st("14", 58.80, 8.49, (N, N, N, N), 6.35, 4.57),
    _st("15", 63.00, 7.97, (N, N, N, N), 6.02, 4.18),
    _st("16", 67.20, 7.45, (N, N, N, N), 5.70, 3.77),
    _st("17", 71.40, 6.94, (N, N, N, N), 5.38, 3.35),
    _st("18", 75.60, 6.42, (N, N, N, N), 5.06, 2.92),
    _st("19", 79.80, 5.91, (N, N, N, N), 4.75, 2.47),
    _st("A.P.", 84.00, 5.39, (N, N, N, N), 4.46, 2.00),
]
STEP_X_IN = 42.0 + BOW
