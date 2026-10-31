"""Holding patterns flown as real paths (not an on-screen animation).

    racetrack       ICAO standard hold: inbound leg to the fix, right turns,
                    outbound leg parallel on the right-hand side
    racetrack_left  non-standard hold: same with left turns
    figure8         figure-of-eight through the fix (common for ISR / tanker tracks)
    orbit           constant-radius loiter circle around the fix (defense / UAS loiter)

The pattern is built on the inbound course (fix -> the blocked landing site),
so the outbound side faces away from the site. It is checked against the known
map; if it does not fit it is rotated, then another pattern is tried, then a
tight orbit inside the fix cell is used. Sizes are in grid cells.
"""
import math

KINDS = {
    "racetrack": "Racetrack, right turns (ICAO standard)",
    "racetrack_left": "Racetrack, left turns (non-standard)",
    "figure8": "Figure-8 (ISR / tanker track)",
    "orbit": "Orbit (loiter circle)",
}
SIZE = {"small": (0.9, 2.6), "aircraft": (0.8, 2.2)}     # turn radius, leg length (cells)


def _arc(cx, cy, r, a0, a1, n):
    return [(cx + r * math.cos(a0 + (a1 - a0) * k / n), cy + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(1, n + 1)]


def _line(p, q, n):
    return [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n) for k in range(1, n + 1)]


def pattern(kind, fix, u, mode="small", scale=1.0):
    """Closed loop of (x, y) points starting at the fix; u = inbound unit vector."""
    R, leg = SIZE.get(mode, SIZE["small"])
    R, leg = R * scale, leg * scale
    fx, fy = float(fix[0]), float(fix[1])
    ux, uy = u
    rx, ry = -uy, ux                                  # right of the inbound course (+y is south on screen)
    hd = math.atan2(uy, ux)
    if kind in ("racetrack", "racetrack_left"):
        s = 1 if kind == "racetrack" else -1
        c1 = (fx + s * R * rx, fy + s * R * ry)                     # turn at the fix
        a0 = math.atan2(fy - c1[1], fx - c1[0])
        pts = [(fx, fy)] + _arc(c1[0], c1[1], R, a0, a0 + s * math.pi, 10)
        p_out_end = (pts[-1][0] - ux * leg, pts[-1][1] - uy * leg)
        pts += _line(pts[-1], p_out_end, 5)
        c2 = (c1[0] - ux * leg, c1[1] - uy * leg)
        a1 = math.atan2(p_out_end[1] - c2[1], p_out_end[0] - c2[0])
        pts += _arc(c2[0], c2[1], R, a1, a1 + s * math.pi, 10)
        pts += _line(pts[-1], (fx, fy), 5)
        return pts
    if kind == "figure8":
        # two loops in line behind the fix, crossing between them: fix -> half of
        # loop 1 -> the whole of loop 2 the other way round -> back up loop 1 to the fix
        r8 = R * 0.95
        c1 = (fx - ux * r8, fy - uy * r8)
        c2 = (fx - 3 * ux * r8, fy - 3 * uy * r8)
        a0 = math.atan2(fy - c1[1], fx - c1[0])
        pts = [(fx, fy)] + _arc(c1[0], c1[1], r8, a0, a0 + math.pi, 12)
        ax = math.atan2(pts[-1][1] - c2[1], pts[-1][0] - c2[0])
        pts += _arc(c2[0], c2[1], r8, ax, ax - 2 * math.pi, 24)
        pts += _arc(c1[0], c1[1], r8, a0 + math.pi, a0 + 2 * math.pi, 12)[:-1]
        return pts
    r = 1.2 * scale
    c = (fx - ux * r, fy - uy * r)
    a0 = math.atan2(fy - c[1], fx - c[0])
    return [(fx, fy)] + _arc(c[0], c[1], r, a0, a0 + 2 * math.pi, 28)


def fits(grid, pts, z):
    for x, y in pts:
        c = (int(round(x)), int(round(y)), z)
        if not grid.free(c):
            return False
    return True


def choose(grid, fix, toward, prefer="racetrack", mode="small"):
    """Pick a pattern that fits: preferred kind on the inbound course, then
    rotated, then smaller, then other kinds, then a tight orbit."""
    ux, uy = toward[0] - fix[0], toward[1] - fix[1]
    n = math.hypot(ux, uy) or 1.0
    base = (ux / n, uy / n)
    kinds = [prefer] + [k for k in ("racetrack", "racetrack_left", "figure8", "orbit") if k != prefer]
    for kind in kinds:
        for scale in (1.0, 0.75):
            for rot in (0, 90, -90, 180):
                a = math.radians(rot)
                u = (base[0] * math.cos(a) - base[1] * math.sin(a), base[0] * math.sin(a) + base[1] * math.cos(a))
                pts = pattern(kind, fix, u, mode, scale)
                if fits(grid, pts, fix[2]):
                    return kind, pts, round(math.degrees(math.atan2(u[1], u[0])) % 360), scale
    return "orbit", pattern("orbit", fix, base, mode, 0.35), 0, 0.35


def arc_lengths(pts):
    cum = [0.0]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        cum.append(cum[-1] + math.dist(a, b))
    return cum


def point_at(pts, cum, s):
    total = cum[-1]
    s = s % total if total > 0 else 0.0
    for i in range(len(pts)):
        if cum[i + 1] >= s:
            a, b = pts[i], pts[(i + 1) % len(pts)]
            seg = cum[i + 1] - cum[i]
            f = 0.0 if seg <= 1e-9 else (s - cum[i]) / seg
            return a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f
    return pts[0]
