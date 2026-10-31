"""Route planning in the local frame.

* A 3-D grid (north x east x altitude layers) is rasterised from the zones with
  numpy: a cell is blocked if a blocking zone, inflated by the vehicle's margin
  (safety + GPS error + turn radius for fixed-wing), covers it at any time in
  the swept horizon (drifting zones).
* A* finds the cheapest grid route (octile + climb cost heuristic).
* The route is shortened by line-of-sight string pulling, checked against the
  true zone geometry, so the autopilot gets a handful of waypoints instead of
  one per cell.
* A reverse Dijkstra field from home gives "distance home" from any cell in
  O(1), which the bingo (point-of-no-return) check queries every second.
* Hold patterns (racetrack, figure-8, orbit) are fitted around a clear fix.
"""
import heapq
import math
import time

import numpy as np

from .zones import blocks

SQ2 = math.sqrt(2.0)
MAX_CELLS_2D = 40000


class PlanResult:
    def __init__(self, ok, path=None, reason="", ms=0.0, expansions=0, raw_len=0):
        self.ok, self.path, self.reason, self.ms, self.expansions, self.raw_len = ok, path, reason, ms, expansions, raw_len

    def __repr__(self):
        return f"PlanResult(ok={self.ok}, wps={len(self.path or [])}, reason={self.reason!r}, {self.ms:.0f} ms)"


def _zone_dist_np(z, N, E, t):
    """Vectorised signed distance of grid centres to zone z at time t."""
    dn, de = z.offset(t)
    n, e = N - dn, E - de
    if z.poly:
        pts = z.poly
        inside = np.zeros(N.shape, dtype=bool)
        d = np.full(N.shape, np.inf)
        j = len(pts) - 1
        for i in range(len(pts)):
            (yi, xi), (yj, xj) = pts[i], pts[j]
            # ray casting in (n, e)
            cond = ((yi > n) != (yj > n)) & (e < (xj - xi) * (n - yi) / (yj - yi + 1e-12) + xi)
            inside ^= cond
            vn, ve = yj - yi, xj - xi
            L2 = vn * vn + ve * ve
            tt = np.zeros(N.shape) if L2 == 0 else np.clip(((n - yi) * vn + (e - xi) * ve) / L2, 0, 1)
            d = np.minimum(d, np.hypot(n - (yi + tt * vn), e - (xi + tt * ve)))
            j = i
        return np.where(inside, -d, d)
    out = np.full(N.shape, np.inf)
    for cn, ce, r, _ in z.circles:
        out = np.minimum(out, np.hypot(n - cn, e - ce) - r)
    return out


class Grid:
    def __init__(self, zones, profile, points, t=None, horizon=0.0, pad=None):
        """points: (n, e) positions that must be inside the grid (home, goals, vehicle)."""
        self.p = profile
        self.margin = profile["margin_m"]
        self.vmargin = profile["vert_margin_m"]
        self.t, self.horizon = t, horizon
        self.layers = [float(a) for a in profile["alt_layers_m"]]
        pad = pad if pad is not None else max(4 * profile["cell_m"], 2 * self.margin, 200.0)
        ns = [p[0] for p in points]
        es = [p[1] for p in points]
        n0, n1, e0, e1 = min(ns) - pad, max(ns) + pad, min(es) - pad, max(es) + pad
        cell = float(profile["cell_m"])
        while ((n1 - n0) / cell + 1) * ((e1 - e0) / cell + 1) > MAX_CELLS_2D:
            cell *= 1.25                               # keep A* fast on a companion computer
        self.cell = cell
        self.n0, self.e0 = n0, e0
        self.rows = int(math.ceil((n1 - n0) / cell)) + 1
        self.cols = int(math.ceil((e1 - e0) / cell)) + 1
        L = len(self.layers)
        N = n0 + np.arange(self.rows)[:, None] * cell + np.zeros((1, self.cols))
        E = e0 + np.arange(self.cols)[None, :] * cell + np.zeros((self.rows, 1))
        self.blocked = np.zeros((L, self.rows, self.cols), dtype=bool)
        self.soft = np.zeros((L, self.rows, self.cols))
        self.src = {}
        times = [t] if (t is None or horizon <= 0) else list(np.linspace(t, t + horizon, 4))
        self.zones = zones
        # cells whose centre is within margin + half a cell diagonal are blocked, so
        # straight legs between free neighbours keep the full margin
        inflate = self.margin + 0.5 * cell * SQ2 * 0.5
        for z in zones:
            blk, cost = blocks(z, profile)
            d = None
            for tt in times:
                dz = _zone_dist_np(z, N, E, tt)
                d = dz if d is None else np.minimum(d, dz)
            mask2 = d < (inflate if blk else 0.0)
            for li, alt in enumerate(self.layers):
                if not z.in_alt(alt, self.vmargin):
                    continue
                if blk:
                    newly = mask2 & ~self.blocked[li]
                    self.blocked[li] |= mask2
                    for (r, c) in zip(*np.nonzero(newly)):
                        self.src[(li, int(r), int(c))] = z.id
                else:
                    self.soft[li][mask2] += cost

    # ------------------------------------------------------------ indexing
    def cell_of(self, n, e, alt):
        r = int(round((n - self.n0) / self.cell))
        c = int(round((e - self.e0) / self.cell))
        li = min(range(len(self.layers)), key=lambda i: abs(self.layers[i] - alt))
        return (li, max(0, min(self.rows - 1, r)), max(0, min(self.cols - 1, c)))

    def centre(self, k):
        li, r, c = k
        return (self.n0 + r * self.cell, self.e0 + c * self.cell, self.layers[li])

    def free(self, k):
        li, r, c = k
        return 0 <= li < len(self.layers) and 0 <= r < self.rows and 0 <= c < self.cols and not self.blocked[li, r, c]

    def neighbors(self, k):
        li, r, c = k
        for dr, dc, w in ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                          (1, 1, SQ2), (1, -1, SQ2), (-1, 1, SQ2), (-1, -1, SQ2)):
            n = (li, r + dr, c + dc)
            if self.free(n) and (w == 1.0 or (self.free((li, r + dr, c)) and self.free((li, r, c + dc)))):
                yield n, w * self.cell * (1.0 + self.soft[li, r + dr, c + dc])
        for dl in (1, -1):
            n = (li + dl, r, c)
            if self.free(n):
                dz = abs(self.layers[li + dl] - self.layers[li])
                # fixed-wing cannot climb in place: charge the horizontal distance a climb needs
                g = self.p.get("max_gradient")
                extra = dz / g if g else 0.0
                yield n, dz * self.p["climb_cost"] + extra

    def heuristic(self, a, b):
        dr, dc = abs(a[1] - b[1]), abs(a[2] - b[2])
        dz = abs(self.layers[a[0]] - self.layers[b[0]])
        return (dr + dc + (SQ2 - 2) * min(dr, dc)) * self.cell + dz * self.p["climb_cost"]

    # ------------------------------------------------------------ geometry checks
    def point_clear(self, n, e, alt, t=None, margin=None):
        """Which blocking zone (id) covers this point with margin, or None."""
        m = self.margin if margin is None else margin
        t = self.t if t is None else t
        for z in self.zones:
            if not blocks(z, self.p)[0] or not z.in_alt(alt, self.vmargin):
                continue
            if z.distance(n, e, t) < m:
                return z.id
        return None

    def min_dist(self, n, e, alt, t=None):
        """Distance to the nearest blocking zone in this altitude band (inf if none),
        over the swept horizon when no time is given (a drifting zone counts where it is going)."""
        if t is not None or self.t is None or self.horizon <= 0:
            times = [self.t if t is None else t]
        else:
            times = [self.t, self.t + self.horizon / 2, self.t + self.horizon]
        d = math.inf
        for z in self.zones:
            if blocks(z, self.p)[0] and z.in_alt(alt, self.vmargin):
                for tt in times:
                    d = min(d, z.distance(n, e, tt))
        return d

    def segment_conflict(self, a, b, clear=None, times=None):
        """First zone a straight 3-D leg a->b passes within `clear` metres of
        (default: the full margin), or None."""
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        step = max(2.0, min(self.cell / 3.0, 10.0))
        k = max(1, int(L / step))
        times = times or ([self.t] if (self.t is None or self.horizon <= 0)
                          else [self.t, self.t + self.horizon / 2, self.t + self.horizon])
        for i in range(k + 1):
            f = i / k
            n, e = a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
            alt = a[2] + f * (b[2] - a[2])
            for tt in times:
                zid = self.point_clear(n, e, alt, tt, margin=clear)
                if zid:
                    return zid
        return None

    def gradient_ok(self, a, b):
        g = self.p.get("max_gradient")
        if not g:
            return True
        h = math.hypot(b[0] - a[0], b[1] - a[1])
        return abs(b[2] - a[2]) <= g * h + 1e-6


def astar(grid, start, goal, max_expansions=400000):
    t0 = time.perf_counter()
    if not grid.free(goal):
        return None, 0, (time.perf_counter() - t0) * 1000
    openh = [(grid.heuristic(start, goal), 0.0, start)]
    g = {start: 0.0}
    parent = {start: None}
    exp = 0
    while openh:
        f, gc, k = heapq.heappop(openh)
        if gc > g.get(k, math.inf):
            continue
        if k == goal:
            path = []
            while k is not None:
                path.append(k)
                k = parent[k]
            return path[::-1], exp, (time.perf_counter() - t0) * 1000
        exp += 1
        if exp > max_expansions:
            break
        for n, w in grid.neighbors(k):
            ng = gc + w
            if ng < g.get(n, math.inf):
                g[n] = ng
                parent[n] = k
                heapq.heappush(openh, (ng + grid.heuristic(n, goal), ng, n))
    return None, exp, (time.perf_counter() - t0) * 1000


class HomeField:
    """Reverse Dijkstra from home: shortest flyable distance home from every cell,
    built once per hazard picture and queried in O(1) by the bingo check."""

    def __init__(self, grid, home_cell):
        self.grid = grid
        self.dist = {}
        if not grid.free(home_cell):
            # home itself inside an inflated zone (e.g. a zone right next to the pad):
            # start from the nearest free cell on the lowest layer
            home_cell = nearest_free(grid, home_cell) or home_cell
        h = [(0.0, home_cell)]
        self.dist[home_cell] = 0.0
        while h:
            d, k = heapq.heappop(h)
            if d > self.dist.get(k, math.inf):
                continue
            for n, _ in grid.neighbors(k):
                # length (not cost): what the battery pays for
                if n[0] != k[0]:
                    w = abs(grid.layers[n[0]] - grid.layers[k[0]])
                else:
                    w = grid.cell * (SQ2 if (n[1] != k[1] and n[2] != k[2]) else 1.0)
                nd = d + w
                if nd < self.dist.get(n, math.inf):
                    self.dist[n] = nd
                    heapq.heappush(h, (nd, n))

    def distance(self, n, e, alt):
        """Metres home from a position, or None if home is unreachable from here."""
        k = self.grid.cell_of(n, e, alt)
        if k in self.dist:
            cn, ce, ca = self.grid.centre(k)
            return self.dist[k] + math.hypot(n - cn, e - ce)
        best = None
        for nb, _ in self.grid.neighbors(k):
            if nb in self.dist:
                best = self.dist[nb] + self.grid.cell * SQ2 if best is None else min(best, self.dist[nb] + self.grid.cell * SQ2)
        return best


def nearest_free(grid, k, max_r=None):
    li, r, c = k
    max_r = max_r or max(grid.rows, grid.cols)
    for rad in range(1, max_r):
        ring = [(li, r + dr, c + dc) for dr in range(-rad, rad + 1) for dc in range(-rad, rad + 1)
                if max(abs(dr), abs(dc)) == rad]
        ring = [q for q in ring if grid.free(q)]
        if ring:
            return min(ring, key=lambda q: math.hypot(q[1] - r, q[2] - c))
    return None


def start_clearance(grid, start):
    """If the vehicle is already inside a zone's margin (it just found ash, or a
    zone drifted onto it), legs from the start may not get any closer than it is
    now: returns that reduced clearance, or None when the start is outside every margin."""
    d0 = grid.min_dist(*start)
    if d0 >= grid.margin:
        return None
    return max(0.0, d0 - 1.0)


def find_exit(grid, start, goal):
    """Best free cell centre reachable from a start inside a margin by a straight
    leg whose clearance never decreases (so the vehicle only ever moves away).
    Searches progressively farther (a drifting cloud can sweep a wide area)."""
    for scale in (1, 2, 3):
        p = _find_exit(grid, start, goal, scale * (2 * grid.margin + 2 * grid.cell))
        if p is not None:
            return p
    return None


def _find_exit(grid, start, goal, R):
    li = grid.cell_of(*start)[0]
    k = int(R / grid.cell) + 1
    s = grid.cell_of(*start)
    best = None
    for r in range(max(0, s[1] - k), min(grid.rows, s[1] + k + 1)):
        for c in range(max(0, s[2] - k), min(grid.cols, s[2] + k + 1)):
            cell = (li, r, c)
            if not grid.free(cell):
                continue
            n, e, _ = grid.centre(cell)
            L = math.hypot(n - start[0], e - start[1])
            if L > R or L < 1.0:
                continue
            p = (n, e, start[2])
            prev, ok = -math.inf, True
            for i in range(21):
                f = i / 20
                d = grid.min_dist(start[0] + f * (n - start[0]), start[1] + f * (e - start[1]), start[2])
                if d < prev - 0.5:
                    ok = False
                    break
                prev = max(prev, d)
            if not ok or grid.point_clear(n, e, start[2]):
                continue
            score = L + math.hypot(goal[0] - n, goal[1] - e)
            if best is None or score < best[0]:
                best = (score, p)
    return best[1] if best else None


def smooth(grid, pts, first_clear=None):
    """Line-of-sight string pulling on the true zone geometry."""
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1:
            clear = first_clear if i == 0 else None
            if grid.gradient_ok(pts[i], pts[j]) and grid.segment_conflict(pts[i], pts[j], clear) is None:
                break
            j -= 1
        out.append(pts[j])
        i = j
    return out


def plan(zones, profile, start, goal, t=None, horizon=0.0, extra_points=()):
    """Plan start -> goal; both are (n, e, alt). Returns PlanResult with the
    smoothed waypoint list (first point = start, last = goal)."""
    t0 = time.perf_counter()
    grid = Grid(zones, profile, [start[:2], goal[:2]] + list(extra_points), t=t, horizon=horizon)
    s = grid.cell_of(*start)
    gk = grid.cell_of(*goal)
    if grid.point_clear(goal[0], goal[1], max(goal[2], grid.layers[0])) or not grid.free(gk):
        return PlanResult(False, reason=f"goal blocked by {grid.point_clear(*goal[:2], max(goal[2], grid.layers[0])) or grid.src.get(gk, 'margin')}",
                          ms=(time.perf_counter() - t0) * 1000), grid
    first_clear = start_clearance(grid, start)
    exit_pt = None
    if first_clear is not None:
        # inside a margin: first leave along a straight leg on which the distance to
        # every hazard only grows, to the nearest point fully outside the margin
        exit_pt = find_exit(grid, start, goal)
        if exit_pt is None:
            return PlanResult(False, reason="inside a hazard margin with no straight way out",
                              ms=(time.perf_counter() - t0) * 1000), grid
        s = grid.cell_of(*exit_pt)
    elif not grid.free(s):
        grid.blocked[s] = False                      # start cell only grazed by the inflation
    path, exp, _ = astar(grid, s, gk)
    if path is None:
        return PlanResult(False, reason="no route", ms=(time.perf_counter() - t0) * 1000, expansions=exp), grid
    head = [tuple(start)] if exit_pt is None else [tuple(start), tuple(exit_pt)]
    pts = head[-1:] + [grid.centre(k) for k in path[1:-1]] + [tuple(goal)]
    pts = head[:-1] + smooth(grid, pts)
    problems = route_problems(grid, pts, first_clear)
    if problems:
        return PlanResult(False, reason="smoothed route not clear: " + ", ".join(problems), ms=(time.perf_counter() - t0) * 1000,
                          expansions=exp), grid
    return PlanResult(True, pts, "ok", (time.perf_counter() - t0) * 1000, exp, len(path)), grid


def route_problems(grid, pts, first_clear=None, times=None):
    out = []
    for i in range(len(pts) - 1):
        z = grid.segment_conflict(pts[i], pts[i + 1], first_clear if i == 0 else None, times)
        if z:
            out.append(f"leg {i + 1} crosses {z}")
    return out


# ---------------------------------------------------------------- hold patterns
def _arc(cn, ce, r, a0, a1, n):
    return [(cn + r * math.cos(a0 + (a1 - a0) * k / n), ce + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def hold_pattern(kind, fix, inbound_deg, radius, leg=None):
    """Waypoints (n, e) of one circuit, starting at the fix. inbound_deg is the
    course flown TOWARD the fix. Right turns for the standard racetrack (ICAO)."""
    u = (math.cos(math.radians(inbound_deg)), math.sin(math.radians(inbound_deg)))   # along course
    rt = (-u[1], u[0])                                                                  # right of course (n,e frame)
    if kind == "racetrack_left":
        rt = (-rt[0], -rt[1])
    fn, fe = fix
    leg = leg if leg is not None else 2.5 * radius
    if kind in ("racetrack", "racetrack_left"):
        # turn at the fix (centre beside it), outbound leg, turn, inbound leg
        c1 = (fn + rt[0] * radius, fe + rt[1] * radius)
        c2 = (c1[0] - u[0] * leg, c1[1] - u[1] * leg)
        a_fix = math.atan2(fn - c1[0], fe - c1[1])
        # parametrise angles in (n, e) -> use atan2(dn, de) consistently
        sgn = -1 if kind == "racetrack" else 1
        pts = [(fn, fe)]
        for k in range(1, 5):                                  # 180 deg turn at the fix end
            a = a_fix + sgn * math.pi * k / 4
            pts.append((c1[0] + radius * math.sin(a), c1[1] + radius * math.cos(a)))
        a_out = a_fix + sgn * math.pi
        for k in range(0, 5):                                  # 180 deg turn at the far end
            a = a_out + sgn * math.pi * k / 4
            pts.append((c2[0] + radius * math.sin(a), c2[1] + radius * math.cos(a)))
        return _dedupe(pts)
    if kind == "figure8":
        pts = []
        for side, sgn in ((1, -1), (-1, 1)):
            c = (fn + side * rt[0] * radius, fe + side * rt[1] * radius)
            a0 = math.atan2(fn - c[0], fe - c[1])
            for k in range(0, 8):
                a = a0 + sgn * 2 * math.pi * k / 8
                pts.append((c[0] + radius * math.sin(a), c[1] + radius * math.cos(a)))
        return _dedupe(pts)
    if kind == "orbit":
        a0 = math.pi
        c = (fn + rt[0] * radius, fe + rt[1] * radius)
        a0 = math.atan2(fn - c[0], fe - c[1])
        return _dedupe([(c[0] + radius * math.sin(a0 - 2 * math.pi * k / 8), c[1] + radius * math.cos(a0 - 2 * math.pi * k / 8))
                        for k in range(8)])
    raise ValueError(kind)


def choose_landing(grid, td, wind_from_deg, profile, cruise_alt):
    """Fixed-wing landing at td (n, e): orbit down at a clear spot, then a straight
    final approach into the wind. Geometry follows PX4's feasibility check for an
    ORBIT_TO_ALT entrance (tangent length / glide slope) and suits ArduPlane too.
    Returns dict(centre=(n, e), radius, heading, alt) or None if nothing is clear."""
    r = profile["loiter_radius_m"]
    L = profile["approach_len_m"]
    h_app = profile["approach_alt_m"]
    base = wind_from_deg if wind_from_deg is not None else 0.0
    for off in (0, 30, -30, 60, -60, 90, -90, 120, -120, 150, -150, 180):
        hdg = (base + off) % 360                      # landing heading: into the wind first
        u = (math.cos(math.radians(hdg)), math.sin(math.radians(hdg)))
        rt = (-u[1], u[0])
        A = (td[0] - u[0] * L, td[1] - u[1] * L)      # where the final approach starts (tangent point)
        C = (A[0] + rt[0] * r, A[1] + rt[1] * r)      # clockwise orbit: centre on the right of the track
        ok = grid.segment_conflict((A[0], A[1], h_app), (td[0], td[1], 0.0)) is None
        if ok:
            ring = [(C[0] + r * math.cos(a), C[1] + r * math.sin(a)) for a in [k * math.pi / 8 for k in range(16)]]
            for z in (h_app, cruise_alt):
                if not pattern_clear(grid, ring, z):
                    ok = False
                    break
        if ok:
            return {"centre": C, "radius": r, "heading": hdg, "alt": h_app, "td": td}
    return None


def _dedupe(pts):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > 1.0:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) < 1.0:
        out.pop()
    return out


def pattern_clear(grid, pts, alt):
    closed = pts + [pts[0]]
    for a, b in zip(closed, closed[1:]):
        if grid.segment_conflict((a[0], a[1], alt), (b[0], b[1], alt)):
            return False
    return True


def choose_hold(grid, goal, inbound_deg, profile):
    """Find a clear fix near a blocked goal and a hold pattern that fits there.
    Returns dict(fix=(n,e,alt), kind, pts, radius) or None."""
    prefer = profile["hold_pattern"]
    kinds = [prefer] + [k for k in ("racetrack", "racetrack_left", "figure8", "orbit") if k != prefer]
    rmin = max(profile["turn_radius_m"] * 1.1, 10.0)
    gk = grid.cell_of(*goal)
    cands = []
    for li in range(len(grid.layers)):
        for r in range(grid.rows):
            for c in range(grid.cols):
                k = (li, r, c)
                if grid.free(k):
                    n, e, a = grid.centre(k)
                    cands.append((math.hypot(n - goal[0], e - goal[1]) + 0.2 * abs(a - goal[2]), k))
    cands.sort()
    for _, k in cands[:60]:
        n, e, alt = grid.centre(k)
        if grid.point_clear(n, e, alt):
            continue
        for kind in kinds:
            r = profile["loiter_radius_m"]
            while r >= rmin:
                for rot in (0, 90, 180, 270, 45, 135, 225, 315):
                    pts = hold_pattern(kind, (n, e), (inbound_deg + rot) % 360, r)
                    if pattern_clear(grid, pts, alt):
                        return {"fix": (n, e, alt), "kind": kind, "pts": pts, "radius": r, "inbound_deg": (inbound_deg + rot) % 360}
                r *= 0.7
    return None
