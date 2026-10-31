"""Headless 3-D A* planner, ported from main.py.

Kept from main.py:
  * 8-way horizontal moves plus up/down; a diagonal is allowed only when both
    adjacent cardinal cells are free (no corner cutting).
  * Move cost = geometric cost (1, sqrt 2, or VERTICAL_COST for a layer change)
    + wind penalty when moving against the wind + cost of soft hazards entered.
  * Octile heuristic + VERTICAL_COST per layer (admissible and consistent).
  * Ash reaching the top layer becomes a full-height exclusion volume.
New for Janus:
  * Blocking is decided per drone profile (turbulence threshold, gust limit,
    rain rating, ceiling) instead of global rules.
  * Only hazards passed in are considered, so the drone plans with what it
    KNOWS, not with the whole truth (main.py's replan saw every hazard).
  * Optional expansion budget so compute overrun can be demonstrated.
"""
import heapq
import math
import time

VERTICAL_COST = 2.0
DIAG = math.sqrt(2.0)
SOLID_TYPES = {"building", "geofence", "ridge", "tfr"}
TURB_SOFT_COST = {1: 2.0, 2: 8.0, 3: 28.0}      # main.py TURBULENCE_COST


def hazard_zrange(h, layers):
    z0, z1 = int(h["z0"]), int(h["z1"])
    if h["type"] == "ash" and z1 >= layers - 1:   # main.py ceiling rule
        return 0, layers - 1
    return z0, z1


ASH_SAR_SOFT_COST = 30.0   # SAR may cross LOW-concentration ash, but it costs a lot
SURFACE_COST = 3.0        # extra cost per horizontal step on layer 0: climb out after launch, no taxiing en route
ASH_BUFFER_COST = 4.0      # soft cost for the one-cell ring around ash: keeps routes off the diffuse edge


# ---------------------------------------------------------------- zone geometry
# A zone is a box (x0..x1, y0..y1 inclusive cells) or a circle (cx, cy, r in
# cells, shape="circle"). Weather zones may drift: vx, vy in cells per second
# from t_on (or t_ref). Everything that asks "is this cell inside?" goes
# through these functions, so circles and drift work everywhere.

def zone_offset(h, t):
    vx, vy = float(h.get("vx", 0.0)), float(h.get("vy", 0.0))
    if not vx and not vy:
        return 0.0, 0.0
    dt = max(0.0, t - float(h.get("t_ref", h.get("t_on", 0.0))))
    return vx * dt, vy * dt


# ---------------------------------------------------------------- volcanic plume
# A plume is ONE zone: a vent that releases a puff every `every` seconds. Each
# puff drifts with the wind (vx, vy), spreads (r0 + dr * age) and thins by
# spreading (mass kept, c ~ 1/r^2) and by fall-out (exp(-age / tau)); it is
# gone once it is older than `life`. Every node expands the same plume the
# same way, so a whole plume costs one advisory on the radio link.

def plume_puffs(h, t):
    if h.get("shape") != "plume":
        return [h]
    if h.get("freeze") is not None:
        t = float(h["freeze"])
    every, life, t0 = float(h["every"]), float(h["life"]), float(h.get("t_start", 0.0))
    t_stop = float(h.get("t_stop", 1e9))
    k_lo = max(0, int(math.ceil((t - life - t0) / every)))
    k_hi = int(math.floor((min(t, t_stop) - t0) / every))
    vx, vy = (0.0, 0.0) if h.get("freeze") is not None else (float(h.get("vx", 0.0)), float(h.get("vy", 0.0)))
    ux, uy = float(h.get("ux", 1.0)), float(h.get("uy", 0.0))           # downwind unit (for frozen plumes)
    spd = float(h.get("speed", 0.0))
    out = []
    for k in range(k_lo, k_hi + 1):
        tk = t0 + k * every
        age = t - tk
        if age < 0 or age > life:
            continue
        wob = float(h.get("meander", 0.3)) * math.sin(tk * 0.7 + float(h.get("phase", 0.0)))
        cx = h["vent"][0] + ux * spd * age - uy * wob
        cy = h["vent"][1] + uy * spd * age + ux * wob
        r = float(h["r0"]) + float(h["dr"]) * age
        c = float(h["c0"]) * (float(h["r0"]) / r) ** 2 * math.exp(-age / float(h["tau"]))
        if c < 0.2:
            continue                                   # dispersed below the LOW band
        out.append({"id": h["id"], "type": "ash", "shape": "circle", "cx": cx, "cy": cy, "r": r, "dr": h["dr"],
                    "tau": h["tau"], "conc_mg": c, "sev": 3 if c >= 4 else 2 if c >= 2 else 1, "vx": vx, "vy": vy,
                    "t_ref": t, "t_on": 0.0, "z0": h["z0"], "z1": h["z1"], "puff": True})
    return out


def zone_r(h, t=0.0):
    """Radius of a circular zone at time t. Volcanic puffs spread as they age
    (dr cells per second since t_ref)."""
    r = float(h["r"])
    if h.get("dr"):
        r += float(h["dr"]) * max(0.0, t - float(h.get("t_ref", h.get("t_on", 0.0))))
    return r


def zone_conc(h, t=0.0):
    """Ash concentration at time t: a spreading puff keeps its mass, so the
    concentration falls with the square of the radius."""
    if h.get("shape") == "plume":
        return max([zone_conc(p, t) for p in plume_puffs(h, t)] or [0.0])
    c = float(h.get("conc_mg", 5.0))
    if h.get("dr"):
        c *= (float(h["r"]) / zone_r(h, t)) ** 2
    if h.get("tau"):
        c *= math.exp(-max(0.0, t - float(h.get("t_ref", 0.0))) / float(h["tau"]))
    return c


def ash_sev(c):
    return 3 if c >= 4.0 else 2 if c >= 2.0 else 1


def zone_at(h, t):
    """A copy with the severity that applies at time t (spreading puffs thin out)."""
    if h.get("type") != "ash" or not (h.get("dr") or h.get("tau")):
        return h
    g = dict(h)
    g["conc_mg"] = round(zone_conc(h, t), 2)
    g["sev"] = ash_sev(g["conc_mg"])
    return g


def zone_contains_xy(h, x, y, t=0.0):
    if h.get("shape") == "plume":
        return any(zone_contains_xy(p, x, y, t) for p in plume_puffs(h, t))
    dx, dy = zone_offset(h, t)
    if h.get("shape") == "circle":
        return (x - h["cx"] - dx) ** 2 + (y - h["cy"] - dy) ** 2 <= zone_r(h, t) ** 2
    return (h["x0"] + dx - 0.5 <= x < h["x1"] + dx + 0.5) and (h["y0"] + dy - 0.5 <= y < h["y1"] + dy + 0.5)


def zone_bounds(h, t=0.0):
    """Float bounding box (x0, y0, x1, y1) in cell-centre coordinates at time t."""
    if h.get("shape") == "plume":
        bs = [zone_bounds(p, t) for p in plume_puffs(h, t)] or [(-99, -99, -99, -99)]
        return min(b[0] for b in bs), min(b[1] for b in bs), max(b[2] for b in bs), max(b[3] for b in bs)
    dx, dy = zone_offset(h, t)
    if h.get("shape") == "circle":
        r = zone_r(h, t)
        return h["cx"] + dx - r, h["cy"] + dy - r, h["cx"] + dx + r, h["cy"] + dy + r
    return h["x0"] + dx, h["y0"] + dy, h["x1"] + dx, h["y1"] + dy


def zone_cells(h, cols, rows, t=0.0, horizon=0.0):
    """Cells covered at time t; with a horizon, the union over [t, t + horizon]
    (the swept area of a drifting zone), sampled every half second."""
    if h.get("shape") == "plume":
        out = set()
        for p in plume_puffs(h, t):
            out |= zone_cells(p, cols, rows, t, horizon)
        return out
    moving = bool(h.get("vx") or h.get("vy") or h.get("dr"))
    times = [t] if not (moving and horizon > 0) else [t + k * 0.5 for k in range(int(horizon / 0.5) + 1)]
    out = set()
    for tt in times:
        x0, y0, x1, y1 = zone_bounds(h, tt)
        for y in range(max(0, int(math.floor(y0)) - 1), min(rows - 1, int(math.ceil(y1)) + 1) + 1):
            for x in range(max(0, int(math.floor(x0)) - 1), min(cols - 1, int(math.ceil(x1)) + 1) + 1):
                if zone_contains_xy(h, x, y, tt):
                    out.add((x, y))
    return out


def zone_distance(h, x, y, t=0.0):
    """Distance in cells from (x, y) to the zone edge; 0 inside."""
    if h.get("shape") == "plume":
        return min([zone_distance(p, x, y, t) for p in plume_puffs(h, t)] or [99.0])
    dx, dy = zone_offset(h, t)
    if h.get("shape") == "circle":
        return max(0.0, math.hypot(x - h["cx"] - dx, y - h["cy"] - dy) - zone_r(h, t))
    ex = max(h["x0"] + dx - 0.5 - x, 0.0, x - (h["x1"] + dx + 0.5))
    ey = max(h["y0"] + dy - 0.5 - y, 0.0, y - (h["y1"] + dy + 0.5))
    return math.hypot(ex, ey)


def inflate(h, by=1.0):
    """The same zone grown by `by` cells (for the buffer ring around ash)."""
    g = dict(h)
    if h.get("shape") == "plume":
        g["r0"] = h["r0"] + by
        return g
    if h.get("shape") == "circle":
        g["r"] = h["r"] + by
    else:
        g["x0"], g["x1"], g["y0"], g["y1"] = h["x0"] - int(by), h["x1"] + int(by), h["y0"] - int(by), h["y1"] + int(by)
    return g


def zone_center(h, t=0.0):
    x0, y0, x1, y1 = zone_bounds(h, t)
    return (x0 + x1) / 2.0, (y0 + y1) / 2.0


def hazard_blocks(h, profile):
    t = h["type"]
    if t == "tfr" and profile.get("sar"):
        return False                 # SAR / emergency operations may enter a TFR (with coordination)
    if t in SOLID_TYPES:
        return True
    if t == "ash":
        # Every drone avoids MEDIUM/HIGH ash. Only SAR mode may cross LOW.
        return not (profile.get("sar") and int(h.get("sev", 3)) <= 1)
    if t == "turb":
        return int(h.get("sev", 2)) >= int(profile["turb_block"])
    if t == "gust":
        return float(h.get("gust_kt", 0)) > float(profile["gust_limit_kt"])
    if t == "rain":
        return not profile["rain_rated"]
    return False


def hazard_soft_cost(h, profile):
    t = h["type"]
    if t == "tfr":
        return 6.0
    if t == "ash":
        return ASH_SAR_SOFT_COST
    if t == "turb":
        return TURB_SOFT_COST.get(int(h.get("sev", 2)), 8.0)
    if t == "gust":
        lim = max(1.0, float(profile["gust_limit_kt"]))
        return 2.0 + 6.0 * min(1.0, float(h.get("gust_kt", 0)) / lim)
    if t == "rain":
        return 3.0
    return 0.0


def wind_vector(wind_from_deg):
    """Unit grid vector the wind blows TOWARD, quantised to 8 directions.
    Grid +x is east, +y is south (screen coordinates, as in main.py)."""
    toward = (wind_from_deg + 180.0) % 360.0
    rad = math.radians(toward)
    vx, vy = math.sin(rad), -math.cos(rad)
    q = lambda v: 0 if abs(v) < 0.38 else (1 if v > 0 else -1)
    return (q(vx), q(vy))


class Grid:
    """Blocked / soft-cost lookup for one profile and one hazard set."""

    def __init__(self, cols, rows, layers, hazards, profile, wind=(1, 0), wind_penalty=None, t=0.0, horizon=0.0,
                 ash_buffer=True, surface_cost=SURFACE_COST):
        self.cols, self.rows, self.layers = cols, rows, layers
        self.profile = profile
        self.wind = wind
        self.wind_penalty = profile.get("wind_penalty", 3.0) if wind_penalty is None else wind_penalty
        self.vcost = VERTICAL_COST * float(profile.get("climb_mult", 1.0))
        self.surface = surface_cost
        n = cols * rows * layers
        self.blocked = bytearray(n)
        self.soft = [0.0] * n
        self.block_src = {}
        ceiling = int(profile.get("ceiling", layers - 1))
        for z in range(ceiling + 1, layers):
            for y in range(rows):
                for x in range(cols):
                    self.blocked[self.idx(x, y, z)] = 1
        expanded = []
        for h in hazards:
            expanded.extend(plume_puffs(h, t) if h.get("shape") == "plume" else [h])
        for h in expanded:
            h = zone_at(h, t)
            z0, z1 = hazard_zrange(h, layers)
            block = hazard_blocks(h, profile)
            cost = 0.0 if block else hazard_soft_cost(h, profile)
            cells = zone_cells(h, cols, rows, t, horizon)
            if h["type"] == "ash" and ash_buffer:
                ring = zone_cells(inflate(h), cols, rows, t, horizon) - cells
                for z in range(max(0, z0), min(layers - 1, z1) + 1):
                    for (x, y) in ring:
                        self.soft[self.idx(x, y, z)] += ASH_BUFFER_COST
            for z in range(max(0, z0), min(layers - 1, z1) + 1):
                for (x, y) in cells:
                    i = self.idx(x, y, z)
                    if block:
                        self.blocked[i] = 1
                        self.block_src.setdefault(i, h["id"])
                    else:
                        self.soft[i] += cost

    def idx(self, x, y, z):
        return (z * self.rows + y) * self.cols + x

    def inside(self, c):
        return 0 <= c[0] < self.cols and 0 <= c[1] < self.rows and 0 <= c[2] < self.layers

    def free(self, c):
        return self.inside(c) and not self.blocked[self.idx(*c)]

    def blocker(self, c):
        return self.block_src.get(self.idx(*c))

    def neighbors(self, c):
        x, y, z = c
        out = []
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy, z)
            if self.free(n):
                out.append(n)
        for dx, dy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
            n = (x + dx, y + dy, z)
            if self.free(n) and self.free((x + dx, y, z)) and self.free((x, y + dy, z)):
                out.append(n)
        for dz in (1, -1):
            n = (x, y, z + dz)
            if self.free(n):
                out.append(n)
        return out

    def move_cost(self, a, b):
        if a[2] != b[2]:
            base, wind = self.vcost, 0.0
        else:
            dx, dy = b[0] - a[0], b[1] - a[1]
            base = DIAG if (dx and dy) else 1.0
            wind = self.wind_penalty if (dx * self.wind[0] + dy * self.wind[1]) < 0 else 0.0
            if b[2] == 0:
                wind += self.surface
        return base + wind + self.soft[self.idx(*b)]

    def heuristic(self, a, b):
        dx, dy = abs(a[0] - b[0]), abs(a[1] - b[1])
        return (dx + dy) + (DIAG - 2.0) * min(dx, dy) + self.vcost * abs(a[2] - b[2])

    def path_cost(self, path):
        return sum(self.move_cost(path[i], path[i + 1]) for i in range(len(path) - 1))


class PlanResult:
    __slots__ = ("path", "cost", "expansions", "ms", "reason")

    def __init__(self, path, cost, expansions, ms, reason):
        self.path, self.cost, self.expansions, self.ms, self.reason = path, cost, expansions, ms, reason

    @property
    def ok(self):
        return self.path is not None

    def as_dict(self):
        return {"ok": self.ok, "cost": None if self.cost is None else round(self.cost, 3),
                "expansions": self.expansions, "ms": round(self.ms, 3), "reason": self.reason,
                "length": 0 if self.path is None else len(self.path)}


def astar(grid, start, goal, budget=None, informed=True):
    """A* search. informed=False drops the heuristic, which is Dijkstra's
    algorithm: same optimal cost, more nodes expanded."""
    start, goal = tuple(start), tuple(goal)
    t0 = time.perf_counter()
    hfun = grid.heuristic if informed else (lambda a, b: 0.0)
    if not grid.inside(goal) or not grid.free(goal):
        return PlanResult(None, None, 0, (time.perf_counter() - t0) * 1e3, "GOAL_BLOCKED")
    if not grid.inside(start):
        return PlanResult(None, None, 0, (time.perf_counter() - t0) * 1e3, "START_OUTSIDE")
    budget = budget or 10 ** 9
    open_set = [(hfun(start, goal), 0, start)]
    came, g = {}, {start: 0.0}
    closed = set()
    counter, expansions = 1, 0
    while open_set:
        _, _, cur = heapq.heappop(open_set)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return PlanResult(path, g[goal], expansions, (time.perf_counter() - t0) * 1e3, "OK")
        closed.add(cur)
        expansions += 1
        if expansions > budget:
            return PlanResult(None, None, expansions, (time.perf_counter() - t0) * 1e3, "BUDGET")
        gc = g[cur]
        for nb in grid.neighbors(cur):
            if nb in closed:
                continue
            t = gc + grid.move_cost(cur, nb)
            if t < g.get(nb, math.inf):
                came[nb] = cur
                g[nb] = t
                heapq.heappush(open_set, (t + hfun(nb, goal), counter, nb))
                counter += 1
    return PlanResult(None, None, expansions, (time.perf_counter() - t0) * 1e3, "NO_ROUTE")


def dijkstra(grid, start, goal, budget=None):
    return astar(grid, start, goal, budget, informed=False)


def plan_through(grid, start, targets, budget=None, informed=True):
    """Plan start -> targets[0] -> targets[1] ... Returns PlanResult and leg start indices."""
    t0 = time.perf_counter()
    path = [tuple(start)]
    cost, exp = 0.0, 0
    legs = []
    for i, tgt in enumerate(targets):
        legs.append(len(path) - 1)
        r = astar(grid, path[-1], tgt, budget, informed=informed)
        exp += r.expansions
        if not r.ok:
            return PlanResult(None, None, exp, (time.perf_counter() - t0) * 1e3,
                              f"{r.reason}_LEG{i + 1}"), legs
        path.extend(r.path[1:])
        cost += r.cost
    return PlanResult(path, cost, exp, (time.perf_counter() - t0) * 1e3, "OK"), legs


class CostField:
    """Reverse Dijkstra from one cell (normally home): the cheapest cost from
    EVERY free cell to that cell, plus the next hop. One search answers
    'can I still get home?' anywhere on the map in O(1), and gives the route
    home without any further search."""

    def __init__(self, grid, target, budget=None):
        t0 = time.perf_counter()
        self.grid, self.target = grid, tuple(target)
        self.dist, self.nxt = {}, {}
        self.expansions = 0
        if grid.free(self.target):
            self.dist[self.target] = 0.0
            heap, n = [(0.0, 0, self.target)], 1
            limit = budget or 10 ** 9
            while heap:
                d, _, v = heapq.heappop(heap)
                if d > self.dist.get(v, math.inf):
                    continue
                self.expansions += 1
                if self.expansions > limit:
                    break
                for u in grid.neighbors(v):          # neighbours are symmetric for free cells
                    nd = d + grid.move_cost(u, v)    # cost of flying u -> v
                    if nd < self.dist.get(u, math.inf):
                        self.dist[u] = nd
                        self.nxt[u] = v
                        heapq.heappush(heap, (nd, n, u))
                        n += 1
        self.ms = (time.perf_counter() - t0) * 1e3

    def cost(self, cell):
        return self.dist.get(tuple(cell))

    def path(self, cell):
        cell = tuple(cell)
        if cell not in self.dist:
            return None
        out = [cell]
        while out[-1] != self.target:
            out.append(self.nxt[out[-1]])
        return out


# 8 horizontal bearings in grid steps (+y is south), clockwise from north
BEARINGS = [(0, -1), (1, -1), (1, 0), (1, 1), (0, 1), (-1, 1), (-1, 0), (-1, -1)]


def _angle(a, b):
    d = abs(math.atan2(a[1], a[0]) - math.atan2(b[1], b[0]))
    return min(d, 2 * math.pi - d)


def _ruler(grid, start, step, length):
    """Cells along one ruler, or None if any cell (or a diagonal corner) is blocked."""
    cells, c = [], tuple(start)
    dx, dy, dz = step
    for _ in range(length):
        n = (c[0] + dx, c[1] + dy, c[2] + dz)
        if not grid.free(n):
            return None
        if dx and dy and not (grid.free((c[0] + dx, c[1], c[2])) and grid.free((c[0], c[1] + dy, c[2]))):
            return None
        cells.append(n)
        c = n
    return cells


def bendy_ruler(grid, cell, target, length=3, margin=1):
    """BendyRuler-style local avoidance (after ArduPilot's OA BendyRuler).

    Probes a fixed-length 'ruler' along each of 8 bearings plus straight up
    and down. A ruler is usable when every cell on it is free and, at its end,
    a second ruler continues roughly toward the target (so it does not lead
    into a pocket). Among usable rulers it picks the one whose bearing is
    closest to the bearing of the target, breaking ties by remaining distance.
    It does not guarantee a shortest path: the global planner takes over from
    the ruler's end. Returns (cells, info) or (None, info)."""
    t0 = time.perf_counter()
    cell, target = tuple(cell), tuple(target)
    want = (target[0] - cell[0], target[1] - cell[1])
    if want == (0, 0):
        want = (1, 0)
    cands, probes = [], 0
    for b in BEARINGS:
        probes += 1
        r = _ruler(grid, cell, (b[0], b[1], 0), length)
        if not r:
            continue
        end = r[-1]
        onward = (target[0] - end[0], target[1] - end[1])
        ok2 = any(_ruler(grid, end, (b2[0], b2[1], 0), max(1, margin + 1))
                  for b2 in BEARINGS if onward == (0, 0) or _angle(b2, onward) <= math.pi / 2 + 1e-9)
        if not ok2:
            continue
        rem = math.hypot(*onward) + abs(target[2] - end[2])
        cands.append((_angle(b, want), rem, r, f"bearing {BEARING_NAMES[BEARINGS.index(b)]}"))
    for dz, name in ((1, "climb"), (-1, "descend")):
        probes += 1
        r = _ruler(grid, cell, (0, 0, dz), min(2, length))
        if r:
            # moving vertically keeps the horizontal bearing: count it as a small deviation
            rem = math.hypot(*want) + abs(target[2] - r[-1][2])
            cands.append((math.pi / 4 + 0.05, rem, r, name))
    ms = (time.perf_counter() - t0) * 1e3
    if not cands:
        return None, {"ms": ms, "probes": probes, "choice": None}
    cands.sort(key=lambda c: (round(c[0], 6), c[1]))
    best = cands[0]
    return best[2], {"ms": ms, "probes": probes, "choice": best[3]}


BEARING_NAMES = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]


def escape_route(grid, start, no_climb=True, max_nodes=4000):
    """Shortest way out of blocked airspace: Dijkstra through blocked cells to the
    nearest free cell. Climbing is not allowed (the standard advice for an ash
    encounter is a descending turn, never a climb out). None if trapped."""
    start = tuple(start)
    if grid.free(start):
        return [start]
    dist, came, heap, n = {start: 0.0}, {}, [(0.0, 0, start)], 1
    while heap and n < max_nodes:
        d, _, c = heapq.heappop(heap)
        if d > dist[c]:
            continue
        if grid.free(c):
            path = [c]
            while path[-1] != start:
                path.append(came[path[-1]])
            return path[::-1]
        x, y, z = c
        moves = [(dx, dy, 0) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if dx or dy] + [(0, 0, -1)] + ([] if no_climb else [(0, 0, 1)])
        for dx, dy, dz in moves:
            nb = (x + dx, y + dy, z + dz)
            if not grid.inside(nb):
                continue
            nd = d + (DIAG if dx and dy else 1.0)
            if nd < dist.get(nb, math.inf):
                dist[nb], came[nb] = nd, c
                heapq.heappush(heap, (nd, n, nb))
                n += 1
    return None


def nearest_free(grid, target, min_d=1.5, max_d=6.0):
    """A holding fix: the closest free cell to `target` that is at least min_d away
    (so the drone does not orbit on top of the blocked landing site)."""
    tx, ty, tz = target
    best = None
    for dz in (0, -1, 1, -2, 2):
        z = tz + dz
        for y in range(int(ty - max_d), int(ty + max_d) + 1):
            for x in range(int(tx - max_d), int(tx + max_d) + 1):
                d = math.hypot(x - tx, y - ty)
                if d < min_d or d > max_d or not grid.free((x, y, z)):
                    continue
                key = (d + abs(dz) * 0.6, x, y, z)
                if best is None or key < best[0]:
                    best = (key, (x, y, z))
    return None if best is None else best[1]


def first_blocked(grid, route, start_index=0, window=None):
    """Index of the first blocked cell on route[start_index:...], or None."""
    end = len(route) if window is None else min(len(route), start_index + 1 + window)
    for i in range(start_index, end):
        if not grid.free(tuple(route[i])):
            return i
    return None


def route_problems(grid, route, profile, layers):
    """Validation used by the drone before accepting an uploaded route.
    Returns a list of (index, code) problems; empty means acceptable."""
    probs = []
    if not route:
        return [(0, "EMPTY_ROUTE")]
    for i, c in enumerate(route):
        c = tuple(c)
        if not grid.inside(c):
            probs.append((i, "OUTSIDE_AREA"))
            break
        if c[2] > int(profile["ceiling"]):
            probs.append((i, "ABOVE_CEILING"))
            break
        if not grid.free(c):
            probs.append((i, "INSIDE_HAZARD"))
            break
        if i:
            p = tuple(route[i - 1])
            d = (abs(c[0] - p[0]), abs(c[1] - p[1]), abs(c[2] - p[2]))
            if max(d) > 1 or (d[2] and (d[0] or d[1])):
                probs.append((i, "DISCONTINUOUS"))
                break
            if d[0] and d[1] and not (grid.free((c[0], p[1], p[2])) and grid.free((p[0], c[1], p[2]))):
                probs.append((i, "CORNER_CUT"))
                break
    return probs


def step_length(a, b):
    """Horizontal distance of one step in cells; a layer change counts as 1 (main.py)."""
    if a[2] != b[2]:
        return 1.0
    return DIAG if (a[0] != b[0] and a[1] != b[1]) else 1.0
