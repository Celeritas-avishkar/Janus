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


def hazard_blocks(h, profile):
    t = h["type"]
    if t in SOLID_TYPES or t == "ash":
        return True
    if t == "turb":
        return int(h.get("sev", 2)) >= int(profile["turb_block"])
    if t == "gust":
        return float(h.get("gust_kt", 0)) > float(profile["gust_limit_kt"])
    if t == "rain":
        return not profile["rain_rated"]
    return False


def hazard_soft_cost(h, profile):
    t = h["type"]
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

    def __init__(self, cols, rows, layers, hazards, profile, wind=(1, 0), wind_penalty=None):
        self.cols, self.rows, self.layers = cols, rows, layers
        self.profile = profile
        self.wind = wind
        self.wind_penalty = profile.get("wind_penalty", 3.0) if wind_penalty is None else wind_penalty
        self.vcost = VERTICAL_COST * float(profile.get("climb_mult", 1.0))
        n = cols * rows * layers
        self.blocked = bytearray(n)
        self.soft = [0.0] * n
        self.block_src = {}
        ceiling = int(profile.get("ceiling", layers - 1))
        for z in range(ceiling + 1, layers):
            for y in range(rows):
                for x in range(cols):
                    self.blocked[self.idx(x, y, z)] = 1
        for h in hazards:
            z0, z1 = hazard_zrange(h, layers)
            block = hazard_blocks(h, profile)
            cost = 0.0 if block else hazard_soft_cost(h, profile)
            for z in range(max(0, z0), min(layers - 1, z1) + 1):
                for y in range(max(0, h["y0"]), min(rows - 1, h["y1"]) + 1):
                    for x in range(max(0, h["x0"]), min(cols - 1, h["x1"]) + 1):
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


def astar(grid, start, goal, budget=None):
    start, goal = tuple(start), tuple(goal)
    t0 = time.perf_counter()
    if not grid.inside(goal) or not grid.free(goal):
        return PlanResult(None, None, 0, (time.perf_counter() - t0) * 1e3, "GOAL_BLOCKED")
    if not grid.inside(start):
        return PlanResult(None, None, 0, (time.perf_counter() - t0) * 1e3, "START_OUTSIDE")
    budget = budget or 10 ** 9
    open_set = [(grid.heuristic(start, goal), 0, start)]
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
                heapq.heappush(open_set, (t + grid.heuristic(nb, goal), counter, nb))
                counter += 1
    return PlanResult(None, None, expansions, (time.perf_counter() - t0) * 1e3, "NO_ROUTE")


def plan_through(grid, start, targets, budget=None):
    """Plan start -> targets[0] -> targets[1] ... Returns PlanResult and leg start indices."""
    t0 = time.perf_counter()
    path = [tuple(start)]
    cost, exp = 0.0, 0
    legs = []
    for i, tgt in enumerate(targets):
        legs.append(len(path) - 1)
        r = astar(grid, path[-1], tgt, budget)
        exp += r.expansions
        if not r.ok:
            return PlanResult(None, None, exp, (time.perf_counter() - t0) * 1e3,
                              f"{r.reason}_LEG{i + 1}"), legs
        path.extend(r.path[1:])
        cost += r.cost
    return PlanResult(path, cost, exp, (time.perf_counter() - t0) * 1e3, "OK"), legs


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
