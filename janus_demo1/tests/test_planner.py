"""Planner tests.

Parity: main.py's own grid_astar (loaded headless, cut before its game loop)
and janus.planner must find routes of equal optimal cost on the same scenario.
main.py blocks every turbulence cell, so parity runs with turb_block=1.
"""
import math
import os
import random
import sys

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from janus import planner as P                       # noqa: E402
from janus.config import make_profile, COLS, ROWS, LAYERS  # noqa: E402

_CANDIDATES = [os.environ.get("JANUS_MAIN_PY", ""),
               os.path.join(HERE, "..", "..", "main.py"),           # janus_demo/ placed inside the Janus repo
               os.path.join(HERE, "..", "..", "janus", "main.py")]  # repo cloned next to janus_demo/
MAIN_PY = next((c for c in _CANDIDATES if c and os.path.exists(c)), _CANDIDATES[1])


os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")


def load_main():
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    src = open(MAIN_PY, encoding="utf-8").read()
    cut = src.index("\nrunning = True\n")
    ns = {"__name__": "main_headless"}
    exec(compile(src[:cut], "main.py", "exec"), ns)
    return ns


def random_world(rng):
    obstacles, hazards = [], []
    for i in range(rng.randint(2, 6)):
        w, h = rng.choice([(1, 2), (2, 2), (2, 3), (3, 2), (1, 5)])
        cs, rs = rng.randint(2, COLS - w - 2), rng.randint(0, ROWS - h)
        z0, z1 = (0, LAYERS - 1) if rng.random() < 0.6 else sorted(rng.sample(range(LAYERS), 2))
        obstacles.append((i + 1, cs, w, rs, h, z0, z1))
    for i in range(rng.randint(2, 7)):
        w, h = rng.choice([(2, 2), (3, 2), (3, 3), (4, 2)])
        cs, rs = rng.randint(1, COLS - w - 1), rng.randint(0, ROWS - h)
        z0 = rng.randint(0, LAYERS - 2)
        z1 = min(LAYERS - 1, z0 + rng.randint(1, 4))
        htype = rng.choice(["ash", "turbulence"])
        sev = rng.randint(1, 3)
        hazards.append((i + 1, htype, cs, w, rs, h, z0, z1, sev))
    return obstacles, hazards


def to_janus(obstacles, hazards):
    out = []
    for (i, cs, w, rs, h, z0, z1) in obstacles:
        out.append({"id": f"o{i}", "type": "building", "x0": cs, "x1": cs + w - 1,
                    "y0": rs, "y1": rs + h - 1, "z0": z0, "z1": z1})
    for (i, t, cs, w, rs, h, z0, z1, sev) in hazards:
        out.append({"id": f"h{i}", "type": "ash" if t == "ash" else "turb", "sev": sev,
                    "x0": cs, "x1": cs + w - 1, "y0": rs, "y1": rs + h - 1, "z0": z0, "z1": z1})
    return out


@pytest.fixture(scope="module")
def main_ns():
    if not os.path.exists(MAIN_PY):
        pytest.skip("main.py not found; set JANUS_MAIN_PY")
    return load_main()


def parity_profile():
    return make_profile("aircraft", None, turb_block=1, climb_mult=1.0, ceiling=8, wind_penalty=3.0)


def test_parity_with_main_py(main_ns):
    rng = random.Random(20261031)
    prof = parity_profile()
    checked = both_none = 0
    for trial in range(200):
        obstacles, hazards = random_world(rng)
        main_ns["obstacles"][:] = [main_ns["make_obstacle"](*o) for o in obstacles]
        main_ns["hazards"][:] = [main_ns["make_hazard"](h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7],
                                                        h[8] if h[1] == "ash" else ["LOW", "MODERATE", "HIGH"][h[8] - 1])
                                 for h in hazards]
        start = (rng.randint(0, 2), rng.randint(0, ROWS - 1), rng.randint(0, 3))
        goal = (rng.randint(COLS - 3, COLS - 1), rng.randint(0, ROWS - 1), rng.randint(0, 3))
        m_path = main_ns["grid_astar"](start, goal)
        grid = P.Grid(COLS, ROWS, LAYERS, to_janus(obstacles, hazards), prof, wind=(1, 0),
                          ash_buffer=False, surface_cost=0.0)   # buffer ring and surface cost are extensions main.py lacks
        if not grid.free(start):
            continue                                   # main.py has no start check
        j = P.astar(grid, start, goal)
        if m_path is None:
            assert not j.ok, f"trial {trial}: main.py found no route but janus did"
            both_none += 1
            continue
        assert j.ok, f"trial {trial}: janus found no route but main.py did ({j.reason})"
        m_cost = sum(main_ns["grid_move_cost"](m_path[i], m_path[i + 1]) for i in range(len(m_path) - 1))
        assert math.isclose(j.cost, m_cost, abs_tol=1e-9), f"trial {trial}: cost {j.cost} vs main.py {m_cost}"
        # janus path must also be a legal main.py path with the same cost under main.py's model
        jm_cost = sum(main_ns["grid_move_cost"](j.path[i], j.path[i + 1]) for i in range(len(j.path) - 1))
        assert math.isclose(jm_cost, m_cost, abs_tol=1e-9)
        assert all(main_ns["grid_cell_free"](c) for c in j.path)
        checked += 1
    assert checked >= 120, f"only {checked} comparable trials"
    print(f"parity: {checked} equal-cost routes, {both_none} agreed no-route")


def test_low_turbulence_passes_with_readme_default():
    prof = make_profile("aircraft", None, turb_block=2, ceiling=8)
    low = {"id": "t", "type": "turb", "sev": 1, "x0": 5, "x1": 19, "y0": 0, "y1": ROWS - 1, "z0": 0, "z1": 8}
    grid = P.Grid(COLS, ROWS, LAYERS, [low], prof)
    r = P.astar(grid, (0, 8, 2), (24, 8, 2))
    assert r.ok
    assert any(grid.soft[grid.idx(*c)] > 0 for c in r.path), "LOW turbulence should be crossed at a cost"
    blocking = make_profile("aircraft", None, turb_block=1, ceiling=8)
    assert not P.astar(P.Grid(COLS, ROWS, LAYERS, [low], blocking), (0, 8, 2), (24, 8, 2)).ok


def test_ash_ceiling_rule_blocks_full_column():
    prof = make_profile("aircraft", None, ceiling=8)
    ash = {"id": "a", "type": "ash", "sev": 2, "x0": 10, "x1": 12, "y0": 0, "y1": ROWS - 1, "z0": 5, "z1": 8}
    grid = P.Grid(COLS, ROWS, LAYERS, [ash], prof)
    assert not grid.free((11, 3, 0)), "ash reaching the top layer must block down to the ground"
    assert not P.astar(grid, (0, 8, 0), (24, 8, 0)).ok


def test_profile_ceiling_and_gust_limit():
    lo = make_profile("small", "Scout-Q  light quadcopter")        # gust limit 18 kt
    hi = make_profile("small", "Swift-V  VTOL fixed-wing")         # gust limit 32 kt
    gust = {"id": "g", "type": "gust", "gust_kt": 24, "x0": 8, "x1": 14, "y0": 0, "y1": ROWS - 1, "z0": 0, "z1": 8}
    assert not P.astar(P.Grid(COLS, ROWS, LAYERS, [gust], lo), (0, 8, 2), (24, 8, 2)).ok
    assert P.astar(P.Grid(COLS, ROWS, LAYERS, [gust], hi), (0, 8, 2), (24, 8, 2)).ok
    hauler = make_profile("small", "Hauler-H  heavy-lift hexacopter")  # ceiling layer 6
    g = P.Grid(COLS, ROWS, LAYERS, [], hauler)
    assert not g.free((3, 3, 7)) and g.free((3, 3, 6))


def test_invariants_random():
    rng = random.Random(7)
    prof = make_profile("small", "Scout-Q  light quadcopter")
    for _ in range(150):
        obstacles, hazards = random_world(rng)
        hz = to_janus(obstacles, hazards)
        grid = P.Grid(COLS, ROWS, LAYERS, hz, prof, wind=P.wind_vector(rng.uniform(0, 360)))
        s = (0, rng.randint(0, ROWS - 1), 1)
        g = (COLS - 1, rng.randint(0, ROWS - 1), 1)
        if not grid.free(s):
            continue
        r = P.astar(grid, s, g)
        if not r.ok:
            continue
        assert P.route_problems(grid, r.path, prof, LAYERS) == []
        assert math.isclose(grid.path_cost(r.path), r.cost, abs_tol=1e-9)
        assert r.cost + 1e-9 >= grid.heuristic(s, g)


def test_budget_trips():
    prof = make_profile("small", None, budget=50)
    grid = P.Grid(COLS, ROWS, LAYERS, [], prof)
    r = P.astar(grid, (0, 0, 0), (24, 16, 8), budget=prof["budget"])
    assert not r.ok and r.reason == "BUDGET"


def test_route_validation_codes():
    prof = make_profile("small", "Hauler-H  heavy-lift hexacopter")
    grid = P.Grid(COLS, ROWS, LAYERS, [{"id": "b", "type": "building", "x0": 5, "x1": 5, "y0": 5, "y1": 5, "z0": 0, "z1": 8}], prof)
    assert P.route_problems(grid, [(4, 5, 1), (5, 5, 1)], prof, LAYERS)[0][1] == "INSIDE_HAZARD"
    assert P.route_problems(grid, [(4, 4, 6), (4, 4, 7)], prof, LAYERS)[0][1] == "ABOVE_CEILING"
    assert P.route_problems(grid, [(1, 1, 1), (3, 1, 1)], prof, LAYERS)[0][1] == "DISCONTINUOUS"
    assert P.route_problems(grid, [(4, 4, 1), (5, 3, 1)], prof, LAYERS) == []
    assert P.route_problems(grid, [(4, 5, 1), (5, 4, 1)], prof, LAYERS)[0][1] == "CORNER_CUT"  # clips (5,5)


def test_wind_vector_quantisation():
    assert P.wind_vector(270) == (1, 0)    # wind FROM west blows toward east (+x)
    assert P.wind_vector(0) == (0, 1)      # from north -> toward south (+y on screen)
    assert P.wind_vector(225) == (1, -1)   # from south-west -> toward north-east
