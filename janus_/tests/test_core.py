"""Geometry, planning and mission-building checks (no flight controller needed)."""
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest

from janus_fc import mission as MS
from janus_fc import planner as P
from janus_fc.config import make_profile
from janus_fc.geo import LocalFrame, distance_m
from janus_fc.zones import Zone, ZoneStore, blocks

F = LocalFrame(47.397742, 8.545594)
EX = os.path.join(os.path.dirname(__file__), "..", "examples")


def circ(zid, n, e, r, typ="tfr", **kw):
    lat, lon = F.to_ll(n, e)
    return Zone({"id": zid, "type": typ, "shape": "circle", "lat": lat, "lon": lon, "radius_m": r, **kw}, F)


def test_local_frame_round_trip_and_scale():
    lat, lon = F.to_ll(500, -300)
    n, e = F.to_ne(lat, lon)
    assert abs(n - 500) < 1e-6 and abs(e + 300) < 1e-6
    assert abs(distance_m(F.lat0, F.lon0, lat, lon) - math.hypot(500, 300)) < 1.0


@pytest.mark.parametrize("cls", ["copter", "plane"])
def test_route_keeps_the_margin_from_every_blocking_zone(cls):
    p = make_profile(cls)
    s = 1 if cls == "copter" else 3
    zones = [circ("T1", 300 * s, 300 * s, 120 * s), circ("T2", 150 * s, 550 * s, 80 * s)]
    goal = (650 * s, 650 * s, p["alt_layers_m"][1])
    res, grid = P.plan(zones, p, (0, 0, p["takeoff_alt_m"]), goal)
    assert res.ok, res.reason
    pts = res.path
    assert pts[-1] == goal and len(pts) <= 6, "smoothed to a few waypoints"
    for a, b in zip(pts, pts[1:]):
        for k in range(101):
            f = k / 100
            n, e = a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])
            for z in zones:
                assert z.distance(n, e) >= p["margin_m"] - 1e-6


def test_altitude_band_lets_the_route_pass_over():
    p = make_profile("copter")
    low = circ("LOW", 300, 300, 200, alt_min_m=0, alt_max_m=35)        # tops out at 35 m (+10 m margin)
    res, _ = P.plan([low], p, (0, 0, 30), (600, 600, 60))
    assert res.ok and all(pt[2] >= 45 for pt in res.path[1:-1] if math.hypot(pt[0] - 300, pt[1] - 300) < 230)


def test_drifting_zone_is_avoided_where_it_is_going():
    p = make_profile("copter")
    z = circ("A1", 300, 100, 60, typ="ash", conc_mg=6, vn=0, ve=5)       # moving east 5 m/s
    t0 = z.t_ref
    still, _ = P.plan([circ("A0", 300, 100, 60, typ="ash", conc_mg=6)], p, (0, 300, 30), (600, 300, 30), t=t0)
    moving, grid = P.plan([z], p, (0, 300, 30), (600, 300, 30), t=t0, horizon=40)
    assert still.ok and moving.ok
    assert len(still.path) == 2, "a still zone 200 m away does not touch the straight route"
    assert len(moving.path) > 2, "the drifting one will be there in 40 s: route bends"


def test_sar_rules():
    p, sar = make_profile("copter"), make_profile("copter", sar=True)
    tfr, low = circ("T", 0, 0, 50), circ("L", 0, 0, 50, typ="ash", conc_mg=1.0)
    assert blocks(tfr, p)[0] and not blocks(tfr, sar)[0]
    assert blocks(low, p)[0] and not blocks(low, sar)[0]
    assert blocks(circ("H", 0, 0, 50, typ="ash", conc_mg=5), sar)[0]


def test_plume_dense_at_vent_and_fades_downwind():
    lat, lon = F.to_ll(0, 0)
    z = Zone({"id": "V", "type": "ash", "shape": "plume", "lat": lat, "lon": lon, "conc_mg": 8, "r0_m": 40,
              "spread": 0.15, "decay_m": 800}, F, wind={"from_deg": 270, "speed_ms": 5})
    cs = z.circles
    assert cs[0][3] > cs[-1][3] >= 0.2 and cs[0][2] < cs[-1][2]
    assert cs[-1][1] > cs[0][1] and abs(cs[-1][0]) < 1.0, "carried east by a westerly"
    assert z.conc_at(0, 0, 50) == pytest.approx(8.0)
    assert z.conc_at(0, -300, 50, edge_m=30) < 0.01      # upwind of the vent: clean


@pytest.mark.parametrize("kind", ["racetrack", "racetrack_left", "figure8", "orbit"])
def test_hold_patterns_are_closed_and_sized(kind):
    pts = P.hold_pattern(kind, (0, 0), 90.0, 50.0)
    assert len(pts) >= 6
    closed = pts + [pts[0]]
    assert max(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(closed, closed[1:])) <= 2.5 * 50 + 1   # longest step is the straight leg
    assert max(math.hypot(*p) for p in pts) < 50 * 2 + 2.5 * 50 + 1


def test_standard_racetrack_turns_right():
    pts = P.hold_pattern("racetrack", (0, 0), 90.0, 50.0)       # inbound heading east
    assert min(p[0] for p in pts) < -50 and max(p[0] for p in pts) <= 1.0, "right of east is south (-n)"


def test_blocked_goal_gets_a_clear_hold():
    p = make_profile("copter")
    goal = (500, 500, 45)
    ash = circ("A", 500, 500, 80, typ="ash", conc_mg=6)
    res, grid = P.plan([ash], p, (0, 0, 30), goal)
    assert not res.ok and res.reason.startswith("goal blocked")
    h = P.choose_hold(grid, goal, 45.0, p)
    assert h and P.pattern_clear(grid, h["pts"], h["fix"][2]) and h["kind"] == "racetrack"


@pytest.mark.parametrize("stack", ["ardupilot", "px4"])
def test_mission_items(stack):
    p = make_profile("copter")
    route = [(0, 0, 30), (200, 100, 45), (400, 400, 45)]
    items, _ = MS.build(F, p, stack, route, takeoff=True)
    cmds = [it["cmd"] for it in items]
    assert cmds == [22, 178, 16, 16, 21]                     # TAKEOFF, SPEED, WP, WP, LAND
    assert all(it["frame"] == 6 for it in items if it["cmd"] != 178)   # nav items: relative altitude
    hold = P.choose_hold(P.Grid([], p, [(0, 0), (400, 400)]), (400, 400, 45), 45.0, p)
    items, _ = MS.build(F, p, stack, route, hold=hold)
    jump = items[-1]
    assert jump["cmd"] == 177 and items[int(jump["p1"])]["tag"] == "hold0"
    off = 1 if stack == "ardupilot" else 0
    assert MS.fix_jumps(items, off)[-1]["p1"] == jump["p1"] + off


def test_plane_steep_leg_gets_loiter_to_alt():
    p = make_profile("plane")
    route = [(0, 0, 80), (400, 0, 300)]                        # 220 m in 400 m: far above 0.15
    grid = P.Grid([], p, [(0, 0), (400, 0)])
    items, notes = MS.build(F, p, "px4", route, grid=grid)
    assert items[0]["cmd"] == 178 and items[1]["cmd"] == 31 and items[1]["alt"] == 300 and not notes


def test_example_zone_files_load_and_plan():
    for name in os.listdir(EX):
        if not name.startswith("zones_"):
            continue
        stack = name.split("_")[1]
        cls = name.split("_")[2].split(".")[0]
        home = (47.397742, 8.545594) if stack == "px4" else (-35.363262, 149.165237)
        f = LocalFrame(*home)
        st = ZoneStore(os.path.join(EX, name), f)
        assert any(not z.reported for z in st.file_zones)
        p = make_profile(cls)
        goal = (650, 650, 40) if cls == "copter" else (2200, 2200, 120)
        res, _ = P.plan(st.known(), p, (0, 0, p["takeoff_alt_m"]), goal)
        assert res.ok, (name, res.reason)
        hidden = [z for z in st.file_zones if not z.reported][0]
        # the unreported plume sits on the planned route: only the onboard sensor can find it
        near = min(hidden.distance(a[0] + k / 50 * (b[0] - a[0]), a[1] + k / 50 * (b[1] - a[1]))
                   for a, b in zip(res.path, res.path[1:]) for k in range(51))
        assert near < 0, (name, near)


def test_broken_zone_file_keeps_previous_zones(tmp_path):
    path = tmp_path / "z.json"
    lat, lon = F.to_ll(100, 100)
    path.write_text('{"zones": [{"id": "T", "type": "tfr", "lat": %f, "lon": %f, "radius_m": 50}]}' % (lat, lon))
    st = ZoneStore(str(path), F)
    import time as _t
    _t.sleep(0.02)
    path.write_text("{ broken")
    os.utime(path, (_t.time() + 5, _t.time() + 5))
    assert not st.reload() and len(st.file_zones) == 1


def test_inside_a_margin_the_route_only_leads_out():
    """Just found ash at its edge: the start is inside the margin. The planner
    must find a way out that never gets closer than we already are."""
    p = make_profile("copter")
    ash = circ("EST1", 60, 0, 50, typ="ash", conc_mg=3)          # edge 10 m ahead of us, margin 20 m
    res, grid = P.plan([ash], p, (0, 0, 45), (400, 0, 45))
    assert res.ok, res.reason
    d0 = ash.distance(0, 0)
    a, b = res.path[0], res.path[1]
    for k in range(51):
        f = k / 50
        assert ash.distance(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])) >= d0 - 1.0 - 1e-6
    rest = res.path[1:]
    for a, b in zip(rest, rest[1:]):
        for k in range(51):
            f = k / 50
            assert ash.distance(a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1])) >= p["margin_m"] - 1e-6


def test_fixed_wing_landing_meets_px4_glide_slope_and_is_clear():
    p = make_profile("plane")
    tfr = circ("T", 450, 0, 200)                               # north of home: on the final approach for a southbound landing
    grid = P.Grid([tfr], p, [(0, 0), (2000, 2000)])
    land = P.choose_landing(grid, (0.0, 0.0), 180.0, p, 80)     # wind from the south: land heading south
    assert land is not None
    C, r = land["centre"], land["radius"]
    d = math.hypot(C[0], C[1])
    tangent = math.sqrt(d * d - r * r)                          # PX4 FeasibilityChecker approach length
    assert d > r and land["alt"] / tangent <= math.tan(math.radians(5.1))
    assert land["heading"] != 180.0, "the into-wind approach crosses the TFR: another heading is chosen"
    items, _ = MS.build(F, p, "px4", [(1000, 1000, 80), (C[0], C[1], 80)], grid=grid, landing=land)
    assert [it["cmd"] for it in items][-3:] == [189, 31, 21] and items[-3]["frame"] == 2
    items, _ = MS.build(F, p, "ardupilot", [(1000, 1000, 80), (C[0], C[1], 80)], grid=grid, landing=land)
    assert [it["cmd"] for it in items][-2:] == [31, 21] and 189 not in [it["cmd"] for it in items]
