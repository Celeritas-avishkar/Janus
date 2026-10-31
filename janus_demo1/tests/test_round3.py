"""Round 3 checks: in-situ OPC/SO2 sensing, ash escape and wear, holding pattern,
drifting and circular zones, curved flight path, map editing."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest

from janus import config as C
from janus import planner as P
from janus.config import PRESETS, make_profile
from janus.drone_brain import DroneBrain
from janus.scenarios import build, _sim
from janus.world import ash_concentration, base_scenario, circle, drift_velocity

ALL = [(m, p) for m in PRESETS for p in PRESETS[m]]


# ---------------------------------------------------------------- sensing
def test_in_situ_sensor_cannot_see_ash_ahead():
    scn = base_scenario(1, "small", "calm", opts={"ash_plumes": 0, "cells": 0, "rain": 0})
    scn["hazards"] = [circle("A1", "ash", 12, 8, 1.5, 0, 8, sev=3, conc_mg=6.0)]
    assert ash_concentration(scn, [12, 8, 1], 0)[0] == pytest.approx(6.0)
    # three cells from the edge the air is clean: a point sensor reads (almost) nothing
    assert ash_concentration(scn, [12 + 1.5 + 3, 8, 1], 0)[0] < 0.01
    # the diffuse edge is what an OPC can pick up just before the core
    edge = ash_concentration(scn, [12 + 1.5 + 0.5, 8, 1], 0)[0]
    assert C.make_profile("small")["opc_thr_mg"] < edge < 6.0


def test_so2_lags_behind_opc():
    p = make_profile("small", "Scout-Q  light quadcopter")
    scn = base_scenario(1, "small", "calm", opts={"ash_plumes": 0, "cells": 0, "rain": 0})
    scn["hazards"] = [circle("A1", "ash", 5, 5, 3, 0, 8, sev=3, conc_mg=6.0)]
    d = DroneBrain(scn, p)
    d.pos, d.state = [5.0, 5.0, 1.0], "ENROUTE"
    d.t = 0.0
    d._sample(0.05)
    d.t = 0.2
    d._sample(0.05)
    assert d.opc > 4.0, "OPC responds within one sample"
    assert d.so2 < 0.2 * 6.0 * C.SO2_PPM_PER_MG, "the SO2 cell is still far from its final value"


# ---------------------------------------------------------------- escape and wear
@pytest.mark.parametrize("mode,preset", ALL)
def test_escape_from_unreported_ash(mode, preset):
    p = make_profile(mode, preset)
    scn = build("escape", 1, mode, p)
    s = _sim(scn, p, seed=4)
    d = s.drone
    codes = [e["code"] for e in d.events]
    assert "ASH_ENCOUNTER" in codes and "ESCAPED" in codes
    assert codes.index("ASH_ENCOUNTER") < codes.index("ESCAPED")
    assert d.state in ("COMPLETE", "LANDED")
    assert d.health < 1.0 and d.dose > 0, "ash wear must be modelled"
    assert d.m["blocked_entries"] == 0
    assert scn["story"]["popup"] in s.gcs.reported, "the drone reports the plume back"


@pytest.mark.parametrize("mode,preset", ALL)
def test_escape_never_climbs(mode, preset, monkeypatch):
    legs = []
    orig = DroneBrain._escape

    def spy(self, hid):
        orig(self, hid)
        legs.append([tuple(c) for c in self.route[:self.escape_end + 1]])
    monkeypatch.setattr(DroneBrain, "_escape", spy)
    p = make_profile(mode, preset)
    scn = build("escape", 1, mode, p)
    _sim(scn, p, seed=4)
    assert legs, "an escape must have happened"
    for leg in legs:
        assert all(b[2] <= a[2] for a, b in zip(leg, leg[1:])), f"climbed while escaping: {leg}"


def test_heavy_wear_aborts_and_critical_wear_forces_landing():
    p = make_profile("small", "Scout-Q  light quadcopter")
    scn = base_scenario(1, "small", "calm", opts={"ash_plumes": 0, "cells": 0, "rain": 0})
    scn["hazards"] = [circle("A1", "ash", 5, 5, 4, 0, 8, sev=3, conc_mg=10.0)]
    d = DroneBrain(scn, p)
    d.pos, d.state, d.targets = [5.0, 5.0, 1.0], "ENROUTE", [(20, 5, 1)]
    for k in range(200):
        d.t = k * 0.05
        d._sample(0.05)
        d._degrade(0.05)
        if d.state == "FORCED":
            break
    assert "ASH_DAMAGE" in d.flags and d.state == "FORCED" and d.health < 0.25


# ---------------------------------------------------------------- holding
@pytest.mark.parametrize("mode,preset", ALL)
def test_holding_pattern_until_landing_site_clears(mode, preset):
    p = make_profile(mode, preset)
    scn = build("holding", 1, mode, p)
    s = _sim(scn, p, seed=4)
    codes = [e["code"] for e in s.drone.events]
    assert "HOLDING" in codes and "HOLD_CLEAR" in codes
    assert codes.index("HOLDING") < codes.index("HOLD_CLEAR")
    assert s.drone.state == "COMPLETE" and s.drone.m["exposures"] == 0


def test_hold_fix_is_clear_and_off_the_site():
    p = make_profile("small", "Scout-Q  light quadcopter")
    scn = base_scenario(2, "small", "calm", opts={"ash_plumes": 0, "cells": 0, "rain": 0})
    goal = tuple(scn["targets"][-1])
    hz = circle("A1", "ash", goal[0], goal[1], 1.3, 0, 8, sev=2, conc_mg=3.0)
    g = P.Grid(scn["cols"], scn["rows"], scn["layers"], scn["static"] + [hz], p)
    assert not g.free(goal)
    fix = P.nearest_free(g, goal)
    assert fix is not None and g.free(fix) and math.hypot(fix[0] - goal[0], fix[1] - goal[1]) >= 1.5


# ---------------------------------------------------------------- moving and circular zones
def test_drift_follows_the_wind():
    vx, vy = drift_velocity("small", {"dir": 270.0, "kt": 20.0}, 1.0)    # wind FROM the west
    assert vx > 0 and abs(vy) < 1e-9
    h = circle("A1", "ash", 5, 5, 1.2, 0, 8, vx=vx, vy=vy, t_ref=0.0)
    assert P.zone_contains_xy(h, 5, 5, 0) and not P.zone_contains_xy(h, 5, 5, 10.0)
    assert P.zone_contains_xy(h, round(5 + vx * 10.0), 5, 10.0)


def test_swept_volume_blocks_where_a_zone_is_going():
    p = make_profile("small", "Scout-Q  light quadcopter")
    h = circle("A1", "ash", 5, 8, 1.2, 0, 8, sev=3, vx=1.0, vy=0.0, t_ref=0.0)
    now = P.Grid(25, 17, 9, [h], p, t=0.0)
    swept = P.Grid(25, 17, 9, [h], p, t=0.0, horizon=4.0)
    assert now.free((8, 8, 1)) and not swept.free((8, 8, 1))


def test_circle_zone_is_round():
    h = circle("A1", "ash", 10, 8, 2.2, 0, 8)
    cells = P.zone_cells(h, 25, 17)
    assert (10, 8) in cells and (12, 8) in cells and (12, 10) not in cells


@pytest.mark.parametrize("mode,preset", ALL)
def test_drifting_weather_nominal_mission_is_safe(mode, preset):
    p = make_profile(mode, preset)
    scn = build("nominal", 3, mode, p, opts={"drift": 1.0})
    assert any(h.get("vx") or h.get("vy") for h in scn["hazards"])
    s = _sim(scn, p, seed=6)
    assert s.drone.state == "COMPLETE" and s.drone.m["exposures"] == 0 and s.drone.m["blocked_entries"] == 0


# ---------------------------------------------------------------- curved flight path
def test_curved_path_stays_inside_corner_cells():
    p = make_profile("small", "Scout-Q  light quadcopter")
    scn = base_scenario(1, "small", "calm")
    d = DroneBrain(scn, p)
    d.route = [(0, 0, 1), (1, 0, 1), (2, 0, 1), (2, 1, 1), (2, 2, 1), (3, 3, 1)]
    for i in range(len(d.route) - 1):
        for k in range(11):
            pos = d._curve_pos(i, k / 10)
            near = min(d.route, key=lambda c: math.hypot(c[0] - pos[0], c[1] - pos[1]))
            assert abs(pos[0] - near[0]) <= 0.5 + 1e-9 and abs(pos[1] - near[1]) <= 0.5 + 1e-9
    # and it really is curved: the corner at (2, 0) is cut
    mid = d._curve_pos(1, 0.75)
    assert mid[0] < 2.0 and mid[1] > 0.0


# ---------------------------------------------------------------- map editing (live runtime)
def test_editing_start_goal_and_zones():
    from janus.runtime import Runtime
    rt = Runtime(port_base=49900)
    try:
        assert rt.prepare({"mode": "small", "preset": "Swift-V  VTOL fixed-wing", "script": "nominal", "seed": 1})["ok"]
        g = rt.scn["targets"][-1]
        assert rt.command("edit", {"tool": "goal", "x": g[0], "y": max(2, g[1] - 2)})["ok"]
        g = rt.scn["targets"][-1]
        assert rt.command("edit", {"tool": "zone", "x": g[0], "y": g[1], "type": "ash", "sev": 2, "size": 1})["ok"]
        assert rt.gcs.hold is not None, "a blocked landing site is planned as a hold"
        home = rt.scn["home"]
        assert not rt.command("edit", {"tool": "zone", "x": home[0], "y": home[1], "type": "ash"})["ok"]
        u = [h for h in rt.scn["hazards"] if h["id"].startswith("U")][0]
        assert rt.command("edit", {"tool": "erase", "x": u["cx"], "y": u["cy"]})["ok"]
        assert rt.gcs.hold is None and rt.gcs.plan.ok
    finally:
        rt.shutdown()
