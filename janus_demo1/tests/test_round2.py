"""Round 2 checks: RTL fixes, ash as primary hazard, SAR, Dijkstra / BendyRuler, reset, antenna."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest

from janus import config as C
from janus import planner as P
from janus.config import PRESETS, make_profile, apply_sar
from janus.scenarios import build, _sim
from janus.sim import Sim
from janus.world import WeatherFeed, base_scenario, grid_for, truth_hazards

ALL = [(m, p) for m in PRESETS for p in PRESETS[m]]


# ---------------------------------------------------------------- RTL
def test_rtl_pressed_during_replan_pause_lands_at_home():
    p = make_profile("small", "Scout-Q  light quadcopter", planner="astar")
    scn = build("popup", 1, "small", p)
    s = Sim(scn, p, deterministic=True)
    s.run(0.4)
    s.gcs.upload()
    s.run(120, stop=lambda x: x.drone.state == "REPLAN")
    assert s.drone.state == "REPLAN"
    s.gcs.command("RTL")
    s.run(s.t + 90, stop=lambda x: x.done())
    assert s.drone.state == "LANDED" and list(s.drone.cell) == list(scn["home"])


def test_rtl_after_mission_complete_flies_home():
    p = make_profile("small", "Swift-V  VTOL fixed-wing")      # enough battery to fly the whole way back
    scn = build("nominal", 1, "small", p)
    s = _sim(scn, p)
    assert s.drone.state == "COMPLETE"
    s.gcs.command("RTL")
    s.run(s.t + 120, stop=lambda x: x.drone.state == "LANDED")
    assert s.drone.state == "LANDED" and list(s.drone.cell) == list(scn["home"])


def test_operator_rtl_not_overridden_when_link_returns():
    p = make_profile("small", "Scout-Q  light quadcopter")       # policy RETURN_TO_LINK
    scn = build("antenna", 1, "small", p)
    st = scn["story"]
    s = Sim(scn, p, deterministic=True)
    s.run(0.4)
    s.gcs.upload()
    s.run(st["t_cut"])
    s.relay_alive = False
    s.run(st["t_cut"] + p["lost_link_s"] + 2)
    assert s.drone.failsafe == "LOST_LINK"
    s.gcs.command("RTL")                                    # queued; arrives once the link is back
    s.run(s.t + 2)
    s.relay_alive = True
    s.run(s.t + 120, stop=lambda x: x.done())
    assert s.drone.state == "LANDED"


def test_rtl_on_the_pad_is_ignored_and_reported():
    p = make_profile("small", "Scout-Q  light quadcopter")
    scn = build("nominal", 1, "small", p)
    s = Sim(scn, p, deterministic=True)
    s.run(1.0)
    s.gcs.command("RTL")
    s.run(3.0)
    assert s.drone.state == "IDLE"
    assert "RTL_IGNORED" in [e["code"] for e in s.gcs.events]


# ---------------------------------------------------------------- ash
@pytest.mark.parametrize("mode,preset", ALL)
def test_sensor_finds_unreported_ash_mid_route(mode, preset):
    for planner in ("hybrid", "astar"):
        p = make_profile(mode, preset, planner=planner)
        scn = build("sensor", 1, mode, p)
        hid = scn["story"]["popup"]
        s = _sim(scn, p, seed=4)
        d = s.drone
        assert d.state == "COMPLETE" and d.m["exposures"] == 0 and d.m["blocked_entries"] == 0
        assert hid in d.sensed, "only the drone's own sensor can find it"
        assert hid in s.gcs.reported, "the drone reports it back over the radio"
        if planner == "hybrid":
            assert d.m["bendy"] >= 1 or d.m["replans"] >= 1      # BendyRuler, or a full replan if no ruler is clear


def test_unreported_ash_never_reaches_the_feed():
    p = make_profile("aircraft", "Sentinel  MALE")
    scn = build("sensor", 1, "aircraft", p)
    hid = scn["story"]["popup"]
    feed = WeatherFeed(scn)
    for t in (0.0, 5.0, 50.0, 500.0):
        assert hid not in [h["id"] for h in feed.visible_hazards(t)]


def test_every_default_world_has_ash_in_both_modes():
    for mode in ("small", "aircraft"):
        for seed in range(20):
            scn = base_scenario(seed, mode, "breezy")
            plumes = [h for h in scn["hazards"] if h.get("shape") == "plume"]
            assert plumes, (mode, seed)
            for h in plumes:                      # Round 4: a vent and its plume, dense at the vent
                assert h["c0"] >= 2.0 and h["life"] > 0 and h["every"] > 0


def test_weather_options_are_respected():
    scn = base_scenario(5, "small", "calm", opts={"wind_dir": 90, "wind_kt": 25, "gust_kt": 40, "ash_plumes": 2,
                                                  "ash_conc": 2, "cells": 0, "rain": 0, "vis_km": 4})
    assert scn["wind"] == {"dir": 90.0, "kt": 25.0, "gust": 40.0}
    ash = [h for h in scn["hazards"] if h["type"] == "ash"]
    assert len(ash) == 2 and all(h["shape"] == "plume" and h["c0"] == 6.0 for h in ash)
    assert not [h for h in scn["hazards"] if h["type"] in ("gust", "rain")]
    snap = WeatherFeed(scn).snapshot(1.0)
    assert snap["visibility_km"] <= 4 and " VA" in snap["metar"] and len(snap["vaa"]) == 2


# ---------------------------------------------------------------- SAR
def test_sar_crosses_low_ash_only():
    base = make_profile("small", "Scout-Q  light quadcopter")
    sar = apply_sar(base)
    low = {"type": "ash", "sev": 1}
    for sev in (2, 3):
        assert P.hazard_blocks({"type": "ash", "sev": sev}, sar)
    assert P.hazard_blocks(low, base) and not P.hazard_blocks(low, sar)
    assert sar["gust_limit_kt"] == base["gust_limit_kt"] + 8 and sar["failsafe"] == "CONTINUE"
    assert sar["rain_rated"] and sar["lost_link_s"] == base["lost_link_s"] * 2


# ---------------------------------------------------------------- planners
def _grid(seed, mode="small"):
    p = make_profile(mode, list(PRESETS[mode])[0])
    scn = base_scenario(seed, mode, "stormy")
    return scn, p, grid_for(scn, scn["static"] + truth_hazards(scn, 0.0), p)


@pytest.mark.parametrize("seed", range(12))
def test_dijkstra_and_cost_field_agree_with_astar(seed):
    scn, p, g = _grid(seed, "small" if seed % 2 else "aircraft")
    goal = tuple(scn["targets"][-1])
    a = P.astar(g, scn["home"], goal)
    d = P.dijkstra(g, scn["home"], goal)
    assert a.ok == d.ok
    if a.ok:
        assert a.cost == pytest.approx(d.cost) and a.expansions <= d.expansions
    # cost-to-home field: cheapest goal -> home equals A* goal -> home
    if not g.free(goal):
        return                                     # goal inside a drifting plume's swept area at this instant
    f = P.CostField(g, scn["home"])
    back = P.astar(g, goal, scn["home"])
    assert (f.cost(goal) is None) == (not back.ok)
    if back.ok:
        assert f.cost(goal) == pytest.approx(back.cost)
        path = f.path(goal)
        assert path[0] == goal and path[-1] == tuple(scn["home"])
        assert not P.route_problems(g, path, p, scn["layers"])


@pytest.mark.parametrize("seed", range(12))
def test_bendy_ruler_returns_a_flyable_local_path(seed):
    scn, p, g = _grid(seed)
    start, goal = tuple(scn["home"]), tuple(scn["targets"][-1])
    cells, info = P.bendy_ruler(g, start, goal, length=3)
    if cells is None:
        return
    path = [start] + cells
    assert not P.route_problems(g, path, p, scn["layers"]), info
    assert info["probes"] == 10


# ---------------------------------------------------------------- live runtime
def test_runtime_reset_and_scheduled_antenna_pull():
    from janus.runtime import Runtime
    rt = Runtime(port_base=49700)

    def wait(pred, t):
        t0 = time.time()
        while time.time() - t0 < t:
            if pred():
                return True
            time.sleep(0.05)
        return False
    try:
        assert rt.prepare({"mode": "small", "preset": "Swift-V  VTOL fixed-wing", "script": "antenna", "seed": 1})["ok"]
        time.sleep(1.0)
        rt.launch()
        tc = rt.scn["story"]["t_cut"]
        assert wait(lambda: not rt.pm.alive("relay"), tc + 4), "antenna must be pulled without the tour"
        rt.command("cut", True)
        assert rt.reset()["ok"]
        assert rt.now() < 1.0 and rt.phase == "PREFLIGHT"
        assert rt.pm.alive("relay") and not rt.impair["blackout"]
        assert wait(lambda: (rt.drone_obs or {}).get("state") == "IDLE", 5)
    finally:
        rt.shutdown()
