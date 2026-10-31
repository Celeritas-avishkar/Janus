"""Round 4 checks: volcanic plume physics, real holding patterns, SAR through
TFRs, measured-wind ash tracking, stop/start clock."""
import math
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest

from janus import holding as HOLD
from janus import planner as P
from janus.config import make_profile, apply_sar
from janus.scenarios import build, _sim
from janus.world import base_scenario


def _plume(**kw):
    h = {"id": "V1", "type": "ash", "shape": "plume", "vent": [5.0, 8.0], "ux": 1.0, "uy": 0.0, "speed": 0.4,
         "vx": 0.4, "vy": 0.0, "every": 1.5, "life": 28.0, "t_start": -28.0, "r0": 0.8, "dr": 0.05, "tau": 14.0,
         "c0": 8.0, "z0": 0, "z1": 6, "meander": 0.0}
    h.update(kw)
    return h


# ---------------------------------------------------------------- plume
def test_plume_is_dense_at_the_vent_and_fades_downwind():
    puffs = sorted(P.plume_puffs(_plume(), 0.0), key=lambda p: p["cx"])
    assert len(puffs) > 8
    assert puffs[0]["cx"] < puffs[-1]["cx"]                                  # carried downwind (+x)
    assert puffs[0]["conc_mg"] > puffs[-1]["conc_mg"]                        # thins out
    assert puffs[0]["r"] < puffs[-1]["r"]                                    # and spreads
    assert all(p["conc_mg"] >= 0.2 for p in puffs)                           # dispersed puffs are dropped


def test_plume_pattern_is_steady_but_puffs_move():
    h = _plume()
    a, b = P.plume_puffs(h, 0.0), P.plume_puffs(h, 6.0)
    assert abs(len(a) - len(b)) <= 1                                         # a steady plume, new puffs keep coming
    assert P.zone_contains_xy(h, 5, 8, 0.0) and P.zone_contains_xy(h, 5, 8, 50.0)


def test_frozen_plume_does_not_move():
    h = _plume(freeze=0.0)
    assert [round(p["cx"], 3) for p in P.plume_puffs(h, 0.0)] == [round(p["cx"], 3) for p in P.plume_puffs(h, 30.0)]


def test_generated_vents_keep_clear_of_launch_and_targets():
    for mode in ("small", "aircraft"):
        for seed in range(15):
            scn = base_scenario(seed, mode, "breezy")
            for h in [h for h in scn["hazards"] if h.get("shape") == "plume"]:
                for pt in [scn["home"]] + scn["targets"]:
                    assert P.zone_distance(h, pt[0], pt[1], 0.0) >= 1.6


def test_grid_severity_follows_the_puff():
    h = _plume()
    p = make_profile("small", "Scout-Q  light quadcopter")
    sar = apply_sar(p)
    g_n, g_s = P.Grid(25, 17, 9, [h], p), P.Grid(25, 17, 9, [h], sar)
    tail = sorted(P.plume_puffs(h, 0.0), key=lambda q: q["cx"])[-1]
    c = (int(round(tail["cx"])), int(round(tail["cy"])), 1)
    assert tail["sev"] == 1 and not g_n.free(c) and g_s.free(c)               # faint tail: SAR may cross it


# ---------------------------------------------------------------- holding patterns
@pytest.mark.parametrize("kind", list(HOLD.KINDS))
def test_hold_patterns_are_closed_flyable_loops(kind):
    pts = HOLD.pattern(kind, (10, 8, 1), (1.0, 0.0), "small")
    cum = HOLD.arc_lengths(pts)
    assert pts[0] == (10.0, 8.0) and cum[-1] > 5.0
    steps = [math.dist(a, b) for a, b in zip(pts, pts[1:] + pts[:1])]
    assert max(steps) < 0.8, "no jumps along the pattern"


def test_standard_racetrack_turns_right():
    pts = HOLD.pattern("racetrack", (10, 8, 1), (1.0, 0.0), "small")       # inbound heading east
    assert max(p[1] for p in pts) > 8.5 and min(p[1] for p in pts) > 7.9    # +y (south) = right of east


@pytest.mark.parametrize("kind", ["racetrack", "figure8", "orbit"])
def test_drone_flies_the_chosen_pattern_and_leaves_straight_for_the_goal(kind):
    p = make_profile("small", "Swift-V  VTOL fixed-wing", hold_pattern=kind)
    scn = build("holding", 1, "small", p)
    s = _sim(scn, p, seed=4)
    d = s.drone
    entry = next(e for e in d.events if e["code"] == "HOLD_ENTRY")
    assert HOLD.KINDS[kind] in entry["text"]
    codes = [e["code"] for e in d.events]
    i = codes.index("HOLD_CLEAR")
    assert d.state == "COMPLETE" and "WAYPOINT" not in codes[i:], "no going back for a waypoint after the hold"
    # the drone really flew a path: it visited several cells while holding
    held = [r for r in d.bb if r[5] == "HOLDING"]
    assert len({(round(r[2]), round(r[3])) for r in held}) >= 3


# ---------------------------------------------------------------- SAR and TFR
def test_sar_may_enter_a_tfr():
    p = make_profile("aircraft", "Sentinel  MALE")
    tfr = {"type": "tfr"}
    assert P.hazard_blocks(tfr, p) and not P.hazard_blocks(tfr, apply_sar(p))


# ---------------------------------------------------------------- ash trajectory from measured wind
def test_estimate_drifts_with_the_measured_wind():
    p = make_profile("aircraft", "Sentinel  MALE")
    scn = build("escape", 1, "aircraft", p)
    scn["env"]["drift"] = 0.5
    s = _sim(scn, p, seed=4)
    est = [h for h in s.drone.sensed.values() if h.get("estimated")]
    assert est and all("wind" in h for h in est)
    w = scn["wind"]
    for h in est:
        assert abs(((h["wind"][0] - w["dir"]) + 180) % 360 - 180) < 20   # measured within a few degrees
        if h["vx"] or h["vy"]:
            toward = math.degrees(math.atan2(h["vx"], -h["vy"])) % 360
            assert abs(((toward - (w["dir"] + 180)) + 180) % 360 - 180) < 25


# ---------------------------------------------------------------- stop clock
def test_stop_clock_freezes_everything_without_link_loss():
    from janus.runtime import Runtime
    rt = Runtime(port_base=50300)

    def wait(pred, t):
        t0 = time.time()
        while time.time() - t0 < t:
            if pred():
                return True
            time.sleep(0.05)
        return False
    try:
        rt.prepare({"mode": "small", "preset": "Swift-V  VTOL fixed-wing", "script": "nominal", "seed": 1})
        time.sleep(1.0)
        rt.launch()
        assert wait(lambda: (rt.drone_obs or {}).get("state") == "ENROUTE", 6)
        time.sleep(1.0)
        rt.command("pause", True)
        time.sleep(0.5)                                   # the pause command crosses to the drone process
        t1, p1 = rt.now(), list((rt.drone_obs or {}).get("pos"))
        time.sleep(3.5)                                   # longer than the lost-link threshold
        assert rt.now() == t1 and (rt.drone_obs or {}).get("pos") == p1
        rt.command("pause", False)
        time.sleep(1.5)
        assert rt.gcs.link == "CONNECTED" and (rt.drone_obs or {}).get("link") == "CONNECTED"
        assert not (rt.drone_obs or {}).get("failsafe")
        assert rt.now() > t1
    finally:
        rt.shutdown()
