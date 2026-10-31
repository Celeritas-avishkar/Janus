"""End-to-end story checks in the fast simulator (same code as the real processes)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest
from janus.config import make_profile, PRESETS
from janus.scenarios import build, _sim
from janus.sim import Sim

ALL = [(m, p) for m in PRESETS for p in PRESETS[m]]


@pytest.mark.parametrize("mode,preset", ALL)
def test_nominal_completes_without_exposure(mode, preset):
    prof = make_profile(mode, preset)
    scn = build("nominal", 2, mode, prof)
    s = _sim(scn, prof, seed=11)
    d = s.drone
    assert d.state == "COMPLETE"
    assert d.m["exposures"] == 0 and d.m["blocked_entries"] == 0
    assert s.gcs.uploads[-1]["status"] == "ACCEPTED"


@pytest.mark.parametrize("mode,preset", ALL)
def test_uplink_detects_beyond_sensor(mode, preset):
    prof = make_profile(mode, preset)
    try:
        scn = build("popup", 3, mode, prof)
    except RuntimeError:
        pytest.skip("no verified popup world for this preset")
    st = scn["story"]
    assert st["detect_uplink"] > prof["lookahead"]
    assert st["detect_sensor_only"] is None or st["detect_sensor_only"] <= prof["lookahead"]


def test_shadow_reconciles_black_box():
    prof = make_profile("small", "Scout-Q  light quadcopter")
    scn = build("shadow", 1, "small", prof)
    s = _sim(scn, prof, seed=5)
    o = s.gcs.outages[0]
    assert o["recovered"] and o["blind"], "black box must fill the blind track"
    assert o["err"] is not None and o["err"] >= 0
    assert s.drone.m["exposures"] == 0


def test_unsafe_upload_rejected_then_valid_accepted_on_bad_link():
    prof = make_profile("small", "Scout-Q  light quadcopter")
    scn = build("badlink", 1, "small", prof)
    s = Sim(scn, prof, seed=9, deterministic=True)
    s.run(0.4)
    s.gcs.upload(unsafe=True, launch=False)
    s.run(8)
    assert s.gcs.uploads[-1]["status"] == "REJECTED" and s.gcs.uploads[-1]["reason"]
    s.gcs.upload()
    s.run(120, stop=lambda x: x.done())
    assert s.gcs.uploads[-1]["status"] == "ACCEPTED"
    assert s.drone.state == "COMPLETE"
    assert s.dep.stats["crc_fail"] + s.gep.stats["crc_fail"] > 0, "corruption must have been caught by CRC"


def test_bingo_returns_home_with_reserve():
    prof = make_profile("small", "Hauler-H  heavy-lift hexacopter")
    scn = build("bingo", 1, "small", prof)
    s = _sim(scn, prof, seed=2)
    assert "BINGO" in [e["code"] for e in s.drone.events]
    assert s.drone.state == "LANDED" and s.drone.energy > 0


def test_relay_down_triggers_failsafe_and_recovers():
    prof = make_profile("small", "Scout-Q  light quadcopter")
    scn = build("antenna", 1, "small", prof)
    st = scn["story"]
    s = _sim(scn, prof, actions={st["t_cut"]: lambda x: setattr(x, "relay_alive", False),
                                 st["t_back"]: lambda x: setattr(x, "relay_alive", True)})
    codes = [e["code"] for e in s.drone.events]
    assert "FAILSAFE" in codes and s.drone.state in ("COMPLETE", "LANDED")
