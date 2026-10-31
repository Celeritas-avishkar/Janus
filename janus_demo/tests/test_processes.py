"""Real multi-process test: GCS runtime + air relay process + drone process over UDP.
Runs in real time (~1-2 minutes)."""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import pytest  # noqa: E402

from janus.runtime import Runtime  # noqa: E402


def wait_for(fn, timeout, step=0.1):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = fn()
        if v:
            return v
        time.sleep(step)
    return None


@pytest.fixture(scope="module")
def rt():
    r = Runtime(port_base=48800)
    yield r
    r.shutdown()


def test_processes_come_up(rt):
    assert wait_for(lambda: rt.prepare({"script": "nominal", "seed": 1}).get("ok"), 20)
    snap = wait_for(lambda: (lambda s: s if all(p["alive"] for p in s["procs"]) else None)(rt.snapshot()), 15)
    assert snap, f"processes not alive: {rt.snapshot()['procs']}"
    pids = {p["name"]: p["pid"] for p in snap["procs"]}
    assert len(set(pids.values())) == 3, "three distinct OS processes"
    assert wait_for(lambda: rt.gcs.link == "CONNECTED", 10), "pre-flight link over UDP"


def test_nominal_mission_over_udp(rt):
    rt.prepare({"script": "nominal", "seed": 1})
    rt.launch()
    assert wait_for(lambda: (rt.drone_obs or {}).get("state") == "COMPLETE", 90), (rt.drone_obs or {}).get("events")
    ev = rt.evidence()
    assert ev["exposures"] == 0 and ev["blocked_entries"] == 0
    assert rt.gcs.uploads[-1]["status"] == "ACCEPTED"
    counts = rt.snapshot()["air"]["counts"]
    assert counts["up"].get("ok", 0) > 10 and counts["dn"].get("ok", 0) > 50


def test_kill_relay_triggers_failsafe_and_blackbox(rt):
    rt.prepare({"script": "antenna", "seed": 1, "preset": "Swift-V  VTOL fixed-wing"})
    st = rt.scn["story"]
    rt.launch()
    assert wait_for(lambda: rt.now() >= st["t_cut"], 30)
    rt.command("kill_relay")
    assert not rt.snapshot()["procs"][1]["alive"]
    assert wait_for(lambda: rt.gcs.link == "LOST", 10)
    assert wait_for(lambda: "FAILSAFE" in [e["code"] for e in (rt.drone_obs or {}).get("events", [])], 15)
    assert rt.gcs.ghost is not None, "ground station shows a dead-reckoned estimate"
    rt.command("restart_relay")
    assert wait_for(lambda: rt.gcs.link == "CONNECTED", 15), "link back after relay restart"
    assert wait_for(lambda: rt.gcs.outages and rt.gcs.outages[-1]["recovered"], 20), "black box fills the gap"
    assert wait_for(lambda: (rt.drone_obs or {}).get("state") in ("COMPLETE", "LANDED"), 90)
