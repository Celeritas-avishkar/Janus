"""End-to-end test against a real ArduPilot or PX4 SITL binary.

  python tools/sitl_test.py ardupilot copter --ap-root ~/ardupilot
  python tools/sitl_test.py ardupilot plane  --ap-root ~/ardupilot
  python tools/sitl_test.py px4 copter --px4-root ~/PX4-Autopilot
  python tools/sitl_test.py px4 plane  --px4-root ~/PX4-Autopilot

Starts the simulator, runs Janus with --auto-launch, then checks the black box:
mission finished, the flown track never entered a blocking zone (true zones,
including the unreported one), and the unreported ash was found by the sensor.
"""
import argparse
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from janus_fc.agent import Agent                      # noqa: E402
from janus_fc.config import make_profile              # noqa: E402
from janus_fc.geo import LocalFrame                   # noqa: E402
from janus_fc.mav import Autopilot                    # noqa: E402
from janus_fc.zones import ZoneStore, blocks          # noqa: E402

HOMES = {"px4": (47.397742, 8.545594), "ardupilot": (-35.363262, 149.165237)}
GOALS = {"copter": (650, 650, 40), "plane": (2200, 2200, 120)}


def start_ardupilot(root, cls, workdir):
    binary = os.path.join(root, "build", "sitl", "bin", "arducopter" if cls == "copter" else "arduplane")
    parm = os.path.join(root, "Tools", "autotest", *(("default_params", "copter.parm") if cls == "copter" else ("models", "plane.parm")))
    home = "%f,%f,584,353" % HOMES["ardupilot"]
    model = "+" if cls == "copter" else "plane"
    cmd = [binary, "--model", model, "--speedup", "1", "--defaults", parm, "--home", home, "-I0", "--wipe"]
    p = subprocess.Popen(cmd, cwd=workdir, stdout=open(os.path.join(workdir, "sitl.log"), "w"), stderr=subprocess.STDOUT,
                         preexec_fn=os.setsid)
    return p, "tcp:127.0.0.1:5760"


def start_px4(root, cls, workdir):
    """PX4 SIH (simulation-in-hardware): PX4's own physics, no Gazebo needed.
    Same as `make px4_sitl sihsim_quadx` / `sihsim_airplane`. SIH's default
    location (SIH_LOC_LAT0/LON0) is the PX4 home used in the example zones."""
    build = os.path.join(root, "build", "px4_sitl_default")
    env = dict(os.environ, PX4_SIM_MODEL="sihsim_quadx" if cls == "copter" else "sihsim_airplane",
               PX4_SIMULATOR="sihsim")
    rootfs = os.path.join(workdir, "rootfs")
    os.makedirs(rootfs, exist_ok=True)
    cmd = [os.path.join(build, "bin", "px4"), "-d", "-w", rootfs, os.path.join(build, "etc")]
    p = subprocess.Popen(cmd, cwd=rootfs, env=env, stdout=open(os.path.join(workdir, "sitl.log"), "w"),
                         stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, preexec_fn=os.setsid)
    return p, "udpin:0.0.0.0:14540"


def check(blackbox, zones_path, prof, home):
    f = LocalFrame(*home)
    st = ZoneStore(zones_path, f)
    rows = [json.loads(line) for line in open(blackbox)]
    pos = [r for r in rows if r["kind"] == "pos" and r.get("armed") and r.get("alt") and r["alt"] > 5]
    events = [r["code"] for r in rows if r["kind"] == "event"]
    worst = math.inf
    worst_zone = None
    for r in pos:
        n, e = f.to_ne(r["lat"], r["lon"])
        for z in st.truth():
            if not blocks(z, prof)[0] or not z.in_alt(r["alt"]):
                continue
            d = z.distance(n, e, r["t"])
            if d < worst:
                worst, worst_zone = d, z.id
    return {"events": events, "closest_blocking_m": round(worst, 1), "closest_zone": worst_zone, "pos_samples": len(pos)}


def main():
    a = argparse.ArgumentParser()
    a.add_argument("stack", choices=["ardupilot", "px4"])
    a.add_argument("cls", choices=["copter", "plane"])
    a.add_argument("--ap-root", default=os.path.expanduser("~/ardupilot"))
    a.add_argument("--px4-root", default=os.path.expanduser("~/PX4-Autopilot"))
    a.add_argument("--max-time", type=float, default=900)
    a.add_argument("--keep", action="store_true")
    args = a.parse_args()
    work = tempfile.mkdtemp(prefix=f"janus_{args.stack}_{args.cls}_")
    zones = os.path.join(HERE, "..", "examples", f"zones_{args.stack}_{args.cls}.json")
    proc, conn = (start_ardupilot(args.ap_root, args.cls, work) if args.stack == "ardupilot"
                  else start_px4(args.px4_root, args.cls, work))
    print(f"[test] {args.stack} {args.cls} SITL pid {proc.pid}, logs in {work}")
    bb = os.path.join(work, "blackbox.jsonl")
    try:
        time.sleep(3)
        ap = Autopilot(conn).connect(timeout=90)
        prof = make_profile(args.cls)
        wind = json.load(open(zones)).get("wind")
        if args.stack == "ardupilot" and wind:
            # give the simulator the same wind as the zone file, so the measured wind is real
            ap.param_set("SIM_WIND_DIR", wind["from_deg"])
            ap.param_set("SIM_WIND_SPD", wind["speed_ms"])
        if args.stack == "px4":
            from janus_fc.__main__ import _sitl_params
            _sitl_params(ap)
        f = LocalFrame(*HOMES[args.stack])
        g = GOALS[args.cls]
        lat, lon = f.to_ll(g[0], g[1])
        agent = Agent(ap, prof, [(lat, lon, g[2])], zones_path=zones, auto_launch=True, blackbox=bb,
                      max_mission_s=args.max_time)
        t0 = time.time()
        state = agent.run()
        summ = agent.summary()
        summ["wall_s"] = round(time.time() - t0)
        res = check(bb, zones, prof, ap.s["home"][:2])
        summ.update(res)
        print(json.dumps(summ, indent=2))
        ok = state == "DONE" and res["closest_blocking_m"] > 0 and "ASH" in res["events"]
        print("[test] PASS" if ok else "[test] FAIL")
        return 0 if ok else 1
    finally:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        time.sleep(1)
        if not args.keep:
            shutil.copy(bb, os.path.join(HERE, "..", f"last_{args.stack}_{args.cls}.jsonl")) if os.path.exists(bb) else None


if __name__ == "__main__":
    sys.exit(main())
