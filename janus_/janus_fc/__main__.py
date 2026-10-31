"""Command line.

  # plan only (no flight controller): check zones and route, write a QGC/MP waypoint file
  python -m janus_fc plan --home 47.397742,8.545594 --goal 47.4010,8.5520,40 --zones examples/zones_px4_sitl.json

  # fly (companion computer wired to the flight controller)
  python -m janus_fc fly --connect /dev/ttyAMA0 --baud 921600 --goal LAT,LON,ALT --zones zones.json

  # PX4 SITL / ArduPilot SITL on this machine
  python -m janus_fc fly --connect udpin:0.0.0.0:14540 --goal ... --zones ... --auto-launch --sitl
  python -m janus_fc fly --connect tcp:127.0.0.1:5760  --goal ... --zones ... --auto-launch
"""
import argparse
import json
import sys
import time

from .config import load_profile, PROFILES
from .geo import LocalFrame
from .mav import Autopilot, MavError, M


def _ll(s):
    parts = [float(x) for x in s.split(",")]
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("use LAT,LON or LAT,LON,ALT")
    return tuple(parts)


def write_waypoints(path, home, items, offset_home=True):
    """QGC WPL 110 text format: opens in QGroundControl and Mission Planner."""
    with open(path, "w") as f:
        f.write("QGC WPL 110\n")
        f.write(f"0\t1\t0\t16\t0\t0\t0\t0\t{home[0]:.7f}\t{home[1]:.7f}\t0\t1\n")
        for i, it in enumerate(items, start=1):
            p4 = it.get("p4", 0)
            p4 = 0 if p4 != p4 else p4
            f.write(f"{i}\t0\t{it.get('frame', 3) if it.get('frame') != 6 else 3}\t{it['cmd']}\t{it.get('p1', 0)}\t{it.get('p2', 0)}\t"
                    f"{it.get('p3', 0)}\t{p4}\t{it.get('lat', 0):.7f}\t{it.get('lon', 0):.7f}\t{it.get('alt', 0)}\t1\n")


def cmd_plan(a):
    from . import mission as MS
    from . import planner as P
    from .geo import heading_ne
    from .zones import ZoneStore
    prof = load_profile(a.vehicle if a.vehicle != "auto" else "copter", a.profile, a.sar)
    frame = LocalFrame(a.home[0], a.home[1])
    store = ZoneStore(a.zones, frame)
    start = (0.0, 0.0, prof["takeoff_alt_m"])
    route, hold, grid = [start], None, None
    t0 = time.perf_counter()
    for g in a.goal:
        n, e = frame.to_ne(g[0], g[1])
        goal = (n, e, g[2] if len(g) > 2 else prof["alt_layers_m"][0])
        res, grid = P.plan(store.known(), prof, route[-1], goal, t=time.time(), horizon=prof["replan_horizon_s"])
        if not res.ok:
            if res.reason.startswith("goal blocked"):
                hold = P.choose_hold(grid, goal, heading_ne(goal[0] - route[-1][0], goal[1] - route[-1][1]), prof)
                if hold:
                    res, grid = P.plan(store.known(), prof, route[-1], hold["fix"], t=time.time(), horizon=prof["replan_horizon_s"])
                    route += res.path[1:]
                    print(f"goal blocked: {hold['kind']} hold at fix {hold['fix'][0]:.0f} N {hold['fix'][1]:.0f} E")
                    break
            print(f"NO ROUTE: {res.reason}")
            return 2
        route += res.path[1:]
    ms = (time.perf_counter() - t0) * 1000
    items, notes = MS.build(frame, prof, a.stack, route, takeoff=True, hold=hold, grid=grid)
    items = MS.fix_jumps(items, 1)
    for n in notes:
        print("note:", n)
    print(f"{prof['class']}: {len(route) - 1} waypoints, {len(items)} mission items, planned in {ms:.0f} ms "
          f"(grid {grid.rows}x{grid.cols}x{len(grid.layers)} @ {grid.cell:.0f} m, margin {prof['margin_m']:.0f} m)")
    for k, p in enumerate(route):
        lat, lon = frame.to_ll(p[0], p[1])
        print(f"  {k:2d}  {lat:.6f}, {lon:.6f}  {p[2]:5.0f} m")
    if a.out:
        write_waypoints(a.out, a.home, items)
        print(f"wrote {a.out} (open in QGroundControl or Mission Planner to review)")
    return 0


def cmd_fly(a):
    from .agent import Agent
    ap = Autopilot(a.connect, source_system=a.source_system, baud=a.baud)
    ap.connect(timeout=a.connect_timeout)
    cls = ap.vclass if a.vehicle == "auto" else a.vehicle
    prof = load_profile(cls, a.profile, a.sar)
    if a.sitl:
        _sitl_params(ap)
    goals = [(g[0], g[1], g[2] if len(g) > 2 else prof["alt_layers_m"][0]) for g in a.goal]
    agent = Agent(ap, prof, goals, zones_path=a.zones, advisory_port=a.advisory_port, auto_launch=a.auto_launch,
                  upload_fence=a.fence, blackbox=a.blackbox, max_mission_s=a.max_time)
    try:
        state = agent.run()
    except KeyboardInterrupt:
        print("\n[janus] stopped by user: the autopilot keeps flying its current mission")
        state = agent.state
    except MavError as exc:
        print(f"[janus] ERROR: {exc}")
        return 3
    print(json.dumps(agent.summary(), indent=2))
    return 0 if state == "DONE" else 1


def _sitl_params(ap):
    """Simulator-only relaxations so SITL can fly with no RC transmitter or GCS.
    NEVER use --sitl on a real aircraft."""
    if ap.stack == "px4":
        ap.param_set("COM_RC_IN_MODE", 4, M.MAV_PARAM_TYPE_INT32)    # no RC required
        ap.param_set("NAV_DLL_ACT", 0, M.MAV_PARAM_TYPE_INT32)       # no GCS-loss failsafe
        ap.param_set("COM_DISARM_PRFLT", 60, M.MAV_PARAM_TYPE_REAL32)
        ap.param_set("SIM_BAT_DRAIN", 1800, M.MAV_PARAM_TYPE_REAL32)  # default 60 s: unrealistically fast
    print("[janus] SITL parameters set (simulator only)")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="janus_fc", description="Janus adaptive route planner for ArduPilot and PX4")
    sub = ap.add_subparsers(dest="cmd", required=True)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--goal", type=_ll, action="append", required=True, help="LAT,LON[,ALT m above home]; repeat for waypoints")
    common.add_argument("--zones", help="zone file (JSON)")
    common.add_argument("--vehicle", choices=["auto"] + list(PROFILES), default="auto")
    common.add_argument("--profile", help="JSON file overriding profile values")
    common.add_argument("--sar", action="store_true", help="search-and-rescue rules (TFRs and LOW ash passable at a cost)")
    p = sub.add_parser("plan", parents=[common], help="plan only, no flight controller")
    p.add_argument("--home", type=_ll, required=True)
    p.add_argument("--stack", choices=["ardupilot", "px4"], default="ardupilot")
    p.add_argument("--out", help="write a .waypoints file")
    f = sub.add_parser("fly", parents=[common], help="connect to a flight controller and fly")
    f.add_argument("--connect", required=True, help="pymavlink connection: /dev/ttyAMA0, udpin:0.0.0.0:14540, tcp:127.0.0.1:5760")
    f.add_argument("--baud", type=int, default=921600)
    f.add_argument("--source-system", type=int, default=1, help="MAVLink system id Janus uses (same as the vehicle; component 191)")
    f.add_argument("--connect-timeout", type=float, default=60)
    f.add_argument("--auto-launch", action="store_true", help="Janus arms and takes off (SITL / supervised tests)")
    f.add_argument("--fence", action="store_true", help="also upload hard zones as exclusion fences (backstop)")
    f.add_argument("--advisory-port", type=int, help="UDP port for JSON zone advisories")
    f.add_argument("--blackbox", default=f"janus_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    f.add_argument("--max-time", type=float, default=3600)
    f.add_argument("--sitl", action="store_true", help="simulator-only parameter relaxations (never on hardware)")
    # "--goal -35.3,149.1": argparse would read the negative latitude as an option
    raw = list(sys.argv[1:] if argv is None else argv)
    fixed, i = [], 0
    while i < len(raw):
        if raw[i] in ("--goal", "--home") and i + 1 < len(raw) and raw[i + 1][:1] == "-" and raw[i + 1][1:2].isdigit():
            fixed.append(f"{raw[i]}={raw[i + 1]}")
            i += 2
            continue
        fixed.append(raw[i])
        i += 1
    a = ap.parse_args(fixed)
    return cmd_plan(a) if a.cmd == "plan" else cmd_fly(a)


if __name__ == "__main__":
    sys.exit(main())
