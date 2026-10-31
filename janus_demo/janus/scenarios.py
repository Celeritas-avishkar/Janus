"""Story scenarios. Each builder generates a seeded world for a chosen drone
profile, then runs the fast headless simulator to check that the story really
happens (for example: the pop-up hazard is detected beyond sensor range, the
radio shadow causes an outage long enough to trigger the failsafe). Only
verified scenarios are returned; `verified` says what was checked."""
import math

from . import planner as P
from . import radio
from .config import MODES, COLS, ROWS, LAYERS
from .sim import Sim
from .world import base_scenario, box, initial_plan, grid_for

SCRIPTS = {
    "nominal": "Nominal mission",
    "popup": "Hazard appears mid-flight",
    "shadow": "Radio shadow: flying blind",
    "badlink": "Bad link + unsafe upload",
    "bingo": "Detour forces low-energy return",
    "antenna": "Antenna pulled mid-flight",
}


def energy_need(scn, profile, path):
    g = grid_for(scn, scn["static"] + [h for h in scn["hazards"] if h.get("t_on", 0) <= 0], profile)
    return g.path_cost(path) * 0.5 * profile["burn"]


def usable_energy(scn, profile):
    return profile["energy"] * scn.get("soc", 1.0) - profile["energy"] * profile["reserve_pct"] / 100.0


def coverage(scn, profile, path):
    """Predicted downlink SNR along a route (deterministic link budget)."""
    m = MODES[scn["mode"]]
    out = []
    for c in path:
        b = radio.budget(m, scn["gcs"], c, scn["static"], profile["tx_dbm"], profile["gain_dbi"])
        out.append(round(min(b["dn_snr"], b["up_snr"]), 1))
    return out


def weak_run(snrs, thresh=radio.SNR50_DB + 1.0):
    best = cur = 0
    for s in snrs:
        cur = cur + 1 if s < thresh else 0
        best = max(best, cur)
    return best


def time_to_index(path, profile, k):
    return 0.5 + sum(P.step_length(path[i], path[i + 1]) for i in range(max(0, k))) / profile["speed"]


def flight_time(path, profile):
    return sum(P.step_length(path[i], path[i + 1]) for i in range(len(path) - 1)) / profile["speed"]


def _sim(scn, profile, uplink=True, until=None, actions=None, seed=3):
    res, _, _ = initial_plan(scn, profile)
    until = until or (flight_time(res.path, profile) * 2.2 + 25)
    s = Sim(scn, profile, uplink=uplink, seed=seed, deterministic=True)
    s.run(0.4)
    s.gcs.upload()
    s.run(until, actions=actions, stop=lambda x: x.done())
    return s


def _robust(scn, profile, check, seeds=(17, 29), actions=None):
    """Re-run with random radio fading (as in the live relay) and require the story to hold."""
    for sd in seeds:
        res, _, _ = initial_plan(scn, profile)
        s = Sim(scn, profile, seed=sd, deterministic=False)
        s.run(0.4)
        s.gcs.upload()
        s.run(flight_time(res.path, profile) * 2.2 + 25, actions=actions, stop=lambda x: x.done())
        if not check(s):
            return False
    return True


def _base_ok(scn, profile, max_weak=1, max_frac=0.75):
    res, legs, _ = initial_plan(scn, profile)
    if not res.ok or len(res.path) < 18:
        return None
    if energy_need(scn, profile, res.path) > max_frac * usable_energy(scn, profile):
        return None
    if weak_run(coverage(scn, profile, res.path)) > max_weak:
        return None
    return res


# ---------------------------------------------------------------- builders
def build(script, seed, mode, profile, weather="breezy", tries=90):
    fn = {"nominal": _nominal, "popup": _popup, "shadow": _shadow, "badlink": _badlink,
          "bingo": _bingo, "antenna": _antenna}[script]
    last = None
    for k in range(tries):
        s = seed * 101 + k
        scn = base_scenario(s, mode, weather, script)
        scn["seed"] = s
        out = fn(scn, profile)
        if out is not None:
            out["story"]["seed_used"] = s
            out["story"]["title"] = SCRIPTS[script]
            return out
        last = scn
    raise RuntimeError(f"could not build a verified '{script}' scenario for {profile['name']} in {tries} tries")


def _nominal(scn, profile):
    res = _base_ok(scn, profile)
    if res is None:
        return None
    sim = _sim(scn, profile)
    ok = lambda s: s.drone.state == "COMPLETE" and not s.drone.m["exposures"]
    if not ok(sim) or not _robust(scn, profile, ok):
        return None
    scn["story"] = {"verified": ["mission completes", "no exposures", "link coverage along route"],
                    "flight_s": round(flight_time(res.path, profile), 1)}
    return scn


def _place_popup(scn, profile, path, frac, hid, t_on, kind=None):
    k = max(4, min(len(path) - 4, int(len(path) * frac)))
    c = path[k]
    protected = {tuple(scn["home"][:2])} | {tuple(t[:2]) for t in scn["targets"]}
    x0, y0 = max(0, c[0] - 1), max(0, c[1] - 1)
    w, h = 3, 3
    if any(x0 <= px < x0 + w and y0 <= py < y0 + h for (px, py) in protected):
        return None
    z0, z1 = max(0, c[2] - 2), min(LAYERS - 1, c[2] + 2)
    if scn["mode"] == "small":
        kind = kind or "gust"
        hz = box(hid, kind, x0, y0, w, h, z0, z1, t_on=t_on, source="WEATHER",
                 gust_kt=round(profile["gust_limit_kt"] + 9))
    else:
        kind = kind or "ash"
        hz = box(hid, kind, x0, y0, w, h, z0, z1, t_on=t_on, source="ADVISORY", sev=3)
    return hz, k


def _popup(scn, profile):
    res = _base_ok(scn, profile)
    if res is None:
        return None
    lead = int(profile["lookahead"]) + 4 + int(math.ceil(profile["speed"] * (scn["feed_latency"] + 0.6)))
    k = next((int(len(res.path) * f) for f in (0.62, 0.7, 0.78) if int(len(res.path) * f) - lead >= 3), None)
    if k is None or k > len(res.path) - 4:
        return None
    t_on = max(2.5, time_to_index(res.path, profile, k - lead))
    placed = _place_popup(scn, profile, res.path, k / len(res.path), f"W{scn['next_hazard_id']}", round(t_on, 1))
    if placed is None:
        return None
    hz, k = placed
    scn["hazards"].append(hz)
    scn["next_hazard_id"] += 1
    a = _sim(scn, profile, uplink=True)
    d = a.drone
    dist = d.m["detect_cells"].get(hz["id"])
    if d.state != "COMPLETE" or d.m["exposures"] or dist is None or dist <= profile["lookahead"]:
        return None
    if not _robust(scn, profile, lambda s: s.drone.state == "COMPLETE" and not s.drone.m["exposures"]
                   and (s.drone.m["detect_cells"].get(hz["id"]) or 0) > profile["lookahead"]):
        return None
    b = _sim(scn, profile, uplink=False)
    scn["story"] = {"popup": hz["id"], "t_on": hz["t_on"], "detect_uplink": dist,
                    "detect_sensor_only": b.drone.m["detect_cells"].get(hz["id"]),
                    "verified": [f"{hz['id']} detected {dist} cells ahead via uplink (sensor sees {profile['lookahead']})",
                                 "mission completes with zero exposures"]}
    return scn


def _shadow(scn, profile):
    """Tall building between the ground station and the middle of the route."""
    scn["static"] = [o for o in scn["static"] if o["type"] not in MODES[scn["mode"]]["physical"]]
    res, _, _ = initial_plan(scn, profile)
    if not res.ok or len(res.path) < 20:
        return None
    gx, gy = scn["gcs"][0], scn["gcs"][1]
    phys = "building" if scn["mode"] == "small" else "ridge"
    top = LAYERS - 1 if scn["mode"] == "small" else 5
    path = res.path
    on_path = {(c[0], c[1]) for c in path}
    protected = {tuple(scn["home"][:2])} | {tuple(t[:2]) for t in scn["targets"]}
    want_lo = 4.5      # >= LOST threshold + 2 s of ghost; fast drones may cross before their failsafe timer
    want_hi = profile["lost_link_s"] + 9.0
    best = None
    seen = set()
    n = len(path)
    tops = (top,) if scn["mode"] == "small" else (2, 3, 4, 5)
    sizes = ((2, 2), (2, 3), (3, 2)) if scn["mode"] == "small" else ((1, 3), (3, 1), (2, 2), (1, 4), (4, 1))
    for i in range(int(n * 0.25), int(n * 0.75)):
        m = path[i]
        for along, (w, h), top in ((a, sz, tp) for a in (0.45, 0.6, 0.75, 0.88) for sz in sizes for tp in tops):
                bx = int(round(gx + (m[0] - gx) * along))
                by = int(round(gy + (m[1] - gy) * along))
                x0, y0 = bx - w // 2, by - h // 2
                key = (x0, y0, w, h, top)
                if key in seen or x0 < 3 or y0 < 0 or x0 + w > COLS - 1 or y0 + h > ROWS:
                    continue
                seen.add(key)
                cells = {(x, y) for x in range(x0, x0 + w) for y in range(y0, y0 + h)}
                ring = {(x + dx, y + dy) for (x, y) in protected for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
                if cells & on_path or cells & ring:
                    continue                      # keep the planned route unchanged
                cand = box("B90", phys, x0, y0, w, h, 0, top, rf_db=32.0,
                           note="Tower block" if phys == "building" else "Ridge")
                trial = dict(scn)
                trial["static"] = scn["static"] + [cand]
                snr = coverage(trial, profile, path)
                weak = [k for k, s in enumerate(snr) if s < radio.SNR50_DB + 1.0]
                if not weak or weak[0] < n * 0.2:
                    continue
                run = weak_run(snr)
                secs = run / profile["speed"]
                tail = n - 1 - weak[-1]
                if want_lo <= secs <= want_hi and tail >= 5:
                    score = abs(secs - (profile["lost_link_s"] + 4.5)) + (0 if secs >= profile["lost_link_s"] + 1.5 else 3)
                    if best is None or score < best[0]:
                        best = (score, cand, snr)
    if best is None:
        return None
    _, cand, snr = best
    trial = dict(scn)
    trial["static"] = scn["static"] + [cand]
    r2, _, _ = initial_plan(trial, profile)
    if not r2.ok or [tuple(c) for c in r2.path] != [tuple(c) for c in path]:
        return None
    if energy_need(trial, profile, r2.path) > 0.55 * usable_energy(trial, profile):
        return None                      # headroom so a blind detour never turns into a low-energy return
    scn["static"].append(cand)
    weak = [i for i, s in enumerate(snr) if s < radio.SNR50_DB + 1.0]
    i0 = weak[0]
    t0 = sum(P.step_length(r2.path[i], r2.path[i + 1]) for i in range(i0)) / profile["speed"]
    # pop-up during the outage, on the route ahead, beyond sensor range at the time it appears
    target_i = min(len(r2.path) - 4, weak[-1] - 1)
    if target_i - i0 < 3:
        return None
    hz_info = _place_popup(scn, profile, r2.path, target_i / len(r2.path), f"W{scn['next_hazard_id']}", round(t0 + 1.5, 1))
    if hz_info is not None:
        scn["hazards"].append(hz_info[0])
        scn["next_hazard_id"] += 1
    sim = _sim(scn, profile)
    d, g = sim.drone, sim.gcs
    codes = [e["code"] for e in d.events]
    if d.m["exposures"] or not g.outages or not g.outages[0].get("recovered"):
        return None
    if d.state not in ("COMPLETE", "LANDED") or "BINGO" in codes:
        return None

    def holds(x):
        c = [e["code"] for e in x.drone.events]
        return (x.gcs.outages and x.gcs.outages[0].get("recovered") and not x.drone.m["exposures"]
                and "BINGO" not in c and x.drone.state in ("COMPLETE", "LANDED"))
    if not _robust(scn, profile, holds):
        return None
    o = g.outages[0]
    scn["story"] = {"building": cand["id"], "outage_s": round(o["t1"] - o["t0"], 1), "ghost_err": o["err"],
                    "popup": hz_info[0]["id"] if hz_info else None,
                    "failsafe": "FAILSAFE" in codes,
                    "verified": [f"line of sight blocked by {cand['id']}: outage {o['t1'] - o['t0']:.1f} s",
                                 ("failsafe triggered on the drone" if "FAILSAFE" in codes else
                                  "outage shorter than this drone's failsafe timer: it flew on"),
                                 "black box reconciles the blind track", "zero exposures"]}
    return scn


def _badlink(scn, profile):
    res = _base_ok(scn, profile)
    if res is None:
        return None
    from .gcs_brain import GcsBrain
    g = GcsBrain(scn, profile)
    unsafe = g._unsafe_route()
    grid = grid_for(scn, scn["static"] + scn["hazards"], profile)
    if not P.route_problems(grid, unsafe, profile, scn["layers"]):
        mid = unsafe[len(unsafe) // 2]
        scn["hazards"].append(box(f"W{scn['next_hazard_id']}", "gust" if scn["mode"] == "small" else "turb",
                                  max(0, mid[0] - 1), max(0, mid[1] - 1), 3, 3, max(0, mid[2] - 1), min(LAYERS - 1, mid[2] + 1),
                                  gust_kt=profile["gust_limit_kt"] + 8, sev=3, source="WEATHER"))
        scn["next_hazard_id"] += 1
        if _base_ok(scn, profile) is None:
            return None
    scn["impair"] = {"loss_up": 0.30, "loss_dn": 0.20, "corrupt": 0.12, "dup": 0.05, "latency_ms": 60, "jitter_ms": 30}
    scn["story"] = {"verified": ["unsafe straight-line route intersects a blocking volume",
                                 "valid route affordable and inside coverage"]}
    return scn


def _bingo(scn, profile):
    res = _base_ok(scn, profile, max_frac=1.0)
    if res is None:
        return None
    need = energy_need(scn, profile, res.path)
    reserve = profile["energy"] * profile["reserve_pct"] / 100.0
    scn["soc"] = round(min(1.0, (need * 1.08 + reserve) / profile["energy"]), 3)
    ft = flight_time(res.path, profile)
    for frac in (0.55, 0.62, 0.7):
        trial = dict(scn)
        trial["hazards"] = list(scn["hazards"])
        placed = _place_popup(trial, profile, res.path, frac, f"W{scn['next_hazard_id']}", round(max(3.0, 0.25 * ft), 1))
        if placed is None:
            continue
        trial["hazards"].append(placed[0])
        sim = _sim(trial, profile)
        codes = [e["code"] for e in sim.drone.events]
        if "BINGO" in codes and sim.drone.state == "LANDED" and not sim.drone.m["exposures"]:
            trial["next_hazard_id"] = scn["next_hazard_id"] + 1
            trial["story"] = {"soc": trial["soc"], "popup": placed[0]["id"],
                              "verified": ["mission affordable at launch", "detour makes it unaffordable",
                                           "drone turns back on its own and lands with reserve"]}
            return trial
    return None


def _antenna(scn, profile):
    res = _base_ok(scn, profile)
    if res is None:
        return None
    ft = flight_time(res.path, profile)
    if ft < 9:
        return None
    t_cut, t_back = round(0.3 * ft + 1, 1), round(0.3 * ft + 1 + profile["lost_link_s"] + 6, 1)

    def cut(s):
        s.relay_alive = False

    def back(s):
        s.relay_alive = True

    sim = _sim(scn, profile, actions={t_cut: cut, t_back: back})
    codes = [e["code"] for e in sim.drone.events]
    if "FAILSAFE" not in codes or sim.drone.m["exposures"] or sim.drone.state not in ("COMPLETE", "LANDED"):
        return None
    scn["story"] = {"t_cut": t_cut, "t_back": t_back,
                    "verified": ["failsafe triggers after relay is killed", "mission recovers after restart"]}
    return scn
