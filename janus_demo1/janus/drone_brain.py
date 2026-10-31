"""Drone agent logic, independent of networking.

The drone plans only with what it KNOWS: the static map database it carries,
the latest hazard picture uplinked by the ground station, and hazards its own
forward sensor has seen. Truth is used only by the sensor model and for the
exposure metric (did the drone enter a hazard it did not know about?).

Failsafe policies on lost link (profile setting):
  CONTINUE        keep flying the mission on the last known map
  RETURN_TO_LINK  retrace the flown track to the last point with a good link,
                  wait there, resume when the link is back
  RTL             return to launch and land (no resume)
  HOLD            hold position, then return to launch
"""
import math
import time
from collections import deque

from . import config as C
from . import planner as P
from . import holding as HOLD
import random

from .world import truth_all, truth_hazards, wind_tuple, ash_concentration, circle, drift_velocity, PLAN_HORIZON_S

FLYING = ("ENROUTE", "RTL", "RTLINK")
AIRBORNE = FLYING + ("REPLAN", "HOLD", "HOLDING")


def _mid(a, b):
    return [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]


class DroneBrain:
    def __init__(self, scn, profile, drone_id=1, uplink_enabled=True):
        self.scn, self.p, self.id = scn, profile, drone_id
        self.uplink_enabled = uplink_enabled
        self.L = scn["layers"]
        self.home = tuple(scn["home"])
        self.uplink, self.sensed = {}, {}
        self.pic_v = 0
        self.state = "IDLE"
        self.resume_state = None
        self.route, self.seg, self.progress = [self.home], 0, 0.0
        self.targets, self.target_idx, self.rid = [], 0, None
        self.cell = self.home
        self.pos = [float(self.home[0]), float(self.home[1]), float(self.home[2])]
        self.energy = float(profile["energy"]) * float(scn.get("soc", 1.0))
        self.energy0 = self.energy
        self.t = 0.0
        self.last_rx = None
        self.link = "NO_LINK"
        self.link_ok_since = None
        self.failsafe = None
        self.failsafe_reason = None
        self.fs_count = 0
        self.fs_policy = None
        self.rx_times = deque(maxlen=24)
        self.last_rx_i = 0
        self._bingo_check_t = 0.0
        self.last_loss_cell = None
        self.hold_until = None
        self.replan_at = None
        self.replan_why = None
        self.retry_at = None
        self.flown = [self.home]
        self.last_good_i = 0
        self.route_rev = 0
        self.tm_seq = 0
        self.next_tm = 0.0
        self.bb, self.next_bb = [], 0.0
        self.gcs_have = -1
        self.next_bb_send = 0.0
        self.events = []
        self.outbox = []
        self.detect = {}
        self._grid_cache = None
        self._field_cache = None
        self.planner = profile.get("planner", "hybrid")
        self.sensor_hits = []          # recent own-sensor detections, for the sensor-cone flash on screen
        self.last_algo = None          # which algorithm produced the current route
        self.m = {"replans": 0, "replan_ms": [], "blocked_entries": 0, "exposures": 0, "sensed": 0,
                  "cells": 0, "dist": 0.0, "detect_cells": {}, "rejected": 0, "accepted": 0,
                  "failsafes": 0, "budget_trips": 0, "path_cost": 0.0, "bendy": 0, "bendy_ms": [],
                  "algo": {}, "field_ms": [], "rtl_cmds": 0, "escapes": 0, "ash_s": 0.0, "holds": 0,
                  "hold_s": 0.0}
        # ---- onboard air sampling (in-situ: the sensors measure the air the drone is in)
        mode = C.MODES[scn["mode"]]
        self.ts = float(mode["time_scale"])              # real seconds per demo second
        self.dose_full = float(mode["dose_full"])
        self.rng = random.Random(f"sensor:{scn.get('seed', 0)}")
        self.c_true, self.c_hid = 0.0, None
        self.opc, self.opc_hits, self.opc_sat, self.next_opc = 0.0, 0, False, 0.0
        self.so2, self.so2_confirmed = 0.0, set()
        self.clean_i = 0                                  # index in flown of the last cell sampled clean
        self.in_ash = None                                # plume id while an encounter is being handled
        self.encounters = {}
        self.escape_end = -1                              # route index where the escape segment ends
        self.dose = 0.0                                   # mg*s/m3, real time
        self.health = 1.0
        self.flags = set()                                # ASH_DAMAGE, AIRSPEED, ...
        self.abort_pending = None
        self.hold = None                                  # {"target", "fix", "mode", "t0", "why"}
        self._sensed_rev = 0
        self._esc_len = 0
        self.ash_samples = []                             # (x, y, t, mg/m3): the drone's own ash map
        self.wind_meas = None
        self.paused = False

    # ---------------------------------------------------------- knowledge
    def known(self):
        hz = {h["id"]: h for h in self.scn["static"]}
        hz.update(self.sensed)
        hz.update(self.uplink)
        return list(hz.values())

    def grid(self):
        known = self.known()
        moving = any(h.get("vx") or h.get("vy") for h in known)
        # drifting zones: rebuild every half second (cell-level motion), planning
        # against the area each zone will sweep over the next PLAN_HORIZON_S
        key = (self.pic_v, self._sensed_rev, tuple(sorted(self.uplink)), round(self.t * 2) if moving else 0)
        if self._grid_cache is None or self._grid_cache[0] != key:
            g = P.Grid(self.scn["cols"], self.scn["rows"], self.L, known, self.p, wind=wind_tuple(self.scn),
                       t=self.t, horizon=PLAN_HORIZON_S if moving else 0.0)
            self._grid_cache = (key, g)
        return self._grid_cache[1]

    def grid_now(self):
        """Known zones where they are right now (no swept horizon): used for the
        safety invariant, which must not count cells a zone will only reach later."""
        known = self.known()
        if not any(h.get("vx") or h.get("vy") for h in known):
            return self.grid()
        key = (self.pic_v, self._sensed_rev, tuple(sorted(self.uplink)), round(self.t * 4))
        if getattr(self, "_now_cache", None) is None or self._now_cache[0] != key:
            self._now_cache = (key, P.Grid(self.scn["cols"], self.scn["rows"], self.L, known, self.p,
                                           wind=wind_tuple(self.scn), t=self.t))
        return self._now_cache[1]

    def _moving_known(self):
        return any(h.get("vx") or h.get("vy") for h in self.known())

    def home_field(self):
        """Reverse-Dijkstra cost-to-home field for the current known map (cached)."""
        g = self.grid()
        if self._field_cache is None or self._field_cache[0] is not g:
            f = P.CostField(g, self.home, self.p["budget"] * 4)
            self.m["field_ms"].append(round(f.ms, 2))
            self.m["field_ms"] = self.m["field_ms"][-20:]
            self._field_cache = (g, f)
        return self._field_cache[1]

    def cost_home(self):
        """Cost of the cheapest route home from here, or None if there is none."""
        if self.planner == "astar":
            r = P.astar(self.grid(), self.cell, self.home, self.p["budget"])
            return r.cost if r.ok else None
        return self.home_field().cost(self.cell)

    def _global_plan(self, start, targets):
        """Global planner per profile: A* (hybrid, astar) or Dijkstra (dijkstra).
        Routes home in hybrid/dijkstra mode come straight from the cost field."""
        grid = self.grid()
        self._esc_len = 0
        if not grid.free(tuple(start)):
            # a known zone has drifted onto the drone: get out first (no climbing),
            # then plan normally from clean air
            esc = P.escape_route(grid, start)
            if esc and len(esc) > 1:
                rest, algo = self._global_plan(esc[-1], targets)
                if rest.ok:
                    self._esc_len = len(esc) - 1
                    return P.PlanResult(esc + list(rest.path[1:]), rest.cost, rest.expansions, rest.ms, "OK"), "escape + " + algo
        if self.planner != "astar" and list(targets) == [self.home]:
            t0 = time.perf_counter()
            f = self.home_field()
            path = f.path(start)
            ms = (time.perf_counter() - t0) * 1e3
            if path is not None:
                return P.PlanResult(path, f.cost(start), 0, ms, "OK"), "Dijkstra cost field"
        informed = self.planner != "dijkstra"
        res, _ = P.plan_through(grid, start, targets, self.p["budget"], informed=informed)
        return res, ("A*" if informed else "Dijkstra")

    def truth_grid(self):
        return P.Grid(self.scn["cols"], self.scn["rows"], self.L, truth_all(self.scn, self.t), self.p,
                      wind=wind_tuple(self.scn), t=self.t)

    def log(self, level, code, text):
        self.events.append({"t": round(self.t, 2), "lvl": level, "code": code, "text": text})
        self.events = self.events[-200:]

    # ---------------------------------------------------------- inbound
    def on_message(self, name, obj, t):
        self.t = t
        self.last_rx = t
        self.last_rx_i = len(self.flown) - 1
        self.rx_times.append(t)
        if name == "HB":
            self.gcs_have = int(obj.get("bb", -1))
        elif name == "WX":
            self._apply_picture(obj, t)
        elif name == "MISSION":
            self._on_mission(obj, t)
        elif name == "CMD":
            self._on_cmd(obj.get("c"), t)

    def _apply_picture(self, pic, t):
        if not self.uplink_enabled:
            return
        v = int(pic.get("v", 0))
        if v <= self.pic_v:
            return
        before = set(self.uplink)
        self.uplink = {h["id"]: h for h in pic.get("hz", [])}
        self.pic_v = v
        new = set(self.uplink) - before
        if new:
            self.log("info", "PICTURE", f"Hazard picture v{v} received: {len(self.uplink)} volumes, new {', '.join(sorted(new))}")
        if self.state in FLYING:
            self._threat_check(full_route=True)

    def _on_mission(self, msg, t):
        rid = msg.get("rid")
        if msg.get("pic"):
            self._apply_picture(msg["pic"], t)
        route = [tuple(c) for c in msg.get("route", [])]
        targets = [tuple(c) for c in msg.get("targets", [])]
        grid = self.grid()
        probs = P.route_problems(grid, route, self.p, self.L)
        if not probs and route and tuple(route[0]) != self.cell:
            probs = [(0, "START_MISMATCH")]
        if not probs and targets and tuple(route[-1]) != targets[-1] and not msg.get("hold"):
            probs = [(len(route) - 1, "END_MISMATCH")]
        if not probs:
            need = grid.path_cost(route) * 0.5 * self.p["burn"]
            if need > self.energy - self.reserve():
                probs = [(0, "ENERGY_INSUFFICIENT")]
        if probs:
            i, code = probs[0]
            cell = list(route[i]) if route and i < len(route) else None
            why = code
            if code == "INSIDE_HAZARD" and cell:
                why = f"INSIDE_HAZARD {grid.blocker(tuple(cell)) or ''}".strip()
            self.m["rejected"] += 1
            self.log("warn", "REJECTED", f"Mission {rid} rejected: {why} at waypoint {i}")
            self.outbox.append(("rel", "RESULT", {"rid": rid, "ok": False, "reason": why, "i": i, "cell": cell}, None))
            return
        self.m["accepted"] += 1
        self.route, self.seg, self.progress = route, 0, 0.0
        self.targets, self.target_idx, self.rid = targets, 0, rid
        self.route_rev += 1
        self.log("ok", "ACCEPTED", f"Mission {rid} accepted: {len(route)} cells, {len(targets)} targets")
        if msg.get("hold") and route:
            self.hold = {"target": targets[-1], "fix": tuple(route[-1]), "mode": "ENROUTE", "t0": None,
                         "why": msg.get("hold_why") or "landing site blocked"}
            self.m["holds"] += 1
            self.log("warn", "HOLD_PLAN", f"Landing site {targets[-1]} is blocked ({self.hold['why']}): "
                     f"route ends at holding fix {tuple(route[-1])}")
        self.outbox.append(("rel", "RESULT", {"rid": rid, "ok": True, "rev": self.route_rev}, None))
        if msg.get("launch") and self.state in ("IDLE", "READY", "LANDED", "COMPLETE", "HOLD", "ENROUTE"):
            self.state = "ENROUTE"
            self.failsafe = None
            self.log("ok", "LAUNCH", "Launch: climbing out on uploaded route")

    def _on_cmd(self, c, t):
        if c == "RTL":
            self.m["rtl_cmds"] += 1
            if self.state in ("IDLE", "READY", "LANDED", "FORCED"):
                self.log("info", "RTL_IGNORED", f"Return to launch ignored: drone is {self.state.lower()}, not airborne")
                self.outbox.append(("rel", "RESULT", {"rid": "RTL", "ok": False, "reason": f"drone {self.state}"}, None))
                return
            if self.state == "RTL" and self.resume_state != "ENROUTE":
                self.log("info", "RTL", "Return to launch already in progress")
                return
            # Operator RTL overrides every automatic behaviour, including a lost-link
            # failsafe that would otherwise resume the mission when the link returns.
            self.failsafe, self.fs_policy = None, None
            self.hold_until = self.retry_at = None
            self.hold = None
            self._start_rtl("OPERATOR")
        elif c == "RESUME" and self.state in ("HOLD", "HOLDING", "RTLINK", "RTL", "REPLAN") and self.target_idx < len(self.targets):
            self.hold = None
            self.failsafe = None
            self.log("ok", "RESUME", "Operator resume: replanning to remaining targets")
            self._schedule_replan("ENROUTE", "RESUME", force=True)

    # ---------------------------------------------------------- helpers
    def reserve(self):
        return self.energy0 / max(1e-9, float(self.scn.get("soc", 1.0))) * self.p["reserve_pct"] / 100.0

    def flying(self):
        return self.state in AIRBORNE

    def _remaining_targets(self, mode):
        if mode == "RTL":
            return [self.home]
        if mode == "RTLINK":
            return [self.flown[self.last_good_i]]
        return self.targets[self.target_idx:]

    def _schedule_replan(self, resume, why, force=False):
        # A replan already pending keeps its goal, except that returning home
        # (or an explicit operator request) always wins. Round 1 dropped an RTL
        # pressed during a replan pause here.
        if self.state != "REPLAN" or force or resume == "RTL":
            self.resume_state = resume
        self.state = "REPLAN"
        self.replan_at = self.t + C.REPLAN_PAUSE_S
        self.replan_why = why

    def _execute_replan(self):
        mode = self.resume_state or "ENROUTE"
        targets = self._remaining_targets(mode)
        if not targets:
            self.state = "COMPLETE"
            return
        res, algo = self._global_plan(self.cell, targets)
        self.m["replan_ms"].append(round(res.ms, 3))
        if not res.ok and "GOAL_BLOCKED" in res.reason and self._plan_hold(mode, targets):
            return
        if res.ok:
            self.escape_end = self._esc_len if self._esc_len else -1
            self.route, self.seg, self.progress = res.path, 0, 0.0
            self.route_rev += 1
            self.m["replans"] += 1
            self.m["algo"][algo] = self.m["algo"].get(algo, 0) + 1
            self.last_algo = algo
            self.state = mode
            self.replan_at = None
            nodes = f"{res.expansions} nodes, " if res.expansions else ""
            self.log("ok", "REPLANNED", f"{algo}: new route ({self.replan_why}) {len(res.path)} cells, "
                     f"{nodes}{res.ms:.1f} ms")
            self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                                  "why": self.replan_why, "ms": round(res.ms, 2), "algo": algo}, "ROUTE"))
            return
        if res.reason.startswith("BUDGET"):
            self.m["budget_trips"] += 1
        self.log("warn", "NO_ROUTE", f"Replan failed ({res.reason}) for {mode}")
        if mode == "ENROUTE":
            self.failsafe, self.failsafe_reason = "NO_ROUTE", res.reason
            self.m["failsafes"] += 1
            self._start_rtl("NO_ROUTE")
        else:
            self.state = "HOLD"
            self.hold_until = None
            self.retry_at = self.t + 3.0
            self.resume_state = mode
            self.log("warn", "HOLDING", "Holding position: no safe route; retrying every 3 s")

    # ---------------------------------------------------------- holding pattern
    def _plan_hold(self, mode, targets):
        """Landing site (or a waypoint) covered by a zone that blocks this drone:
        fly to a nearby clear holding fix and orbit until it clears."""
        grid = self.grid()
        tgt = next((tuple(t) for t in targets if not grid.free(tuple(t))), None)
        if tgt is None:
            return False
        fix = P.nearest_free(grid, tgt)
        if fix is None:
            return False
        before = [tuple(t) for t in targets[:[tuple(t) for t in targets].index(tgt)]]
        res, algo = self._global_plan(self.cell, before + [fix])       # visit waypoints on the way, then hold
        if not res.ok:
            return False
        why = grid.blocker(tgt) or "weather"
        self.hold = {"target": tgt, "fix": fix, "mode": mode, "t0": None, "why": why}
        self._hold_pattern(grid)
        self.escape_end = self._esc_len if self._esc_len else -1
        self.route, self.seg, self.progress = res.path, 0, 0.0
        self.route_rev += 1
        self.state, self.replan_at = mode, None
        self.m["holds"] += 1
        where = "launch point" if tgt == self.home else ("goal" if self.targets and tgt == tuple(self.targets[-1]) else "waypoint")
        self.log("warn", "HOLD_PLAN", f"{why} covers the {where} {tgt}: flying to holding fix {fix} ({algo})")
        self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                              "why": f"hold for {why}", "algo": algo, "hold": self._hold_view()}, "ROUTE"))
        return True

    def _hold_pattern(self, grid=None):
        """Build the holding pattern around the fix (racetrack / figure-8 / orbit)."""
        h = self.hold
        grid = grid or self.grid()
        kind, pts, hdg, scale = HOLD.choose(grid, h["fix"], h["target"], self.p.get("hold_pattern", "racetrack"),
                                            self.scn["mode"])
        h.update(kind=kind, pts=pts, cum=HOLD.arc_lengths(pts), s=0.0, hdg=hdg, scale=scale)

    def _hold_view(self):
        h = self.hold
        if not h:
            return None
        return {"fix": list(h["fix"]), "target": list(h["target"]), "why": h["why"], "kind": h.get("kind"),
                "pts": [[round(x, 2), round(y, 2)] for x, y in h.get("pts", [])], "hdg": h.get("hdg")}

    def _holding(self, dt):
        h = self.hold
        if h["t0"] is None:
            h["t0"] = self.t
            if "pts" not in h:
                self._hold_pattern()
            self.log("warn", "HOLD_ENTRY", f"Entering hold: {HOLD.KINDS.get(h['kind'], h['kind'])} at {tuple(h['fix'])}, "
                     f"inbound {h['hdg']:03d}°, one lap {h['cum'][-1] * C.MODES[self.scn['mode']]['cell_m'] / 1000:.1f} km")
        self.energy -= self.p["loiter_burn"] * dt * self.power_factor()
        self.m["hold_s"] += dt
        # fly the pattern at cruise speed (worn motors fly it slower)
        h["s"] += self.p["speed"] * self.thrust_factor() * dt
        x, y = HOLD.point_at(h["pts"], h["cum"], h["s"])
        fz = h["fix"][2]
        self.pos = [x, y, float(fz)]
        c = (int(round(x)), int(round(y)), fz)
        if c != self.cell and self.grid().inside(c):
            self.cell = c
            self.flown.append(c)
        if self.t < h.get("next_check", 0.0):
            return
        h["next_check"] = self.t + 0.5
        grid = self.grid()
        if grid.free(h["target"]):
            mode = h["mode"]
            skipped = []
            if mode == "ENROUTE" and tuple(h["target"]) in [tuple(t) for t in self.targets]:
                j = [tuple(t) for t in self.targets].index(tuple(h["target"]))
                skipped = [f"WP{i + 1}" for i in range(self.target_idx, j)]
                self.target_idx = max(self.target_idx, j)      # straight on to the end point, no going back
            self.log("ok", "HOLD_CLEAR", f"Landing site {h['target']} is clear after {self.t - h['t0']:.1f} s in the hold: "
                     f"leaving the pattern straight for it" + (f" (skipping {', '.join(skipped)})" if skipped else ""))
            self.hold = None
            self._schedule_replan(mode, "HOLD_CLEAR", force=True)
            return
        home_cost = self.cost_home() if h["target"] != self.home else None
        need = (home_cost or 0.0) * 0.5 * self.p["burn"] * self.power_factor()
        can_home = home_cost is not None and self.energy - need >= self.reserve() + 1.0
        if h.get("home_ok") is None:
            h["home_ok"] = can_home              # could it still get home when the hold began?
        out_of_time = self.t - h["t0"] > 30.0
        if h["home_ok"] and (not can_home or out_of_time):
            # point of no return for going home: leave the hold now
            why = "hold time limit" if out_of_time else "energy: last chance to get home"
            self.hold = None
            self.failsafe, self.failsafe_reason = "HOLD_EXIT", why
            self.log("warn", "HOLD_EXIT", f"Leaving the hold ({why}): returning to launch")
            self._start_rtl("HOLD_EXIT")
        elif not h["home_ok"] and (self.energy <= self.reserve() + 0.5 or out_of_time):
            # home was never reachable from here: land at the clear holding fix instead
            self.hold = None
            self.state = "LANDED"
            self.cell = tuple(h["fix"])
            self.pos = [float(c) for c in self.cell]
            self.log("crit", "DIVERT", f"Landing site still blocked and home out of reach: landed at the holding fix {self.cell}")

    def _start_rtl(self, reason):
        self.failsafe = self.failsafe or reason
        self.failsafe_reason = reason
        self.log("warn", "RTL", f"Return to launch ({reason})")
        self._schedule_replan("RTL", reason)

    def _threat_check(self, full_route=False, sensor_ids=()):
        grid = self.grid()
        i = P.first_blocked(grid, self.route, max(self.seg + 1, self.escape_end + 1), None)
        if i is None:
            return False
        hid = grid.blocker(tuple(self.route[i])) or "ceiling"
        dist = i - self.seg
        if hid not in self.detect:
            self.detect[hid] = dist
            self.m["detect_cells"][hid] = dist
        mode = self.state if self.state in FLYING else (self.resume_state or "ENROUTE")
        # Something the drone's OWN sensor just found close ahead: react locally
        # at once (BendyRuler), then hand back to the global planner. Threats
        # that arrive by uplink are far away and go to the global planner.
        if hid in sensor_ids and self.planner != "astar" and self.state in FLYING and self._bendy(mode, hid, dist):
            return True
        self.log("warn", "THREAT", f"{hid} blocks route {dist} cells ahead: replanning")
        self._schedule_replan(mode, f"{hid} ahead")
        return True

    def _bendy(self, mode, hid, dist):
        targets = self._remaining_targets(mode)
        if not targets:
            return False
        grid = self.grid()
        length = max(int(self.p.get("br_ruler", 3)), min(dist + 1, 6))
        local, info = P.bendy_ruler(grid, self.cell, targets[0], length=length)
        self.m["bendy_ms"].append(round(info["ms"], 3))
        if not local:
            self.log("warn", "BENDYRULER", f"BendyRuler found no clear ruler ({info['probes']} probes): full replan")
            return False
        rest, algo = self._global_plan(local[-1], targets)
        if not rest.ok:
            self.log("warn", "BENDYRULER", f"Ruler {info['choice']} leads nowhere ({rest.reason}): full replan")
            return False
        self.route = [self.cell] + list(local) + list(rest.path[1:])
        self.seg, self.progress = 0, 0.0
        self.route_rev += 1
        self.m["replans"] += 1
        self.m["bendy"] += 1
        self.m["replan_ms"].append(round(info["ms"] + rest.ms, 3))
        key = f"BendyRuler + {algo}"
        self.m["algo"][key] = self.m["algo"].get(key, 0) + 1
        self.last_algo = key
        self.log("warn", "BENDYRULER", f"{hid} {dist} cells ahead: BendyRuler takes {info['choice']} at once "
                 f"({info['probes']} probes, {info['ms']:.2f} ms), {algo} rejoins the route ({rest.ms:.1f} ms)")
        self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                              "why": f"{hid} (OPC/SO2)", "ms": round(info["ms"] + rest.ms, 2),
                                              "algo": key}, "ROUTE"))
        return True

    # ---------------------------------------------------------- in-situ sensing
    def _sample(self, dt):
        """OPC + SO2 model. Both sensors measure the air the drone is flying through,
        so neither can see ash ahead: the OPC sees particles within one histogram
        period, the SO2 cell follows the true value with a first-order lag
        (t90 from the datasheet). Lags are converted from real seconds to demo
        seconds with the mode's time scale, so they stay physically in proportion."""
        p = self.p
        c, hid = ash_concentration(self.scn, self.pos, self.t)
        self.c_true, self.c_hid = c, hid
        tau = max(1e-3, float(p.get("so2_t90_s", 60.0)) / 2.303 / self.ts)
        self.so2 += (c * C.SO2_PPM_PER_MG - self.so2) * (1.0 - math.exp(-dt / tau))
        if self.t < self.next_opc:
            return None
        self.next_opc = self.t + float(p.get("opc_period_s", 1.0)) / self.ts
        bias = min(0.1, 0.1 * self.dose / self.dose_full)          # optics fouling reads slightly high
        r = max(0.0, c * (1.0 + self.rng.gauss(0.0, 0.08)) + bias)
        self.opc_sat = r >= C.OPC_SAT_MG
        self.opc = min(r, C.OPC_SAT_MG)
        thr = float(p.get("opc_thr_mg", 0.2))
        self.opc_hits = self.opc_hits + 1 if self.opc >= thr else 0
        if self.opc >= thr * 0.5:
            self.ash_samples.append((round(self.pos[0], 2), round(self.pos[1], 2), round(self.t, 2), round(self.opc, 2)))
            self.ash_samples = self.ash_samples[-80:]
        if self.opc < thr * 0.5 and self.in_ash and not self._escaping():
            self.log("ok", "ASH_CLEAR", f"OPC back to {self.opc:.2f} mg/m³: clear of ash {self.in_ash}")
            self.in_ash = None
        if hid and self.so2 >= 0.1 and hid not in self.so2_confirmed and hid in self.encounters:
            self.so2_confirmed.add(hid)
            self.log("info", "SO2", f"SO₂ {self.so2:.2f} ppm confirms volcanic origin of {hid} "
                     f"(electrochemical cell lags: t90 ≤ {p.get('so2_t90_s', 60):.0f} s real)")
        if self.opc_hits >= 2 and hid and self.in_ash is None:     # two samples in a row: debounce
            return hid
        return None

    def _escaping(self):
        return self.escape_end >= 0 and self.seg < self.escape_end

    def _ash_detected(self, hid):
        """React to ash measured on board. Edge (low reading): mark the plume as
        lying ahead and veer away. Core (high reading): the drone is inside it;
        180° turn back along its own track to the last air it sampled clean,
        never climbing, then plan around the plume from there."""
        n = self.encounters.get(hid, 0)
        self.encounters[hid] = n + 1
        self.in_ash = hid
        inside = self.opc >= C.ASH_EDGE_MG
        if not inside and hid in self.uplink:
            # the edge of a plume the drone already knows about: confirm, no new action
            self.sensor_hits.append({"id": hid, "t": round(self.t, 2), "k": 0, "mg": round(self.opc, 2), "inside": False})
            self.sensor_hits = self.sensor_hits[-6:]
            self.log("info", "ASH_EDGE", f"OPC {self.opc:.2f} mg/m³: edge of known plume {hid} (advisory confirmed)")
            return
        hd = self._heading()
        z = self.cell[2]
        ahead = 0.4 if inside else 2.1
        r = (1.7 if inside else 1.5) + 0.8 * n                      # grows with every re-encounter
        z0 = 0 if self.scn["mode"] == "small" else max(0, z - 2)
        # trajectory: ash moves with the wind. The drone measures the wind itself
        # (ground-speed vs airspeed triangle), with a few degrees / knots of error,
        # and projects the plume along it.
        wdir, wkt = self._measured_wind()
        drift = float((self.scn.get("env") or {}).get("drift", 0.5))
        vx, vy = drift_velocity(self.scn["mode"], {"dir": wdir, "kt": wkt}, drift)
        self.wind_meas = [round(wdir), round(wkt, 1), vx, vy]
        cx, cy = self.pos[0] + hd[0] * ahead, self.pos[1] + hd[1] * ahead
        samples = [s_ for s_ in self.ash_samples if s_[3] >= float(self.p.get("opc_thr_mg", 0.2))]
        if len(samples) >= 3:                                  # centre on what was actually measured, advected to now
            mx = sum(s_[0] + vx * (self.t - s_[2]) for s_ in samples[-12:]) / len(samples[-12:])
            my = sum(s_[1] + vy * (self.t - s_[2]) for s_ in samples[-12:]) / len(samples[-12:])
            cx, cy = (cx + mx) / 2.0, (cy + my) / 2.0
        est = circle(hid, "ash", cx, cy, r, z0, min(self.L - 1, z + 2),
                     sev=3, conc_mg=round(self.opc, 2), source="DRONE OPC/SO2", estimated=True, t_on=round(self.t, 2),
                     t_ref=round(self.t, 2), vx=vx, vy=vy, wind=[round(wdir), round(wkt, 1)])
        self.sensed[hid] = est
        self._sensed_rev += 1
        self.m["sensed"] += 1
        self.sensor_hits.append({"id": hid, "t": round(self.t, 2), "k": 0, "mg": round(self.opc, 2), "inside": inside})
        self.sensor_hits = self.sensor_hits[-6:]
        self.outbox.append(("rel", "HZR", {"hz": est, "t": round(self.t, 2), "opc": round(self.opc, 2),
                                           "so2": round(self.so2, 3)}, f"HZR{hid}"))
        sat = " (saturated)" if self.opc_sat else ""
        self.log("info", "ASH_TRACK", f"Wind measured {wdir:03.0f}°/{wkt:.0f} kt: plume {hid} projected to drift "
                 f"{math.hypot(vx, vy) * C.MODES[self.scn['mode']]['cell_m'] / C.MODES[self.scn['mode']]['time_scale'] * 1.944:.0f} kt "
                 f"toward {(math.degrees(math.atan2(vx, -vy)) % 360):03.0f}°")
        if inside:
            self.log("crit", "ASH_ENCOUNTER", f"IN VOLCANIC ASH {hid}: OPC {self.opc:.1f} mg/m³{sat}, SO₂ {self.so2:.2f} ppm. "
                     f"Escaping: 180° turn back to clean air, no climbing")
            self._escape(hid)
        else:
            self.log("warn", "ASH_EDGE", f"OPC {self.opc:.2f} mg/m³: edge of ash {hid}; plume assumed ahead (r {r:.1f} cells)")
            self._threat_check(sensor_ids={hid})

    def _measured_wind(self):
        w = self.scn["wind"]
        return ((w["dir"] + self.rng.gauss(0, 4.0)) % 360, max(0.0, w["kt"] * (1 + self.rng.gauss(0, 0.06))))

    def _heading(self):
        r, i = self.route, self.seg
        for k in range(max(0, i), len(r) - 1):
            a, b = r[k], r[k + 1]
            if a[0] != b[0] or a[1] != b[1]:
                d = math.hypot(b[0] - a[0], b[1] - a[1])
                return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
        if len(self.flown) > 1:
            a, b = self.flown[-2], self.flown[-1]
            d = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
            return ((b[0] - a[0]) / d, (b[1] - a[1]) / d)
        return (1.0, 0.0)

    def _escape(self, hid):
        mode = self.state if self.state in FLYING else (self.resume_state or "ENROUTE")
        if mode not in FLYING:
            mode = "ENROUTE"
        grid = self.grid()
        # walk back along the flown track to the first cell outside the plume estimate
        back = [self.cell]
        for c in reversed(self.flown[:-1]):
            c = tuple(c)
            if c == back[-1]:
                continue
            if c[2] > back[-1][2]:
                continue                                    # never climb out of ash
            back.append(c)
            if grid.free(c) or len(back) > 12:
                break
        how = "180° turn back along own track"
        if not grid.free(back[-1]):
            esc = P.escape_route(grid, self.cell)
            if esc:
                back, how = esc, "shortest way out (no climb)"
        targets = self._remaining_targets(mode)
        rest, algo = self._global_plan(back[-1], targets) if targets else (None, "")
        route = list(back) + (list(rest.path[1:]) if rest is not None and rest.ok else [])
        self.route, self.seg, self.progress = route, 0, 0.0
        self.escape_end = len(back) - 1
        self.route_rev += 1
        self.m["escapes"] += 1
        self.state = mode if mode in FLYING else "ENROUTE"
        self.replan_at = None
        key = "escape + " + (algo or "hold")
        self.m["algo"][key] = self.m["algo"].get(key, 0) + 1
        self.last_algo = key
        self.log("warn", "ESCAPE", f"{how}: {len(back) - 1} cells to clean air at {back[-1]}"
                 + (f", then {algo} to the mission" if rest is not None and rest.ok else ", then replan"))
        self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                              "why": f"escape {hid}", "algo": key}, "ROUTE"))

    # ---------------------------------------------------------- ash damage (illustrative model)
    def power_factor(self):
        """Extra power draw: worn motors / engines plus flying in ash."""
        return 1.0 + 0.6 * (1.0 - self.health) + 0.08 * min(self.c_true, 8.0)

    def thrust_factor(self):
        return 0.55 + 0.45 * self.health

    def _degrade(self, dt):
        if self.c_true <= 0.05:
            return
        if self.c_true >= 0.2:
            self.m["ash_s"] += dt
        self.dose += self.c_true * dt * self.ts
        self.health = max(0.0, 1.0 - self.dose / self.dose_full)
        fixed_wing = self.scn["mode"] == "aircraft" or "VTOL" in str(self.p.get("name", ""))
        if fixed_wing and "AIRSPEED" not in self.flags and self.dose > 0.15 * self.dose_full:
            self.flags.add("AIRSPEED")
            self.log("warn", "AIRSPEED", "Pitot eroded by ash: airspeed unreliable, flying on ground speed")
        if self.health < 0.6 and "ASH_DAMAGE" not in self.flags:
            self.flags.add("ASH_DAMAGE")
            self.log("crit", "ASH_DAMAGE", f"Ash wear: {'engine' if self.scn['mode'] == 'aircraft' else 'motors'} at "
                     f"{self.health * 100:.0f} %, thrust down: mission aborted, returning to launch")
            self.abort_pending = "ASH_DAMAGE"
        if self.health < 0.25 and self.state not in ("FORCED", "LANDED"):
            self.state = "FORCED"
            self.log("crit", "FORCED_LANDING", f"Ash damage critical ({self.health * 100:.0f} %): emergency landing")

    def _enter_cell(self, prev, cell):
        grid = self.grid()
        if not self.grid_now().free(cell) and not (self.escape_end >= 0 and self.seg <= self.escape_end):
            self.m["blocked_entries"] += 1
            self.log("crit", "INVARIANT", f"Entered known-blocked cell {cell}")
        tg = self.truth_grid()
        if not tg.free(cell):
            self.m["exposures"] += 1
            self.log("crit", "EXPOSURE", f"Flew into {tg.blocker(cell)} at {cell} (not known in time)")
        cost = grid.move_cost(prev, cell)
        self.m["path_cost"] += cost
        self.energy -= cost * 0.5 * self.p["burn"] * self.power_factor()
        self.m["cells"] += 1
        self.m["dist"] += P.step_length(prev, cell)
        self.cell = cell
        self.flown.append(cell)
        if self.opc < float(self.p.get("opc_thr_mg", 0.2)) * 0.5 and self.in_ash is None:
            self.clean_i = len(self.flown) - 1
        if self.escape_end >= 0 and self.seg >= self.escape_end:
            self.escape_end = -1
            self.log("ok", "ESCAPED", f"Out of the plume estimate at {cell}; OPC {self.opc:.2f} mg/m³, "
                     f"{'motors' if self.scn['mode'] == 'small' else 'engine'} {self.health * 100:.0f} %")
            if self.abort_pending:
                why, self.abort_pending = self.abort_pending, None
                self.failsafe, self.failsafe_reason = why, why
                self.m["failsafes"] += 1
                self._start_rtl(why)
        if self.link == "CONNECTED":
            self.last_good_i = len(self.flown) - 1

    # ---------------------------------------------------------- main step
    def step(self, dt, t):
        self.t = t
        self._link_watchdog()
        st = self.state
        if st in ("IDLE", "READY", "LANDED", "COMPLETE", "FORCED"):
            self._periodic()
            return
        if st in AIRBORNE:
            hid = self._sample(dt)
            self._degrade(dt)
            if self.state == "FORCED":
                self._periodic()
                return
            if hid:
                self._ash_detected(hid)
                st = self.state
            elif (self.abort_pending and not self._escaping() and self.state in FLYING
                  and self.state != "RTL"):
                why, self.abort_pending = self.abort_pending, None
                self.failsafe, self.failsafe_reason = why, why
                self.m["failsafes"] += 1
                self._start_rtl(why)
                st = self.state
        if st in FLYING and t >= getattr(self, "_next_sweep", 0.0) and self._moving_known():
            self._next_sweep = t + 0.5                    # drifting zones: re-check the route twice a second
            if self._threat_check():
                st = self.state
        if st == "HOLDING":
            self._holding(dt)
        elif st == "REPLAN":
            self.energy -= self.p["loiter_burn"] * dt * self.power_factor()
            if self.replan_at is not None and t >= self.replan_at:
                self._execute_replan()
        elif st == "HOLD":
            self.energy -= self.p["loiter_burn"] * dt * self.power_factor()
            if self.resume_state != "RTL" and self._hold_bingo(t):
                self.hold_until = self.retry_at = None
                self.failsafe, self.failsafe_reason = "BINGO", "energy while holding"
                self._start_rtl("BINGO")
            elif self.retry_at is not None and t >= self.retry_at:
                self.retry_at = None
                self._schedule_replan(self.resume_state or "RTL", "RETRY")
            elif self.hold_until is not None and t >= self.hold_until:
                self.hold_until = None
                if self.failsafe == "LOST_LINK":
                    self.log("warn", "RTL", "No link while holding: heading home (will resume if the link returns)")
                    self._schedule_replan("RTL", "LINK_SEARCH")
                else:
                    self._start_rtl("HOLD_TIMEOUT")
        elif st in FLYING:
            self._advance(dt)
        if self.energy <= 0 and self.state not in ("LANDED", "COMPLETE", "FORCED"):
            self.energy = 0.0
            self.state = "FORCED"
            self.log("crit", "FORCED_LANDING", "Energy exhausted: forced landing")
        self._periodic()

    def _advance(self, dt):
        if self.seg >= len(self.route) - 1:
            self._arrive()
            return
        a, b = tuple(self.route[self.seg]), tuple(self.route[self.seg + 1])
        self.progress += self.p["speed"] * self.thrust_factor() * dt / P.step_length(a, b)
        while self.progress >= 1.0 and self.state in FLYING:
            self.progress -= 1.0
            prev = tuple(self.route[self.seg])
            self.seg += 1
            cell = tuple(self.route[self.seg])
            self._enter_cell(prev, cell)
            if self._on_cell():
                self.progress = 0.0
                break
            if self.seg >= len(self.route) - 1:
                self._arrive()
                self.progress = 0.0
                break
        if self.state in FLYING and self.seg < len(self.route) - 1:
            self.pos = self._curve_pos(self.seg, min(1.0, self.progress))
        else:
            self.pos = [float(self.cell[0]), float(self.cell[1]), float(self.cell[2])]

    def _curve_pos(self, i, f):
        """Position on the route drawn as a smooth curve: each corner is rounded
        with a quadratic Bézier from the middle of one leg to the middle of the
        next (control point = the corner cell). The curve stays inside the corner
        cell, and A* never cuts corners past a blocked cell, so the rounding
        cannot clip an obstacle."""
        r = self.route
        a, b = r[i], r[i + 1]
        if f < 0.5 and i > 0:
            p0, c, p1, u = _mid(r[i - 1], a), a, _mid(a, b), 0.5 + f
        elif f >= 0.5 and i + 2 < len(r):
            p0, c, p1, u = _mid(a, b), b, _mid(b, r[i + 2]), f - 0.5
        else:
            return [a[k] + (b[k] - a[k]) * f for k in range(3)]
        w0, w1, w2 = (1 - u) ** 2, 2 * (1 - u) * u, u * u
        return [w0 * p0[k] + w1 * c[k] + w2 * p1[k] for k in range(3)]

    def _on_cell(self):
        """Called after entering a cell. Returns True if movement must stop this frame."""
        if self.state == "ENROUTE" and self.target_idx < len(self.targets) and self.cell == self.targets[self.target_idx]:
            self.target_idx += 1
            self.fs_count = 0
            if self.target_idx < len(self.targets):
                self.log("ok", "WAYPOINT", f"Waypoint {self.target_idx} reached")
        if self._threat_check():
            return True
        if self.state == "ENROUTE" and self.target_idx < len(self.targets):
            g = self.grid()
            finish = g.path_cost(self.route[self.seg:]) * 0.5 * self.p["burn"]
            if self.energy - finish >= self.reserve():
                return False                       # can still finish with reserve: carry on
            home_cost = self.cost_home()
            if home_cost is not None:
                need = home_cost * 0.5 * self.p["burn"]
                if self.energy - need < self.reserve() + 1.0:
                    self.failsafe, self.failsafe_reason = "BINGO", "energy"
                    self.m["failsafes"] += 1
                    self.log("warn", "BINGO", f"Bingo energy: {self.energy:.1f} left, {need:.1f} needed to get home "
                             f"+ {self.reserve():.1f} reserve")
                    self._start_rtl("BINGO")
                    return True
        return False

    def _arrive(self):
        if self.hold and self.cell == tuple(self.hold["fix"]):
            self.state = "HOLDING"
            self.hold["t0"] = None
            self.log("warn", "HOLDING", f"Holding pattern at {self.cell}: waiting for {self.hold['why']} to clear the "
                     f"{'launch point' if self.hold['target'] == self.home else 'landing site'}")
            return
        if self.state == "ENROUTE":
            if self.target_idx >= len(self.targets) or self.cell == self.targets[-1]:
                self.target_idx = len(self.targets)
                self.state = "COMPLETE"
                self.log("ok", "COMPLETE", "Mission complete: goal reached")
        elif self.state == "RTL":
            self.state = "LANDED"
            self.log("ok", "LANDED", "Landed at launch point")
        elif self.state == "RTLINK":
            if self.link == "CONNECTED":
                self._resume_after_link()
            else:
                self.state = "HOLD"
                self.hold_until = self.t + 5.0
                self.resume_state = "ENROUTE"
                self.log("warn", "WAIT_LINK", "At last good-link point: holding 5 s for link, then heading home")

    # ---------------------------------------------------------- link watchdog & failsafe
    def _link_watchdog(self):
        if self.last_rx is None:
            new = "NO_LINK"
        else:
            age = self.t - self.last_rx
            new = "CONNECTED" if age < C.LINK_DEGRADED_S else ("DEGRADED" if age < C.LINK_LOST_S else "LOST")
        if new != self.link:
            if new == "LOST":
                self.log("crit", "LINK_LOST", "Uplink silent: drone is on its own")
                self.last_loss_cell = self.cell
            elif new == "CONNECTED" and self.link in ("LOST", "DEGRADED"):
                self.log("ok", "LINK_BACK", "Uplink restored")
            self.link = new
            self.link_ok_since = self.t if new == "CONNECTED" else None
        if self.last_rx is None:
            return
        age = self.t - self.last_rx
        if (self.failsafe is None and age > self.p["lost_link_s"]
                and self.state in FLYING + ("REPLAN",) and self.state != "RTL"):
            self._lost_link_failsafe(age)
        if self.failsafe == "LOST_LINK" and self.link_healthy():
            pol = self.fs_policy
            if pol in ("RETURN_TO_LINK", "HOLD") and self.state in ("RTLINK", "HOLD", "REPLAN", "RTL"):
                self._resume_after_link()
            elif pol == "CONTINUE":
                self.failsafe = None
                self.log("ok", "LINK_BACK", "Link healthy again: ground station back in the loop")

    def link_healthy(self):
        """At least 3 ground-station packets in the last 2.5 s (heartbeat is 2 Hz)."""
        return sum(1 for x in self.rx_times if self.t - x <= 2.5) >= 3

    def _lost_link_failsafe(self, age):
        pol = self.p["failsafe"]
        self.m["failsafes"] += 1
        self.fs_count += 1
        if pol == "RETURN_TO_LINK" and self.fs_count >= 2:
            self.log("warn", "FAILSAFE", "Second link loss on this leg: continuing autonomously on last known map")
            pol = "CONTINUE"
        self.failsafe, self.failsafe_reason, self.fs_policy = "LOST_LINK", pol, pol
        self.log("crit", "FAILSAFE", f"Lost link {age:.1f} s: policy {pol}")
        if pol == "CONTINUE":
            return
        if pol == "RTL":
            self._start_rtl("LOST_LINK")
        elif pol == "HOLD":
            self.state = "HOLD"
            self.resume_state = "ENROUTE"
            self.hold_until = self.t + 8.0
            self.log("warn", "HOLDING", "Holding 8 s for link, then RTL")
        elif pol == "RETURN_TO_LINK":
            self.last_good_i = max(0, min(self.last_rx_i, len(self.flown) - 1) - 1)
            if self.last_good_i >= len(self.flown) - 1:
                self.state = "HOLD"
                self.resume_state = "ENROUTE"
                self.hold_until = self.t + 5.0
                return
            back = list(reversed(self.flown[self.last_good_i:]))
            dedup = [back[0]]
            for c in back[1:]:
                if c != dedup[-1]:
                    dedup.append(c)
            grid = self.grid()
            if all(grid.free(c) for c in dedup[1:]) and not P.route_problems(grid, dedup, self.p, self.L):
                self.route, self.seg, self.progress = dedup, 0, 0.0
                self.route_rev += 1
                self.state = "RTLINK"
                self.log("warn", "RTLINK", f"Retracing {len(dedup) - 1} cells to last good link")
                self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                                      "why": "RETURN_TO_LINK"}, "ROUTE"))
            else:
                self._schedule_replan("RTLINK", "RETURN_TO_LINK")

    def _hold_bingo(self, t):
        if t < self._bingo_check_t:
            return False
        self._bingo_check_t = t + 1.0
        c = self.cost_home()
        need = (c or 0.0) * 0.5 * self.p["burn"]
        return self.energy - need < self.reserve() + 1.0

    def _resume_after_link(self):
        self.failsafe = None
        if self.target_idx >= len(self.targets):
            self._start_rtl("DONE")
            return
        self.log("ok", "RESUME", "Link healthy: resuming mission")
        self._schedule_replan("ENROUTE", "RESUME")

    # ---------------------------------------------------------- periodic output
    def _periodic(self):
        t = self.t
        if t >= self.next_bb:
            self.next_bb = t + C.BLACKBOX_PERIOD_S
            self.bb.append([len(self.bb), round(t, 2), round(self.pos[0], 2), round(self.pos[1], 2),
                            round(self.pos[2], 2), self.state, self.link, round(self.energy, 1)])
        if t >= self.next_tm:
            self.next_tm = t + 1.0 / C.TELEMETRY_HZ
            self.tm_seq += 1
            self.outbox.append(("unrel", "TM", self.telemetry(), None))
        if t >= self.next_bb_send and self.link in ("CONNECTED", "DEGRADED"):
            self.next_bb_send = t + 0.25
            start = self.gcs_have + 1
            if start < len(self.bb):
                chunk = self.bb[start:start + 24]
                self.outbox.append(("unrel", "BB", {"from": start, "r": chunk, "n": len(self.bb)}, None))

    def telemetry(self):
        return {"s": self.tm_seq, "t": round(self.t, 2), "p": [round(v, 2) for v in self.pos],
                "c": list(self.cell), "st": self.state, "e": round(self.energy, 1), "lk": self.link,
                "v": self.pic_v, "rr": self.route_rev, "ti": self.target_idx, "rp": self.m["replans"],
                "fs": self.failsafe, "sp": round(self.p["speed"] * self.thrust_factor(), 2), "seg": self.seg, "bbn": len(self.bb),
                "opc": round(self.opc, 2), "so2": round(self.so2, 3), "hp": round(self.health * 100, 1),
                "hold": list(self.hold["fix"]) if self.hold else None}

    def observer(self):
        """Full onboard state for the right-hand screen (simulation channel, not radio)."""
        return {
            "t": round(self.t, 2), "state": self.state, "pos": [round(v, 3) for v in self.pos], "cell": list(self.cell),
            "route": [list(c) for c in self.route], "seg": self.seg, "rev": self.route_rev,
            "targets": [list(c) for c in self.targets], "ti": self.target_idx, "energy": round(self.energy, 2),
            "energy0": round(self.energy0, 2), "reserve": round(self.reserve(), 2),
            "link": self.link, "link_age": None if self.last_rx is None else round(self.t - self.last_rx, 2),
            "failsafe": self.failsafe, "fs_reason": self.failsafe_reason, "pic_v": self.pic_v,
            "known": sorted(set(self.uplink) | set(self.sensed)), "sensed": sorted(self.sensed),
            "sensed_hz": list(self.sensed.values()),
            "flown": [list(c) for c in self.flown[-400:]], "events": self.events[-30:],
            "metrics": {k: (v if k != "replan_ms" else v[-20:]) for k, v in self.m.items()},
            "bb_n": len(self.bb), "lookahead": self.p["lookahead"], "uplink_enabled": self.uplink_enabled,
            "sensor_hits": self.sensor_hits, "algo": self.last_algo, "planner": self.planner,
            "sar": bool(self.p.get("sar")), "resume_state": self.resume_state,
            "sensors": {"opc": round(self.opc, 3), "opc_sat": self.opc_sat, "so2": round(self.so2, 3),
                        "c_true": round(self.c_true, 3), "thr": self.p.get("opc_thr_mg", 0.2),
                        "opc_period_s": self.p.get("opc_period_s", 1.0), "so2_t90_s": self.p.get("so2_t90_s", 60.0),
                        "in_ash": self.in_ash, "time_scale": self.ts},
            "health": round(self.health * 100, 1), "dose": round(self.dose, 1), "flags": sorted(self.flags),
            "fouling": round(min(100.0, 100 * self.dose / self.dose_full * 1.5), 1),
            "hold": self._hold_view(),
            "escaping": self._escaping(), "escape_end": self.escape_end,
            "samples": self.ash_samples[-60:], "wind_meas": self.wind_meas,
        }
