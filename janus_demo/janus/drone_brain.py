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
from collections import deque

from . import config as C
from . import planner as P
from .world import truth_all, truth_hazards, wind_tuple

FLYING = ("ENROUTE", "RTL", "RTLINK")


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
        self.m = {"replans": 0, "replan_ms": [], "blocked_entries": 0, "exposures": 0, "sensed": 0,
                  "cells": 0, "dist": 0.0, "detect_cells": {}, "rejected": 0, "accepted": 0,
                  "failsafes": 0, "budget_trips": 0, "path_cost": 0.0}

    # ---------------------------------------------------------- knowledge
    def known(self):
        hz = {h["id"]: h for h in self.scn["static"]}
        hz.update(self.sensed)
        hz.update(self.uplink)
        return list(hz.values())

    def grid(self):
        key = (self.pic_v, len(self.sensed), tuple(sorted(self.uplink)))
        if self._grid_cache is None or self._grid_cache[0] != key:
            g = P.Grid(self.scn["cols"], self.scn["rows"], self.L, self.known(), self.p, wind=wind_tuple(self.scn))
            self._grid_cache = (key, g)
        return self._grid_cache[1]

    def truth_grid(self):
        return P.Grid(self.scn["cols"], self.scn["rows"], self.L, truth_all(self.scn, self.t), self.p,
                      wind=wind_tuple(self.scn))

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
        if not probs and targets and tuple(route[-1]) != targets[-1]:
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
        self.outbox.append(("rel", "RESULT", {"rid": rid, "ok": True, "rev": self.route_rev}, None))
        if msg.get("launch") and self.state in ("IDLE", "READY", "LANDED", "COMPLETE", "HOLD", "ENROUTE"):
            self.state = "ENROUTE"
            self.failsafe = None
            self.log("ok", "LAUNCH", "Launch: climbing out on uploaded route")

    def _on_cmd(self, c, t):
        if c == "RTL" and self.state not in ("IDLE", "LANDED", "COMPLETE", "FORCED"):
            self._start_rtl("OPERATOR")
        elif c == "RESUME" and self.state in ("HOLD", "RTLINK", "RTL") and self.target_idx < len(self.targets):
            self.failsafe = None
            self.log("ok", "RESUME", "Operator resume: replanning to remaining targets")
            self._schedule_replan("ENROUTE", "RESUME")

    # ---------------------------------------------------------- helpers
    def reserve(self):
        return self.energy0 / max(1e-9, float(self.scn.get("soc", 1.0))) * self.p["reserve_pct"] / 100.0

    def flying(self):
        return self.state in FLYING or self.state in ("REPLAN", "HOLD")

    def _remaining_targets(self, mode):
        if mode == "RTL":
            return [self.home]
        if mode == "RTLINK":
            return [self.flown[self.last_good_i]]
        return self.targets[self.target_idx:]

    def _schedule_replan(self, resume, why):
        if self.state != "REPLAN":
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
        grid = self.grid()
        res, _ = P.plan_through(grid, self.cell, targets, self.p["budget"])
        self.m["replan_ms"].append(round(res.ms, 3))
        if res.ok:
            self.route, self.seg, self.progress = res.path, 0, 0.0
            self.route_rev += 1
            self.m["replans"] += 1
            self.state = mode
            self.replan_at = None
            self.log("ok", "REPLANNED", f"New route locked ({self.replan_why}): {len(res.path)} cells, "
                     f"{res.expansions} nodes, {res.ms:.1f} ms")
            self.outbox.append(("rel", "ROUTE", {"rev": self.route_rev, "route": [list(c) for c in self.route],
                                                  "why": self.replan_why, "ms": round(res.ms, 2)}, "ROUTE"))
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

    def _start_rtl(self, reason):
        self.failsafe = self.failsafe or reason
        self.failsafe_reason = reason
        self.log("warn", "RTL", f"Return to launch ({reason})")
        self._schedule_replan("RTL", reason)

    def _threat_check(self, full_route=False):
        grid = self.grid()
        window = None if full_route else None
        i = P.first_blocked(grid, self.route, self.seg + 1, window)
        if i is None:
            return False
        hid = grid.blocker(tuple(self.route[i])) or "ceiling"
        dist = i - self.seg
        if hid not in self.detect:
            self.detect[hid] = dist
            self.m["detect_cells"][hid] = dist
        mode = self.state if self.state in FLYING else (self.resume_state or "ENROUTE")
        self.log("warn", "THREAT", f"{hid} blocks route {dist} cells ahead: replanning")
        self._schedule_replan(mode, f"{hid} ahead")
        return True

    def _sense(self):
        """Forward sensor: looks `lookahead` cells along the remaining route (main.py semantics)."""
        end = min(len(self.route), self.seg + 1 + int(self.p["lookahead"]))
        cells = [tuple(self.route[i]) for i in range(self.seg, end)]
        found = []
        for h in truth_hazards(self.scn, self.t):
            if h["id"] in self.sensed or h["id"] in self.uplink:
                continue
            z0, z1 = P.hazard_zrange(h, self.L)
            for k, c in enumerate(cells):
                if h["x0"] <= c[0] <= h["x1"] and h["y0"] <= c[1] <= h["y1"] and z0 <= c[2] <= z1:
                    hh = dict(h)
                    hh["source"] = "DRONE SENSOR"
                    self.sensed[h["id"]] = hh
                    self.m["sensed"] += 1
                    found.append((h["id"], k))
                    break
        for hid, k in found:
            self.log("warn", "SENSED", f"Own sensor detected {hid} {k} cells ahead")
            self.outbox.append(("rel", "HZR", {"hz": self.sensed[hid], "t": round(self.t, 2)}, f"HZR{hid}"))
        return found

    def _enter_cell(self, prev, cell):
        grid = self.grid()
        if not grid.free(cell):
            self.m["blocked_entries"] += 1
            self.log("crit", "INVARIANT", f"Entered known-blocked cell {cell}")
        tg = self.truth_grid()
        if not tg.free(cell):
            self.m["exposures"] += 1
            self.log("crit", "EXPOSURE", f"Flew into {tg.blocker(cell)} at {cell} (not known in time)")
        cost = grid.move_cost(prev, cell)
        self.m["path_cost"] += cost
        self.energy -= cost * 0.5 * self.p["burn"]
        self.m["cells"] += 1
        self.m["dist"] += P.step_length(prev, cell)
        self.cell = cell
        self.flown.append(cell)
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
        if st == "REPLAN":
            self.energy -= self.p["loiter_burn"] * dt
            if self.replan_at is not None and t >= self.replan_at:
                self._execute_replan()
        elif st == "HOLD":
            self.energy -= self.p["loiter_burn"] * dt
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
        self.progress += self.p["speed"] * dt / P.step_length(a, b)
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
            a, b = self.route[self.seg], self.route[self.seg + 1]
            f = min(1.0, self.progress)
            self.pos = [a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f, a[2] + (b[2] - a[2]) * f]
        else:
            self.pos = [float(self.cell[0]), float(self.cell[1]), float(self.cell[2])]

    def _on_cell(self):
        """Called after entering a cell. Returns True if movement must stop this frame."""
        if self.state == "ENROUTE" and self.target_idx < len(self.targets) and self.cell == self.targets[self.target_idx]:
            self.target_idx += 1
            self.fs_count = 0
            if self.target_idx < len(self.targets):
                self.log("ok", "WAYPOINT", f"Waypoint {self.target_idx} reached")
        self._sense()
        if self._threat_check():
            return True
        if self.state == "ENROUTE" and self.target_idx < len(self.targets):
            g = self.grid()
            finish = g.path_cost(self.route[self.seg:]) * 0.5 * self.p["burn"]
            if self.energy - finish >= self.reserve():
                return False                       # can still finish with reserve: carry on
            home = P.astar(g, self.cell, self.home, self.p["budget"])
            if home.ok:
                need = home.cost * 0.5 * self.p["burn"]
                if self.energy - need < self.reserve() + 1.0:
                    self.failsafe, self.failsafe_reason = "BINGO", "energy"
                    self.m["failsafes"] += 1
                    self.log("warn", "BINGO", f"Bingo energy: {self.energy:.1f} left, {need:.1f} needed to get home "
                             f"+ {self.reserve():.1f} reserve")
                    self._start_rtl("BINGO")
                    return True
        return False

    def _arrive(self):
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
        r = P.astar(self.grid(), self.cell, self.home, self.p["budget"])
        need = (r.cost if r.ok else 0.0) * 0.5 * self.p["burn"]
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
                "fs": self.failsafe, "sp": self.p["speed"], "seg": self.seg, "bbn": len(self.bb)}

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
        }
