"""The Janus companion-computer agent.

Division of responsibility (the safety case):
  * The autopilot (ArduPilot / PX4) flies: stabilisation, navigation, its own
    failsafes (battery, RC loss, GCS loss, geofence). Janus never disables them.
  * Janus plans: it uploads missions, watches the hazard picture and the ash
    sensor, and replaces the mission when the route is no longer safe.
  * The mission lives on the autopilot. If the companion computer dies, the
    vehicle keeps flying the last safe route and its own failsafes still work.
  * If anyone else changes the flight mode (pilot on the sticks, a GCS, an
    autopilot failsafe), Janus stops commanding (OVERRIDE) until the mode is
    put back to Mission.
"""
import json
import math
import time

from pymavlink import mavutil

from . import mission as MS
from . import planner as P
from .geo import LocalFrame, heading_ne, wind_to_ne
from .mav import MavError
from .sensor import VirtualOPC
from .zones import Zone, ZoneStore

M = mavutil.mavlink

ACTIVE = ("TAKEOFF", "ENROUTE", "HOLDING", "RETURNING", "ARRIVING")


class BlackBox:
    def __init__(self, path):
        self.f = open(path, "a", buffering=1) if path else None

    def write(self, kind, **kw):
        if self.f:
            self.f.write(json.dumps({"t": round(time.time(), 3), "kind": kind, **kw}, default=str) + "\n")


class Agent:
    def __init__(self, ap, profile, goals_ll, zones_path=None, advisory_port=None, auto_launch=False,
                 upload_fence=False, blackbox=None, log=print, sensor=None, rate_hz=5.0, max_mission_s=3600):
        self.ap, self.p, self.log_fn = ap, profile, log
        self.goals_ll = goals_ll                       # [(lat, lon, rel_alt)]
        self.zones_path, self.advisory_port = zones_path, advisory_port
        self.auto_launch, self.upload_fence = auto_launch, upload_fence
        self.bb = BlackBox(blackbox)
        self.sensor = sensor
        self.dt = 1.0 / rate_hz
        self.max_mission_s = max_mission_s
        self.state = "PREFLIGHT"
        self.engaged = False
        self.expected = set()
        self.route, self.items, self.offset = [], [], 0
        self.goal_idx = 0
        self.hold = None
        self.escape = None
        self.clean_track = []                          # recent (n, e, alt) with clean air
        self.alarm_count = 0
        self.last_sample = 0.0
        self.last_check = 0.0
        self.last_bingo = 0.0
        self.last_pos_log = 0.0
        self.zone_version = -1
        self.field = None
        self.metrics = {"replans": 0, "replan_ms": [], "escapes": 0, "holds": 0, "ash_max": 0.0, "flown_m": 0.0, "air_s": 0.0}
        self.batt0, self.rate_pct_min = None, None
        self._last_ne = None
        self.t_start = None
        self.done_reason = None
        ap.on_text = lambda sev, text: self.bb.write("fc", sev=sev, text=text)

    # ------------------------------------------------------------------ utils
    def event(self, code, text, sev=M.MAV_SEVERITY_INFO):
        key = (code, text)
        now = time.time()
        if getattr(self, "_last_ev", {}).get(key, 0) > now - 10.0:
            return                                       # same message within 10 s: don't spam the GCS
        self._last_ev = {k: v for k, v in getattr(self, "_last_ev", {}).items() if v > now - 10.0}
        self._last_ev[key] = now
        self.log_fn(f"[janus] {code}: {text}")
        self.bb.write("event", code=code, text=text, state=self.state)
        try:
            self.ap.statustext(f"Janus {code}: {text}", sev)
        except Exception:                                # never let a GCS message break the loop
            pass

    def pos(self):
        s = self.ap.s
        n, e = self.frame.to_ne(s["lat"], s["lon"])
        return (n, e, s["rel_alt"] or 0.0)

    def now(self):
        return time.time()

    def goals_ne(self):
        out = []
        for (lat, lon, alt) in self.goals_ll:
            n, e = self.frame.to_ne(lat, lon)
            out.append((n, e, float(alt)))
        return out

    def _set_mode(self, name):
        prev = self.ap.mode_name()
        self.expected = {name} | ({"guided"} if name == "hold" else set())
        # the heartbeat that confirms a mode change can lag the acknowledgement by ~1 s
        self.grace = ({prev} if prev else set(), time.time() + 2.5)
        ok = self.ap.set_mode(name)
        if not ok:
            raise MavError(f"flight controller refused mode {name}")
        return ok

    # ------------------------------------------------------------------ preflight
    def preflight(self):
        ap = self.ap
        self.log_fn(f"[janus] connected: {ap.stack} {ap.vclass} (sys {ap.tsys}); waiting for GPS and home")
        if ap.vclass != self.p["class"]:
            raise MavError(f"profile is for a {self.p['class']} but the autopilot reports a {ap.vclass}")
        ap.wait_ready()
        h = ap.s["home"]
        self.frame = LocalFrame(h[0], h[1])
        self.home = (0.0, 0.0, 0.0)
        self.store = ZoneStore(self.zones_path, self.frame, self.advisory_port)
        if self.sensor is None:
            self.sensor = VirtualOPC(self.store, self.p, seed=1)
        self.event("PREFLIGHT", f"home {h[0]:.6f},{h[1]:.6f}; {len(self.store.file_zones)} zones; "
                                f"{sum(1 for z in self.store.file_zones if not z.reported)} unreported")
        plan = self._plan_from(self.home, takeoff=True)
        if plan is None:
            raise MavError("no safe route to the goal: mission not uploaded")
        if self.upload_fence:
            fz = MS.fence_items(self.frame, self.store.known(), min(self.p["alt_layers_m"]), max(self.p["alt_layers_m"]))
            if fz:
                ap.upload(fz, mission_type=M.MAV_MISSION_TYPE_FENCE)
                self.event("FENCE", f"{len(fz)} exclusion fence items uploaded as a backstop")
        self._upload(plan, start_seq_tag=None)
        self.state = "READY"
        self.zone_version = self.store.version
        self._rebuild_field()

    # ------------------------------------------------------------------ planning
    def _known(self):
        return self.store.known()

    def _plan_from(self, start, takeoff=False, return_home=False):
        """Plan from start through the remaining goals (or home). Returns a dict
        with items, route, hold; or None if no safe plan exists."""
        t = self.now()
        # plan for twice the horizon the safety check uses: a new route stays valid
        # while drifting zones move, instead of being re-flagged a second later
        horizon = 2.0 * self.p["replan_horizon_s"]
        home_pt = (0.0, 0.0, self.p["alt_layers_m"][0])
        if return_home:
            targets = [home_pt]
        else:
            targets = self.goals_ne()[self.goal_idx:]
            if self.p["on_arrival"] == "home":
                targets = targets + [home_pt]     # Janus plans the way back: never a straight-line RTL
        start = (start[0], start[1], max(start[2], self.p["takeoff_alt_m"] if takeoff else start[2]))
        landing = None
        ends_home = return_home or self.p["on_arrival"] == "home"
        if ends_home and self.p["class"] == "plane":
            g0 = P.Grid(self._known(), self.p, [start[:2], self.home[:2]] + [g[:2] for g in targets], t=t, horizon=horizon)
            meas = self.ap.s.get("wind")
            wfrom = meas[0] if (meas and meas[1] > 0.5) else (self.store.wind or {}).get("from_deg")
            landing = P.choose_landing(g0, (0.0, 0.0), wfrom, self.p, self.p["alt_layers_m"][0])
            if landing is None:
                self.event("NOTE", "no clear landing approach at home: the mission ends with RTL", M.MAV_SEVERITY_WARNING)
            else:
                c = landing["centre"]
                targets = targets[:-1] + [(c[0], c[1], self.p["alt_layers_m"][0])]
        route = [start]
        hold = None
        t0 = time.perf_counter()
        grid = None
        for i, g in enumerate(targets):
            res, grid = P.plan(self._known(), self.p, route[-1], g, t=t, horizon=horizon,
                               extra_points=[self.home[:2]])
            if not res.ok:
                if res.reason.startswith("goal blocked") and not return_home:
                    inbound = heading_ne(g[0] - route[-1][0], g[1] - route[-1][1])
                    hold = P.choose_hold(grid, g, inbound, self.p)
                    if hold is None:
                        self.event("NO_HOLD", f"goal blocked and no clear hold pattern fits near it", M.MAV_SEVERITY_WARNING)
                        return None
                    res2, grid = P.plan(self._known(), self.p, route[-1], hold["fix"], t=t, horizon=horizon,
                                        extra_points=[self.home[:2], g[:2]])
                    if not res2.ok:
                        self.event("NO_ROUTE", f"cannot reach a hold fix: {res2.reason}", M.MAV_SEVERITY_WARNING)
                        return None
                    route += res2.path[1:]
                    hold["goal_index"] = self.goal_idx + i
                    break
                self.event("NO_ROUTE", res.reason, M.MAV_SEVERITY_WARNING)
                return None
            route += res.path[1:]
        ms = (time.perf_counter() - t0) * 1000
        # final item: at home a copter lands, a plane hands over to RTL (already over home);
        # at the goal: land or loiter as configured
        arrival = ("land" if self.p["class"] == "copter" else "rtl") if ends_home else self.p["on_arrival"]
        items, notes = MS.build(self.frame, self.p, self.ap.stack, route, takeoff=takeoff, arrival=arrival,
                                hold=hold, grid=grid, landing=None if hold else landing)
        if landing and not hold:
            self.event("LANDING", f"planned approach heading {landing['heading']:.0f} deg, orbit r={landing['radius']:.0f} m "
                                  f"down to {landing['alt']:.0f} m, {self.p['approach_len_m']:.0f} m final")
        for n in notes:
            self.event("NOTE", n, M.MAV_SEVERITY_NOTICE)
        return {"items": items, "route": route, "hold": hold, "ms": ms, "return_home": return_home}

    def _upload(self, plan, start_seq_tag=None):
        items = plan["items"]
        # the offset is only known after upload; DO_JUMP needs it, so compute it first
        offset = 1 if self.ap.stack == "ardupilot" else 0
        items = MS.fix_jumps(items, offset)
        self.offset = self.ap.upload(items)
        self.items, self.route, self.hold = items, plan["route"], plan["hold"]
        self.bb.write("plan", route=[[round(x, 1) for x in p] for p in self.route], hold=bool(self.hold),
                      ms=round(plan["ms"], 1), items=len(items))
        desc = f"{len(self.route) - 1} waypoints, planned in {plan['ms']:.0f} ms"
        if self.hold:
            desc += f"; goal blocked -> {self.hold['kind']} hold at the nearest clear fix"
        self.event("MISSION", desc)

    def _tag(self, seq):
        if seq is None:
            return None
        i = seq - self.offset
        return self.items[i]["tag"] if 0 <= i < len(self.items) else None

    def _remaining_route(self):
        """Current position followed by the waypoints not yet reached."""
        here = self.pos()
        tag = self._tag(self.ap.s["seq"])
        if tag and tag.startswith("wp"):
            k = int(tag[2:])
            return [here] + self.route[k:]
        if tag and tag.startswith("hold"):
            return [here] + [(n, e, self.hold["fix"][2]) for (n, e) in self.hold["pts"]] + [(self.hold["pts"][0][0], self.hold["pts"][0][1], self.hold["fix"][2])]
        if tag and tag.startswith("climb"):
            k = int(tag[5:])
            return [here] + self.route[k - 1:]
        if tag in ("land", "rtl", "loiter", "land_start", "approach"):
            return [here] + self.route[-1:]
        return [here] + self.route[1:]          # takeoff / speed item (or not reported yet): the whole route is ahead          # takeoff item (or not reported yet): the whole route is ahead

    def replan(self, why, return_home=False):
        t0 = time.perf_counter()
        start = self.pos()
        plan = self._plan_from(start, return_home=return_home)
        if plan is None:
            if self.state != "ESCAPING" and self._escape_to_clean_air("no safe route from inside a hazard margin"):
                return False
            if self.state != "WAITING":
                # never keep flying a route we know is unsafe: stop here and keep trying
                self._set_mode("hold")
                self.state_before_wait = "RETURNING" if return_home else (self.state if self.state != "ESCAPING" else "ENROUTE")
                self.state = "WAITING"
                self._wait_started = self.now()
                self.event("WAIT", f"{why}, but no safe route yet: holding here and retrying", M.MAV_SEVERITY_WARNING)
            self._wait_t = self.now()
            return False
        self._set_mode("hold")                       # stop on the current (safe) spot while the mission is replaced
        self._upload(plan)
        first = self.offset
        if not self.ap.set_current(first):
            self.event("WARN", "could not set the current mission item", M.MAV_SEVERITY_WARNING)
        self._set_mode("mission")
        ms = (time.perf_counter() - t0) * 1000
        self.metrics["replans"] += 1
        self.metrics["replan_ms"].append(ms)
        self.event("REPLAN", f"{why}; new route live in {ms:.0f} ms")
        self.state = "HOLDING" if plan["hold"] else ("RETURNING" if return_home else "ENROUTE")
        if plan["hold"]:
            self.metrics["holds"] += 1
        return True

    def _rebuild_field(self):
        pts = [self.home[:2]] + [g[:2] for g in self.goals_ne()]
        if self.ap.s["lat"] is not None:
            pts.append(self.pos()[:2])
        grid = P.Grid(self._known(), self.p, pts, t=self.now(), horizon=0.0)
        self.field = P.HomeField(grid, grid.cell_of(0.0, 0.0, self.p["alt_layers_m"][0]))

    # ------------------------------------------------------------------ launch
    def launch(self):
        ap = self.ap
        self.event("LAUNCH", "auto launch")
        if ap.stack == "ardupilot" and ap.vclass == "copter":
            self.expected = {"guided"}
            ap.takeoff_copter_apm(self.p["takeoff_alt_m"])
            first_wp = next(i for i, it in enumerate(self.items) if it["tag"] == "wp1")
            ap.set_current(self.offset + first_wp)
            self._set_mode("mission")
        else:
            # ArduPlane, PX4 copter and plane: start the mission (its TAKEOFF item) and arm
            self._set_mode("mission")
            ap.set_current(self.offset)
            ap.arm()
        self.engaged = True
        self.state = "TAKEOFF"
        self.t_start = self.now()

    # ------------------------------------------------------------------ sensing
    def _sample(self, t):
        if t - self.last_sample < self.p["opc_period_s"]:
            return
        self.last_sample = t
        n, e, alt = self.pos()
        c = self.sensor.read(n, e, alt, t)
        self.metrics["ash_max"] = max(self.metrics["ash_max"], c)
        self.bb.write("opc", n=round(n, 1), e=round(e, 1), alt=round(alt, 1), mg=round(c, 3))
        if c < self.p["opc_alarm_mg"]:
            self.alarm_count = 0
            self.clean_track.append((n, e, alt))
            self.clean_track = self.clean_track[-120:]
            return
        self.alarm_count += 1
        if self.alarm_count == 2 and self.state in ("ENROUTE", "HOLDING", "RETURNING") and self.escape is None:
            self._on_ash(n, e, alt, c, t)

    def _on_ash(self, n, e, alt, c, t):
        """Ash found by our own sensor. Estimate where the plume is, drift it with
        the measured wind, and get out: veer off at the edge, turn back to the
        last clean air (never climbing) in the core."""
        s = self.ap.s
        trk = math.atan2(s["ve"], s["vn"]) if math.hypot(s["vn"], s["ve"]) > 0.5 else 0.0
        r = self.p["est_radius_m"]
        prev = [z for z in self.store.estimated.values() if z.distance(n, e, t) < r]
        if prev:
            r = max(z.circles[0][2] for z in prev) * 1.5
        ahead = 0.6 * r if c >= self.p["opc_core_mg"] else r + self.p["margin_m"] * 0.5
        cn, ce = n + ahead * math.cos(trk), e + ahead * math.sin(trk)
        # wind: the autopilot's estimate if it has one (ArduPlane / PX4 with airspeed),
        # otherwise the forecast in the zone file (multirotors rarely estimate wind)
        meas = s.get("wind")
        fc = self.store.wind or {}
        if meas and meas[1] > 0.5:
            wind, wsrc = meas, "measured"
        elif fc.get("speed_ms") is not None:
            wind, wsrc = (fc["from_deg"], fc["speed_ms"]), "forecast"
        else:
            wind, wsrc = None, "none"
        vn = ve = 0.0
        if wind:
            vn, ve = wind_to_ne(wind[0], wind[1])
        lat, lon = self.frame.to_ll(cn, ce)
        zid = f"EST{len(self.store.estimated) + 1}"
        z = Zone({"id": zid, "type": "ash", "shape": "circle", "lat": lat, "lon": lon, "radius_m": r,
                  "conc_mg": max(c, 2.0), "alt_min_m": max(0.0, alt - 60), "alt_max_m": alt + 60,
                  "vn": vn, "ve": ve, "t_ref": t, "estimated": True}, self.frame)
        self.store.add_estimate(z)
        self.metrics["escapes"] += 1
        where = "core" if c >= self.p["opc_core_mg"] else "edge"
        self.event("ASH", f"OPC {c:.2f} mg/m3 ({where}); estimate {zid} r={r:.0f} m drifting "
                          f"{math.hypot(vn, ve):.1f} m/s ({wsrc} wind)", M.MAV_SEVERITY_WARNING)
        if where == "core" and self._escape_to_clean_air("inside the ash core"):
            return
        self.replan("ash ahead from onboard sensor", return_home=self.state == "RETURNING")

    def _escape_to_clean_air(self, why):
        """180-degree turn back to the most recent clean air a little behind us,
        never climbing (published advice for inadvertent ash encounters)."""
        if not self.clean_track:
            return False
        n, e, alt = self.pos()
        grid = P.Grid(self._known(), self.p, [(n, e), self.home[:2]], t=self.now(), horizon=self.p["replan_horizon_s"])
        if P.start_clearance(grid, (n, e, alt)) is None:
            return False                                    # not inside any margin: nothing to escape from
        back = next((p for p in reversed(self.clean_track)
                     if math.hypot(p[0] - n, p[1] - e) > max(self.p["accept_radius_m"] * 2, self.p["margin_m"])),
                    self.clean_track[0])
        tgt = (back[0], back[1], min(alt, back[2]) if back[2] > 5 else alt)
        la, lo = self.frame.to_ll(tgt[0], tgt[1])
        self.grace = ({self.ap.mode_name()}, time.time() + 2.5)
        self.expected = {"guided", "hold"}
        if not self.ap.goto(la, lo, tgt[2]):
            self.event("WARN", "reposition refused by the autopilot", M.MAV_SEVERITY_WARNING)
            return False
        self.escape = {"target": tgt, "t0": self.now()}
        if self.state not in ("ESCAPING", "WAITING"):
            self.state_before_escape = self.state
        elif self.state == "WAITING":
            self.state_before_escape = getattr(self, "state_before_wait", "ENROUTE")
        self.state = "ESCAPING"
        self.event("ESCAPE", f"{why}: turning back to the last clean air (no climb)", M.MAV_SEVERITY_WARNING)
        return True

    # ------------------------------------------------------------------ checks
    def _check_route(self, t):
        if t - self.last_check < 1.0:
            return
        self.last_check = t
        changed = self.store.reload() | self.store.poll_advisories()
        if changed or self.store.version != self.zone_version:
            self.zone_version = self.store.version
            self._rebuild_field()
        if self.state not in ("ENROUTE", "HOLDING", "RETURNING"):
            return
        if self.state == "HOLDING":
            self._check_hold_clear()
            return
        rem = self._remaining_route()
        grid = P.Grid(self._known(), self.p, [p[:2] for p in rem] + [self.home[:2]], t=t, horizon=self.p["replan_horizon_s"])
        probs = P.route_problems(grid, rem, P.start_clearance(grid, rem[0]),
                                 times=[t, t + self.p["replan_horizon_s"] / 2, t + self.p["replan_horizon_s"]])
        if probs:
            self.replan("route conflict: " + probs[0], return_home=self.state == "RETURNING")

    def _check_hold_clear(self):
        if self.now() - getattr(self, "_last_hold_check", 0.0) < 3.0:
            return
        self._last_hold_check = self.now()
        gi = self.hold.get("goal_index", self.goal_idx)
        g = self.goals_ne()[gi]
        res, _ = P.plan(self._known(), self.p, self.pos(), g, t=self.now(), horizon=2.0 * self.p["replan_horizon_s"])
        if res.ok:
            self.event("HOLD_CLEAR", "goal is clear: leaving the hold straight for it")
            self.replan("hold cleared")

    def _bingo(self, t):
        """Point of no return: can we still get home with the reserve?"""
        if t - self.last_bingo < 1.0 or self.state not in ("ENROUTE", "HOLDING", "WAITING"):
            return
        self.last_bingo = t
        b = self.ap.s["batt_pct"]
        if b is None or self.field is None:
            return
        n, e, alt = self.pos()
        d = self.field.distance(n, e, alt)
        if d is None:
            return
        if self.batt0 is None:
            self.batt0 = b
        # multirotors (and planes) burn energy per unit time, not per metre: learn %/min in the air
        used = self.batt0 - b
        prior = self.p["pct_per_min"]
        air_min = self.metrics["air_s"] / 60.0
        if air_min > 1.0 and used > 2.0:
            learned = used / air_min
            self.rate_pct_min = learned if self.rate_pct_min is None else 0.8 * self.rate_pct_min + 0.2 * learned
        rate = max(prior * 0.5, self.rate_pct_min or prior)
        t_home_min = d / self.p["cruise_ms"] / 60.0
        need = t_home_min * rate * 1.25 + self.p["reserve_pct"]
        if b < need:
            self.event("BINGO", f"battery {b:.0f}% < {need:.0f}% needed to get home ({d:.0f} m, {rate:.1f} %/min) "
                                f"with the {self.p['reserve_pct']:.0f}% reserve: returning",
                       M.MAV_SEVERITY_WARNING)
            if not self.replan("bingo", return_home=True):
                self._fallback_home()                    # battery is the hard limit: no time to keep retrying

    def _fallback_home(self):
        if self.ap.stack == "ardupilot" and self.ap.vclass == "copter":
            self.event("FALLBACK", "no planned way home: SMART_RTL (retraces the flown, known-clear path)", M.MAV_SEVERITY_CRITICAL)
            self._set_mode("smart_rtl")
        else:
            self.event("FALLBACK", "no planned way home: autopilot RTL (straight line, hazards NOT avoided)", M.MAV_SEVERITY_CRITICAL)
            self._set_mode("rtl")
        self.state = "HANDED_OFF"

    def _progress(self):
        seq = self.ap.s["seq"]
        if seq is not None and self.items and seq - self.offset >= len(self.items) - 2:
            # flying the last leg: the final item may switch the mode (ArduPlane runs a mission RTL
            # as RTL mode) and the heartbeat can show it before MISSION_CURRENT does
            last = self.items[-1]["tag"]
            self.expected |= {"rtl", "hold"} if last == "rtl" else ({"land", "hold"} if last == "land" else set())
        tag = self._tag(seq)
        if tag in ("land", "rtl", "loiter", "land_start", "approach") and self.state in ("ENROUTE", "RETURNING"):
            self.state = "ARRIVING"
            self.expected |= {"rtl", "land", "hold"}     # PX4 switches to Hold once landed / mission ends
            self.event("ARRIVING", f"final item: {tag}")

    def _watch_override(self):
        mode = self.ap.mode_name()
        if mode is None:
            return
        g_modes, g_until = getattr(self, "grace", (set(), 0.0))
        if self.engaged and mode not in self.expected and not (mode in g_modes and time.time() < g_until):
            self.engaged = False
            self.state_before_override = self.state
            self.state = "OVERRIDE"
            self.event("OVERRIDE", f"mode changed to {mode} by pilot/GCS/failsafe: Janus stops commanding",
                       M.MAV_SEVERITY_WARNING)
        elif not self.engaged and self.state == "OVERRIDE" and mode == "mission":
            self.engaged = True
            self.state = getattr(self, "state_before_override", "ENROUTE")
            self.expected = {"mission"}
            self.event("RESUME", "back in Mission: Janus re-engaged, re-checking the route")
            self.last_check = 0.0

    # ------------------------------------------------------------------ main loop
    def step(self):
        ap = self.ap
        ap.pump(self.dt)
        t = self.now()
        s = ap.s
        if s["lat"] is None:
            return
        here = self.pos()
        if self._last_ne is not None and s["armed"]:
            self.metrics["flown_m"] += math.hypot(here[0] - self._last_ne[0], here[1] - self._last_ne[1])
            if here[2] > 2.0:
                self.metrics["air_s"] += t - getattr(self, "_last_t", t)
        self._last_t = t
        self._last_ne = here
        if t - self.last_pos_log >= 0.5:
            self.last_pos_log = t
            self.bb.write("pos", lat=s["lat"], lon=s["lon"], alt=s["rel_alt"], mode=ap.mode_name(), state=self.state,
                          batt=s["batt_pct"], seq=s["seq"], armed=s["armed"])
        if not ap.link_ok():
            return                                         # autopilot still flies its mission; nothing to command
        if self.state == "READY":
            if s["armed"] and ap.mode_is("mission"):
                self.engaged, self.state, self.expected = True, "TAKEOFF", {"mission", "takeoff"}
                self.t_start = t
                self.event("ENGAGED", "operator started the mission")
            return
        if self.state == "OVERRIDE":
            self._watch_override()
            return
        if self.state == "TAKEOFF":
            self.expected = {"mission", "takeoff", "guided"}
            if (s["rel_alt"] or 0) > 0.8 * self.p["takeoff_alt_m"]:
                self.state = "ENROUTE"
                self.expected = {"mission"}
                self.event("ENROUTE", "climbed out; monitoring")
            self._watch_override()
            return
        if self.state == "WAITING":
            self._watch_override()
            if self.state == "OVERRIDE":
                return
            self._sample(t)
            self._bingo(t)
            if self.state == "WAITING" and t - getattr(self, "_wait_t", 0.0) > 3.0:
                ok = self.replan("retry", return_home=self.state_before_wait == "RETURNING")
                if not ok and t - getattr(self, "_wait_started", t) > 120:
                    self._fallback_home()
            return
        if self.state in ("ENROUTE", "HOLDING", "RETURNING", "ESCAPING", "ARRIVING"):
            self._progress()                               # final item may switch the mode (ArduPlane RTL)
            self._watch_override()
            if self.state == "OVERRIDE":
                return
            self._sample(t)
            self._progress()
            if self.state == "ESCAPING":
                tg = self.escape["target"]
                if math.hypot(here[0] - tg[0], here[1] - tg[1]) < max(self.p["accept_radius_m"] * 2, 8.0) or t - self.escape["t0"] > 60:
                    self.escape = None
                    self.state = getattr(self, "state_before_escape", "ENROUTE")
                    self.event("ESCAPED", "back in clean air: planning around the plume")
                    self.replan("around the ash we found", return_home=self.state == "RETURNING")
                return
            self._check_route(t)
            self._bingo(t)
            self._progress()
            if self.state == "ARRIVING":
                tag = self._tag(s["seq"])
                last = self.route[-1]
                if not s["armed"]:
                    self.done_reason = "landed and disarmed"
                elif tag == "loiter" and math.hypot(here[0] - last[0], here[1] - last[1]) < self.p["loiter_radius_m"] * 1.5:
                    self.done_reason = "loitering at the goal"
                elif (tag == "rtl" or ap.mode_is("rtl")) and math.hypot(here[0], here[1]) < self.p["loiter_radius_m"] * 2 \
                        and self.p["class"] == "plane":
                    self.done_reason = "back over home; the autopilot's RTL takes it from here"
                if self.done_reason:
                    self.state = "DONE"
                    self.event("DONE", f"{self.done_reason}; {self.metrics['replans']} replans, "
                                       f"{self.metrics['escapes']} ash encounters, {self.metrics['flown_m']:.0f} m flown")

    def run(self):
        if self.state == "PREFLIGHT":
            self.preflight()
        if self.auto_launch:
            self.launch()
        else:
            self.event("READY", "mission uploaded: arm and switch to Mission/AUTO to start")
        t_end = time.time() + self.max_mission_s
        while self.state not in ("DONE", "HANDED_OFF") and time.time() < t_end:
            self.step()
        return self.state

    def summary(self):
        m = self.metrics
        return {"state": self.state, "reason": self.done_reason, "replans": m["replans"],
                "replan_ms_max": round(max(m["replan_ms"]), 0) if m["replan_ms"] else None,
                "ash_encounters": m["escapes"], "holds": m["holds"], "ash_max_mg": round(m["ash_max"], 2),
                "flown_m": round(m["flown_m"]), "estimated_zones": list(self.store.estimated)}
