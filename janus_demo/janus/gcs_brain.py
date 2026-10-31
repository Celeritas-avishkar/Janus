"""Ground-station logic, independent of networking.

Everything the ground station shows about the drone comes from radio messages
it received (telemetry, route updates, hazard reports, black-box records).
It never reads simulation truth. During an outage it shows a dead-reckoned
ghost: the last known position advanced along the last known route at the
profile speed, with an uncertainty radius that grows with time.
"""
import math

from . import config as C
from . import planner as P
from .world import WeatherFeed, grid_for, initial_plan


class GcsBrain:
    def __init__(self, scn, profile):
        self.scn, self.p = scn, profile
        self.feed = WeatherFeed(scn)
        self.t = 0.0
        self.wx = self.feed.snapshot(0.0)
        self.next_wx = 1.0
        self.pic_v = 1
        self.reported = {}
        self.picture = self._build_picture(0.0)
        self.plan, self.legs, _ = initial_plan(scn, profile, scn["static"] + self.picture)
        self.events = []
        self.outbox = []
        self.uploads = []
        self.mission_live = False
        self.rid = 0
        self.next_hb = 0.0
        self.last_rx = None
        self.link = "NO_LINK"
        self.tm = None
        self.tm_t = None
        self.tm_count = 0
        self.tm_first_seq = None
        self.route = None
        self.route_rev = 0
        self.track = []
        self.bb = {}
        self.bb_have = -1
        self.outages = []
        self.outage = None
        self.ghost = None
        self.results = []
        self._sent = {}
        self._last_ghost = None
        self.paused_feed = False

    # ---------------------------------------------------------- picture
    def _build_picture(self, t):
        hz = {h["id"]: h for h in self.feed.visible_hazards(t)}
        for hid, h in self.reported.items():
            hz.setdefault(hid, h)
        return list(hz.values())

    def log(self, level, code, text):
        self.events.append({"t": round(self.t, 2), "lvl": level, "code": code, "text": text})
        self.events = self.events[-200:]

    def picture_msg(self):
        return {"v": self.pic_v, "hz": self.picture, "t": round(self.t, 2), "src": self.wx.get("source")}

    # ---------------------------------------------------------- operator actions
    def upload(self, route=None, targets=None, launch=True, unsafe=False):
        """Queue a MISSION upload. unsafe=True deliberately draws the route
        straight through the nearest blocking volume (operator-error demo)."""
        self.rid += 1
        rid = f"M{self.rid}"
        targets = targets or [list(c) for c in self.scn["targets"]]
        if route is None:
            if unsafe:
                route = self._unsafe_route()
            else:
                if not self.plan.ok:
                    self.log("warn", "NO_PLAN", f"No safe pre-planned route ({self.plan.reason}); upload blocked")
                    return None
                route = [list(c) for c in self.plan.path]
        up = {"rid": rid, "status": "SENDING", "t0": round(self.t, 2), "attempts": 0, "reason": None,
              "unsafe": unsafe, "cells": len(route), "rtt": None}
        self.uploads.append(up)
        self.uploads = self.uploads[-8:]
        self._sent[rid] = route
        msg = {"rid": rid, "route": route, "targets": targets, "launch": launch, "pic": self.picture_msg()}
        self.outbox.append(("rel", "MISSION", msg, None, rid))
        self.log("info", "UPLOAD", f"Mission {rid} sent: {len(route)} cells" + (" (operator error: drawn through a hazard)" if unsafe else ""))
        return rid

    def _unsafe_route(self):
        """Straight-line route home -> goal at cruise altitude, ignoring hazards."""
        home, goal = self.scn["home"], self.scn["targets"][-1]
        z = max(1, min(int(self.p["ceiling"]), self.scn["targets"][0][2]))
        x, y = home[0], home[1]
        route = [[x, y, 0]]
        for zz in range(1, z + 1):
            route.append([x, y, zz])
        while (x, y) != (goal[0], goal[1]):
            x += (goal[0] > x) - (goal[0] < x)
            y += 0 if x % 2 else (goal[1] > y) - (goal[1] < y)
            route.append([x, y, z])
        while route[-1][2] != goal[2]:
            zz = route[-1][2] + (1 if goal[2] > route[-1][2] else -1)
            route.append([x, y, zz])
        return route

    def command(self, c):
        self.outbox.append(("rel", "CMD", {"c": c}, f"CMD{c}", None))
        self.log("info", "COMMAND", f"Command {c} sent")

    def set_feed_paused(self, paused):
        self.feed.paused = paused
        self.paused_feed = paused
        self.log("warn" if paused else "ok", "FEED", "Weather feed paused (stale)" if paused else "Weather feed live")

    # ---------------------------------------------------------- transport callbacks
    def on_upload_delivered(self, rid, rtt, attempts):
        for u in self.uploads:
            if u["rid"] == rid and u["status"] == "SENDING":
                u["status"], u["rtt"], u["attempts"] = "DELIVERED", round(rtt, 2), attempts
                self.log("ok", "DELIVERED", f"Mission {rid} delivered in {rtt:.2f} s after {attempts} transmission(s)")

    def on_upload_failed(self, rid):
        for u in self.uploads:
            if u["rid"] == rid and u["status"] == "SENDING":
                u["status"] = "TIMEOUT"
                self.log("crit", "TIMEOUT", f"Mission {rid} not acknowledged: gave up")

    # ---------------------------------------------------------- inbound radio
    def on_message(self, name, obj, t):
        self.t = t
        self.last_rx = t
        if name == "TM":
            if self.tm is None or obj["s"] > self.tm["s"]:
                self.tm, self.tm_t = obj, t
                self.tm_count += 1
                if self.tm_first_seq is None:
                    self.tm_first_seq = obj["s"]
                self.track.append([obj["t"], obj["p"][0], obj["p"][1], obj["p"][2], "tm"])
                self.track = self.track[-1500:]
        elif name == "RESULT":
            for u in self.uploads:
                if u["rid"] == obj.get("rid"):
                    if obj.get("ok"):
                        u["status"] = "ACCEPTED"
                        self.mission_live = True
                        self.log("ok", "ACCEPTED", f"Drone accepted mission {u['rid']}")
                        if obj.get("rev", 0) >= self.route_rev:
                            self.route = self._sent.get(u["rid"], self.route)
                            self.route_rev = obj.get("rev", self.route_rev)
                    else:
                        u["status"], u["reason"] = "REJECTED", obj.get("reason")
                        u["cell"] = obj.get("cell")
                        self.log("warn", "REJECTED", f"Drone rejected {u['rid']}: {obj.get('reason')} (waypoint {obj.get('i')})")
            self.results.append(obj)
        elif name == "ROUTE":
            if obj.get("rev", 0) >= self.route_rev:
                self.route, self.route_rev = obj["route"], obj["rev"]
                self.log("info", "ROUTE", f"Drone reports new route rev {obj['rev']} ({obj.get('why')})")
        elif name == "HZR":
            h = obj.get("hz")
            if h and h["id"] not in self.reported:
                self.reported[h["id"]] = h
                self.log("warn", "DRONE_REPORT", f"Drone reported {h['id']} from its own sensor")
                self._refresh_picture(force=True)
        elif name == "BB":
            start = int(obj.get("from", 0))
            for r in obj.get("r", []):
                self.bb[int(r[0])] = r
            while (self.bb_have + 1) in self.bb:
                self.bb_have += 1
            self._try_reconcile()

    # ---------------------------------------------------------- periodic
    def step(self, dt, t):
        self.t = t
        if t >= self.next_wx:
            self.next_wx = t + 1.0
            self.wx = self.feed.snapshot(t)
            self._refresh_picture()
        if t >= self.next_hb:
            self.next_hb = t + 1.0 / C.GCS_HEARTBEAT_HZ
            self.outbox.append(("unrel", "HB", {"t": round(t, 2), "bb": self.bb_have, "v": self.pic_v}, None, None))
        self._link_monitor()

    def _refresh_picture(self, force=False):
        new = self._build_picture(self.t)
        if force or sorted(h["id"] for h in new) != sorted(h["id"] for h in self.picture):
            added = {h["id"] for h in new} - {h["id"] for h in self.picture}
            self.picture = new
            self.pic_v += 1
            if added:
                self.log("warn", "WX_NEW", f"New hazard {', '.join(sorted(added))} in feed: picture v{self.pic_v}")
            if not self.mission_live:
                self.plan, self.legs, _ = initial_plan(self.scn, self.p, self.scn["static"] + self.picture)
            else:
                self.outbox.append(("rel", "WX", self.picture_msg(), "WX", None))

    def _link_monitor(self):
        if self.last_rx is None:
            new = "NO_LINK"
        else:
            age = self.t - self.last_rx
            new = "CONNECTED" if age < C.LINK_DEGRADED_S else ("DEGRADED" if age < C.LINK_LOST_S else "LOST")
        if new != self.link:
            if new == "LOST" and self.tm is not None:
                self._last_ghost = None
                self.outage = {"t0": round(self.tm_t, 2), "t1": None, "drone_t0": self.tm["t"],
                               "last": list(self.tm["p"]), "ghost": [], "err": None, "recovered": False,
                               "blind": [], "lost_pkts": None, "tm_seq0": self.tm["s"], "bb0": self.bb_have}
                self.log("crit", "LINK_LOST", "No packets from drone: showing dead-reckoned estimate")
            elif new == "CONNECTED" and self.outage is not None and self.outage["t1"] is None:
                o = self.outage
                o["t1"] = round(self.t, 2)
                est = self._last_ghost
                if est and self.tm is not None:
                    o["err"] = round(math.dist(est[:2], self.tm["p"][:2]), 2)
                    o["est_end"] = est
                    o["true_end"] = list(self.tm["p"])
                o["lost_pkts"] = max(0, self.tm["s"] - o["tm_seq0"] - 1) if self.tm else None
                self.log("ok", "LINK_BACK", f"Link restored after {o['t1'] - o['t0']:.1f} s; estimate was off by "
                         f"{o['err']} cells. Requesting black box")
                self.outages.append(o)
                self.outages = self.outages[-6:]
            self.link = new
        if self.link in ("LOST", "DEGRADED") and self.tm is not None:
            self.ghost = self._ghost_at(self.t)
            if self.link == "LOST":
                self._last_ghost = self.ghost
            if self.outage is not None and self.outage["t1"] is None and self.ghost:
                g = self.outage["ghost"]
                if not g or self.t - g[-1][0] > 0.2:
                    g.append([round(self.t, 2)] + [round(v, 2) for v in self.ghost[:3]])
        else:
            self.ghost = None

    def _ghost_at(self, t):
        """Dead reckoning: advance from last telemetry along the last known route."""
        if self.tm is None:
            return None
        age = max(0.0, t - self.tm_t)
        st = self.tm["st"]
        p = list(self.tm["p"])
        r = 0.6 + 0.45 * age
        if st not in ("ENROUTE", "RTL", "RTLINK") or not self.route:
            return [p[0], p[1], p[2], r, age]
        route = self.route
        best, bi = 1e9, 0
        for i, c in enumerate(route):
            d = (c[0] - p[0]) ** 2 + (c[1] - p[1]) ** 2 + (c[2] - p[2]) ** 2
            if d < best:
                best, bi = d, i
        travel = self.tm.get("sp", self.p["speed"]) * age
        x, y, z = p
        i = bi
        while travel > 0 and i < len(route) - 1:
            nx, ny, nz = route[i + 1]
            seg = math.dist((x, y, z), (nx, ny, nz))
            if seg <= 1e-9:
                i += 1
                continue
            if travel >= seg:
                x, y, z = nx, ny, nz
                travel -= seg
                i += 1
            else:
                f = travel / seg
                x, y, z = x + (nx - x) * f, y + (ny - y) * f, z + (nz - z) * f
                travel = 0
        return [x, y, z, r, age]

    def _try_reconcile(self):
        for o in self.outages:
            if o["recovered"] or o["t1"] is None:
                continue
            recs = [r for i, r in sorted(self.bb.items()) if o["drone_t0"] - 0.3 <= r[1] <= o["drone_t0"] + (o["t1"] - o["t0"]) + 0.6]
            have_end = any(r[1] >= o["drone_t0"] + (o["t1"] - o["t0"]) - 0.3 for r in recs)
            contiguous = self.bb_have >= 0 and recs and recs[-1][0] <= self.bb_have
            if recs and have_end and contiguous:
                o["blind"] = [[r[1], r[2], r[3], r[4], r[5]] for r in recs]
                o["recovered"] = True
                o["bb_records"] = len(recs)
                states = sorted({r[5] for r in recs})
                self.log("ok", "BLACKBOX", f"Black box downlinked {len(recs)} records covering the outage; "
                         f"drone was in {', '.join(states)}")

    # ---------------------------------------------------------- views
    def view(self):
        return {
            "t": round(self.t, 2), "link": self.link,
            "age": None if self.last_rx is None else round(self.t - self.last_rx, 2),
            "tm": self.tm, "route": self.route, "route_rev": self.route_rev, "plan": None if not self.plan.ok else [list(c) for c in self.plan.path],
            "plan_info": self.plan.as_dict(), "picture": self.picture, "pic_v": self.pic_v,
            "reported": list(self.reported), "wx": self.wx, "uploads": self.uploads,
            "track": self.track[-600:], "ghost": self.ghost, "outage": self.outage if (self.outage and self.outage["t1"] is None) else None,
            "outages": self.outages, "events": self.events[-40:], "bb_have": self.bb_have,
            "mission_live": self.mission_live, "feed_paused": self.paused_feed,
        }
