"""Ground-station runtime: runs inside the UI process.

Owns the GCS radio socket, the observer socket, the two child processes
(air relay, drone agent), the session lifecycle, failure commands and the
hands-free tour. The UI reads `snapshot()` and calls `command()`.
"""
import atexit
import json
import os
import select
import socket
import subprocess
import sys
import tempfile
import threading
import time
from collections import deque

from . import config as C
from . import planner as P
from .config import MODES, make_profile, validate_profile
from .gcs_brain import GcsBrain
from .net import udp_socket, recv_all, send_bytes, send_json, ports_from
from .node import deliver, pump
from .protocol import Endpoint
from .scenarios import build, SCRIPTS
from .world import box

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------- child processes
class ProcessManager:
    def __init__(self, ports, host, log_dir):
        self.ports, self.host, self.log_dir = ports, host, log_dir
        self.procs = {}
        self.exited = {}

    def start(self, name, module, extra=()):
        self.stop(name)
        log = open(os.path.join(self.log_dir, f"{name}.log"), "ab")
        env = dict(os.environ)
        env["PYTHONPATH"] = ROOT + os.pathsep + env.get("PYTHONPATH", "")
        args = [sys.executable, "-u", "-m", f"janus.{module}", "--ports", json.dumps(self.ports), "--host", self.host] + list(extra)
        kw = {}
        if sys.platform.startswith("win"):
            kw["creationflags"] = getattr(subprocess, "CREATE_NO_WINDOW", 0)
        p = subprocess.Popen(args, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, env=env, **kw)
        self.procs[name] = {"popen": p, "log": log, "started": time.time(), "module": module}
        self.exited.pop(name, None)
        return p.pid

    def stop(self, name):
        info = self.procs.pop(name, None)
        if not info:
            return None
        p = info["popen"]
        pid = p.pid
        if p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
        try:
            info["log"].close()
        except Exception:
            pass
        self.exited[name] = {"pid": pid, "at": time.time(), "code": p.returncode}
        return pid

    def alive(self, name):
        info = self.procs.get(name)
        return bool(info) and info["popen"].poll() is None

    def pid(self, name):
        info = self.procs.get(name)
        return info["popen"].pid if info else None

    def stop_all(self):
        for n in list(self.procs):
            self.stop(n)


# ---------------------------------------------------------------- runtime
DEFAULT_CFG = {"mode": "small", "preset": "Scout-Q  light quadcopter", "script": "nominal", "seed": 1,
               "weather": "breezy", "overrides": {}, "uplink": True}


class Runtime:
    def __init__(self, host=C.HOST, port_base=None, spawn=True, drone_host=None, relay_host=None):
        self.host = host
        self.ports = ports_from(port_base)
        self.lock = threading.RLock()
        self.log_dir = tempfile.mkdtemp(prefix="janus_logs_")
        self.pm = ProcessManager(self.ports, host, self.log_dir)
        self.spawn = spawn
        self.relay_host = relay_host or host
        self.drone_host = drone_host or host
        self.s_radio = udp_socket(host if host != "0.0.0.0" else "0.0.0.0", self.ports["gcs_radio"])
        self.s_obs = udp_socket(host if host != "0.0.0.0" else "0.0.0.0", self.ports["gcs_obs"])
        self.relay_gcs = (self.relay_host, self.ports["air_gcs"])
        self.relay_ctl = (self.relay_host, self.ports["air_ctl"])
        self.drone_ctl = (self.drone_host, self.ports["drone_ctl"])
        self.cfg = dict(DEFAULT_CFG)
        self.scn = self.profile = None
        self.scn_rev = 0
        self.gcs = self.ep = None
        self.phase = "SETUP"
        self.session = 0
        self.t0 = time.monotonic()
        self.actions = []
        self.drone_obs = None
        self.drone_seen = None
        self.drone_pid = None
        self.air = {"seen": None, "pid": None, "budget": None, "settings": None, "counts": None}
        self.air_events = deque(maxlen=4000)
        self.evt_id = 0
        self.impair = {"loss_up": 0.0, "loss_dn": 0.0, "latency_ms": 20.0, "jitter_ms": 10.0,
                       "corrupt": 0.0, "dup": 0.0, "blackout": False, "rf": True}
        self.message = "Choose a drone and a story, then press Generate."
        self.error = None
        self.tour = Tour(self)
        self._cfg_id, self._cfg_sent, self._start_sent, self._start_msg = None, 0.0, 0.0, None
        self._drone_session = None
        self.running = True
        if spawn:
            self.pm.start("relay", "relay", ["--gcs-host", host if host != "0.0.0.0" else C.HOST,
                                             "--drone-host", self.drone_host if self.drone_host != "0.0.0.0" else C.HOST])
            self.pm.start("drone", "drone", ["--relay-host", self.relay_host if self.relay_host != "0.0.0.0" else C.HOST,
                                             "--gcs-host", host if host != "0.0.0.0" else C.HOST])
        atexit.register(self.shutdown)
        self.thread = threading.Thread(target=self._loop, name="janus-gcs", daemon=True)
        self.thread.start()

    # ---------------------------------------------------------- lifecycle
    def shutdown(self):
        self.running = False
        try:
            send_json(self.s_obs, {"cmd": "quit"}, self.drone_ctl)
            send_json(self.s_obs, {"cmd": "quit"}, self.relay_ctl)
        except Exception:
            pass
        self.pm.stop_all()

    def now(self):
        return time.monotonic() - self.t0

    def prepare(self, cfg):
        """Build and verify a scenario, then start a preflight session."""
        with self.lock:
            self.tour.stop_if_manual()
            c = dict(DEFAULT_CFG)
            c.update({k: v for k, v in cfg.items() if v is not None})
            prof = make_profile(c["mode"], c["preset"] if c["preset"] in C.PRESETS[c["mode"]] else None)
            ov = {k: v for k, v in (c.get("overrides") or {}).items() if k in prof}
            prof.update(ov)
            prof = validate_profile(prof)
            if c["preset"] not in C.PRESETS[c["mode"]] or ov:
                prof["name"] = (c["preset"] if c["preset"] in C.PRESETS[c["mode"]] else "Custom") + (" (custom)" if ov else "")
        try:
            scn = build(c["script"], int(c["seed"]), c["mode"], prof, c["weather"])
        except Exception as e:
            with self.lock:
                self.error = str(e)
                self.message = f"Could not verify this story for {prof['name']}: {e}"
            return {"ok": False, "error": str(e)}
        with self.lock:
            self.cfg, self.profile, self.scn = c, prof, scn
            self.scn_rev += 1
            self.error = None
            self.impair.update({"loss_up": 0.0, "loss_dn": 0.0, "corrupt": 0.0, "dup": 0.0,
                                "latency_ms": 20.0, "jitter_ms": 10.0, "blackout": False})
            self.impair.update(scn.get("impair", {}))
            self._start_session(mission=False)
            self.message = f"{SCRIPTS[c['script']]} ready: {prof['name']}. Pre-flight link check running."
            return {"ok": True, "story": scn["story"], "profile": prof}

    def _start_session(self, mission):
        self.session += 1
        self.t0 = time.monotonic()
        self.actions = []
        self.drone_obs = None
        self.air_events.clear()
        scn = self.scn
        self.gcs = GcsBrain(scn, self.profile)
        self.ep = Endpoint(0, lambda b: send_bytes(self.s_radio, b, self.relay_gcs), peer=1)
        self._send_relay_config()
        self._start_msg = {"cmd": "start", "scn": scn, "profile": self.profile,
                           "uplink": self.cfg.get("uplink", True), "session": self.session}
        send_json(self.s_obs, self._start_msg, self.drone_ctl)
        self._start_sent = time.monotonic()
        self.phase = "MISSION" if mission else "PREFLIGHT"

    def _send_relay_config(self):
        self._cfg_id = f"{self.session}-{time.monotonic():.3f}"
        send_json(self.s_obs, {"cmd": "config", "id": self._cfg_id, "scn": self.scn, "tx": self.profile["tx_dbm"],
                               "gain": self.profile["gain_dbi"], "settings": dict(self.impair)}, self.relay_ctl)
        self._cfg_sent = time.monotonic()

    def _handshakes(self):
        """Control messages are UDP too: resend until the child echoes them."""
        if self.scn is None:
            return
        mono = time.monotonic()
        if self.air.get("cfg") != self._cfg_id and mono - self._cfg_sent > 0.5:
            send_json(self.s_obs, {"cmd": "config", "id": self._cfg_id, "scn": self.scn, "tx": self.profile["tx_dbm"],
                                   "gain": self.profile["gain_dbi"], "settings": dict(self.impair)}, self.relay_ctl)
            self._cfg_sent = mono
        if (self.drone_obs is None or self._drone_session != self.session) and mono - self._start_sent > 0.5:
            send_json(self.s_obs, self._start_msg, self.drone_ctl)
            self._start_sent = mono

    def launch(self, upload=True):
        with self.lock:
            if self.scn is None:
                return {"ok": False, "error": "Generate a scenario first."}
            fresh = self.scn
            fresh["hazards"] = [h for h in fresh["hazards"] if not h.get("manual")]
            self._start_session(mission=True)
            if upload:
                self.actions.append((0.4, lambda: self.gcs.upload()))
            self.message = "Mission running."
            return {"ok": True}

    # ---------------------------------------------------------- commands
    def command(self, name, arg=None):
        with self.lock:
            g = self.gcs
            if name == "launch":
                return self.launch()
            if name == "launch_noupload":
                return self.launch(upload=False)
            if name == "tour_start":
                self.tour.start(int(arg) if isinstance(arg, (int, float)) else 0)
                return {"ok": True}
            if name == "tour_stop":
                self.tour.stop()
                return {"ok": True}
            if g is None:
                return {"ok": False, "error": "No scenario yet."}
            if name == "rtl":
                g.command("RTL")
            elif name == "resume":
                g.command("RESUME")
            elif name == "upload":
                g.upload()
            elif name == "unsafe":
                g.upload(unsafe=True, launch=False)
            elif name == "cut":
                self.set_impair({"blackout": bool(arg) if arg is not None else not self.impair["blackout"]})
            elif name == "impair":
                self.set_impair(arg or {})
            elif name == "kill_relay":
                pid = self.pm.stop("relay")
                g.log("crit", "RELAY_KILLED", f"Air relay process {pid} terminated: no radio path exists")
            elif name == "restart_relay":
                self.restart_relay()
            elif name == "feed":
                g.set_feed_paused(bool(arg) if arg is not None else not g.paused_feed)
            elif name == "popup":
                return self.popup_now()
            elif name == "tour_start":
                self.tour.start()
            elif name == "tour_stop":
                self.tour.stop()
            else:
                return {"ok": False, "error": f"unknown command {name}"}
            return {"ok": True}

    def set_impair(self, settings):
        clean = {}
        for k, v in settings.items():
            if k in self.impair:
                clean[k] = bool(v) if isinstance(self.impair[k], bool) else float(v)
        self.impair.update(clean)
        send_json(self.s_obs, {"cmd": "set", "settings": clean}, self.relay_ctl)
        if self.gcs is not None and clean:
            desc = ", ".join(f"{k}={v}" for k, v in clean.items())
            self.gcs.log("warn" if any(clean.values()) else "info", "LINK_LAB", f"Air settings changed: {desc}")

    def restart_relay(self):
        if not self.spawn:
            return
        pid = self.pm.start("relay", "relay", ["--gcs-host", self.host if self.host != "0.0.0.0" else C.HOST,
                                               "--drone-host", self.drone_host if self.drone_host != "0.0.0.0" else C.HOST])
        if self.gcs is not None:
            self.gcs.log("ok", "RELAY_UP", f"Air relay restarted as process {pid}")
        self.air["cfg"] = None          # handshake re-sends the configuration once the new process reports

    def popup_now(self):
        """Inject a hazard ahead of the drone's real position (truth); the GCS
        learns it through its feed, the drone through uplink or its sensor."""
        o = self.drone_obs
        if not o or not o.get("route") or self.scn is None:
            return {"ok": False, "error": "Drone is not flying."}
        route, seg = o["route"], o.get("seg", 0)
        ahead = int(self.profile["lookahead"]) + 4
        if seg + ahead >= len(route) - 2:
            ahead = max(2, len(route) - 3 - seg)
        if ahead < 2:
            return {"ok": False, "error": "Too close to the end of the route."}
        c = route[seg + ahead]
        hid = f"W{self.scn['next_hazard_id']}"
        self.scn["next_hazard_id"] += 1
        z0, z1 = max(0, c[2] - 2), min(self.scn["layers"] - 1, c[2] + 2)
        t = self.now()
        if self.scn["mode"] == "small":
            hz = box(hid, "gust", max(0, c[0] - 1), max(0, c[1] - 1), 3, 3, z0, z1, t_on=round(t, 2),
                     gust_kt=round(self.profile["gust_limit_kt"] + 9), source="WEATHER", manual=True)
        else:
            hz = box(hid, "ash", max(0, c[0] - 1), max(0, c[1] - 1), 3, 3, z0, z1, t_on=round(t, 2), sev=3,
                     source="ADVISORY", manual=True)
        for p in self.scn["home"], *self.scn["targets"]:
            if hz["x0"] <= p[0] <= hz["x1"] and hz["y0"] <= p[1] <= hz["y1"]:
                return {"ok": False, "error": "Would cover a waypoint; try again in a moment."}
        self.scn["hazards"].append(hz)
        self.scn_rev += 1
        send_json(self.s_obs, {"cmd": "add_hazard", "hz": hz}, self.drone_ctl)
        self.gcs.log("warn", "INJECT", f"Pop-up {HZ_NAME(hz)} {hid} injected {ahead} cells ahead of the drone")
        return {"ok": True, "id": hid}

    # ---------------------------------------------------------- loop
    def _loop(self):
        last = 0.0
        while self.running:
            try:
                select.select([self.s_radio, self.s_obs], [], [], 0.01)
            except (OSError, ValueError):
                time.sleep(0.01)
            with self.lock:
                now = self.now()
                for data, _a in recv_all(self.s_radio):
                    if self.gcs is not None:
                        deliver(self.gcs, self.ep, data, now)
                for data, _a in recv_all(self.s_obs):
                    self._on_observer(data, now)
                if self.gcs is not None:
                    while self.actions and self.actions[0][0] <= now:
                        _, fn = self.actions.pop(0)
                        try:
                            fn()
                        except Exception as e:
                            self.error = f"action failed: {e}"
                    dt = max(0.0, now - last)
                    self.gcs.step(dt, now)
                    pump(self.gcs, self.ep, now, gcs=True)
                last = now
                self._handshakes()
                try:
                    self.tour.tick(now)
                except Exception as e:
                    self.error = f"tour: {e}"
                    self.tour.active = False

    def _on_observer(self, data, now):
        try:
            m = json.loads(data.decode())
        except Exception:
            return
        kind = m.get("kind")
        if kind == "drone":
            self.drone_seen = time.time()
            self.drone_pid = m.get("pid")
            self._drone_session = m.get("session")
            if m.get("session") == self.session and m.get("obs"):
                self.drone_obs = m["obs"]
        elif kind == "air":
            self.air.update({"seen": time.time(), "pid": m.get("pid"), "cfg": m.get("cfg"), "budget": m.get("budget"),
                             "settings": m.get("settings"), "counts": m.get("counts")})
            rnow = m.get("now", 0.0)
            for ev in m.get("ev", []):
                self.evt_id += 1
                t = round(now - (rnow - ev[0]), 3)
                self.air_events.append([self.evt_id, t] + ev[1:])

    # ---------------------------------------------------------- snapshot for the UI
    def snapshot(self, scn_rev=-1, last_evt=0):
        with self.lock:
            now = self.now()
            out = {"phase": self.phase, "t": round(now, 2), "session": self.session, "msg": self.message,
                   "err": self.error, "scn_rev": self.scn_rev, "cfg": self.cfg, "profile": self.profile,
                   "ports": self.ports, "tour": self.tour.view(), "impair": self.impair}
            if self.scn is not None and scn_rev != self.scn_rev:
                s = self.scn
                out["scn"] = {k: s[k] for k in ("mode", "cols", "rows", "layers", "home", "targets", "gcs",
                                                 "static", "hazards", "wind", "story", "script", "seed", "soc")}
                out["scn"]["layer_ft"] = MODES[s["mode"]]["layer_ft"]
                out["scn"]["cell_m"] = MODES[s["mode"]]["cell_m"]
            if self.gcs is not None:
                out["gcs"] = self.gcs.view()
                out["gcs"]["ep"] = dict(self.ep.stats)
                out["gcs"]["pending"] = self.ep.pending()
                out["gcs"]["gaps"] = self.ep.gaps[-6:]
            out["drone"] = self.drone_obs
            evs = [e for e in self.air_events if e[0] > last_evt][-500:]
            if last_evt > self.evt_id:
                evs = list(self.air_events)[-500:]
            out["air"] = dict(self.air)
            out["air"]["events"] = evs
            out["air"]["evt_id"] = self.evt_id
            tnow = time.time()
            out["air"]["age"] = None if self.air["seen"] is None else round(tnow - self.air["seen"], 2)
            relay_alive = self.pm.alive("relay") if self.spawn else (out["air"]["age"] is not None and out["air"]["age"] < 2)
            out["procs"] = [
                {"name": "GROUND STATION", "pid": os.getpid(), "alive": True,
                 "ports": f"udp {self.ports['gcs_radio']}"},
                {"name": "AIR RELAY", "pid": self.pm.pid("relay") or (self.pm.exited.get("relay") or {}).get("pid") or self.air["pid"],
                 "alive": relay_alive, "ports": f"udp {self.ports['air_gcs']} / {self.ports['air_drone']}",
                 "age": out["air"]["age"]},
                {"name": "DRONE AGENT", "pid": self.pm.pid("drone") or self.drone_pid,
                 "alive": (self.pm.alive("drone") if self.spawn else True) and self.drone_seen is not None and tnow - self.drone_seen < 2.5,
                 "ports": f"udp {self.ports['drone_radio']}",
                 "age": None if self.drone_seen is None else round(tnow - self.drone_seen, 2)},
            ]
            return out

    def evidence(self):
        """Numbers for the Evidence tab; every figure is from this simulated run."""
        with self.lock:
            g, d = self.gcs, self.drone_obs
            if g is None:
                return None
            m = (d or {}).get("metrics", {})
            ms = m.get("replan_ms") or []
            counts = self.air.get("counts") or {}
            return {
                "profile": self.profile, "cfg": self.cfg, "story": self.scn.get("story") if self.scn else None,
                "uploads": g.uploads, "outages": [{k: o.get(k) for k in ("t0", "t1", "err", "recovered", "bb_records", "lost_pkts")}
                                                  for o in g.outages],
                "drone_state": (d or {}).get("state"), "replans": m.get("replans"),
                "replan_ms_median": sorted(ms)[len(ms) // 2] if ms else None, "replan_ms_max": max(ms) if ms else None,
                "detect_cells": m.get("detect_cells"), "exposures": m.get("exposures"),
                "blocked_entries": m.get("blocked_entries"), "sensed": m.get("sensed"), "failsafes": m.get("failsafes"),
                "energy_left": (d or {}).get("energy"), "energy0": (d or {}).get("energy0"),
                "path_cost": m.get("path_cost"), "dist": m.get("dist"),
                "air_counts": counts, "gcs_ep": dict(self.ep.stats), "drone_ep": (d or {}).get("ep"),
                "events_gcs": g.events[-60:], "events_drone": (d or {}).get("events", []),
                "t": round(self.now(), 1),
            }


def HZ_NAME(h):
    return C.HAZARD_LABEL.get(h["type"], h["type"]).lower()


# ---------------------------------------------------------------- hands-free tour
class Tour:
    """Plays verified stories back to back with captions. Judges can stop it or
    use any control at any time; the tour then steps aside."""

    def __init__(self, rt):
        self.rt = rt
        self.active = False
        self.idx = -1
        self.caption = ""
        self.sub = ""
        self.steps = []
        self.chapter = None
        self.t_launch = None
        self.done_at = None
        self.pending = None
        self.cap_t = -1e9
        self.chapters = chapters()

    def view(self):
        return {"active": self.active, "idx": self.idx, "n": len(self.chapters),
                "title": self.chapter["title"] if self.chapter else None,
                "caption": self.caption, "sub": self.sub,
                "list": [c["title"] for c in self.chapters]}

    def start(self, idx=0):
        self.active = True
        self.idx = idx - 1
        self._next()

    def stop(self):
        self.active = False
        self.caption = ""
        self.sub = ""
        self.chapter = None

    def stop_if_manual(self):
        if self.pending is None and self.active:
            self.stop()

    def _next(self):
        self.idx += 1
        if self.idx >= len(self.chapters):
            self.idx = 0
        ch = self.chapters[self.idx]
        self.chapter = ch
        self.caption, self.sub = ch["title"], ch["intro"]
        self.cap_t = time.monotonic()
        self.steps = list(ch["steps"])
        self.done_at = None
        self.pending = ch
        threading.Thread(target=self._prepare, args=(ch,), daemon=True).start()

    def _prepare(self, ch):
        res = self.rt.prepare(ch["cfg"])
        with self.rt.lock:
            if not self.active or self.chapter is not ch:
                self.pending = None
                return
            self.pending = None
            if not res.get("ok"):
                self.caption, self.sub = ch["title"], "Skipped: " + res.get("error", "")
                self.done_at = self.rt.now() - 10
                return
            self.t_launch = self.rt.now() + ch.get("preflight", 2.5)

    def tick(self, now):
        if not self.active or self.pending is not None or self.chapter is None:
            return
        rt = self.rt
        if self.t_launch is not None and now >= self.t_launch:
            self.t_launch = None
            rt.launch(upload=self.chapter.get("auto_upload", True))
            return
        if self.t_launch is not None:
            return
        while self.steps:
            fired = None
            for i, st in enumerate(self.steps):      # a later step may fire first if the story branches
                if time.monotonic() - self.cap_t < st.get("hold", 3.2):
                    break                            # let people read the current caption first
                try:
                    if st["when"](rt):
                        fired = i
                        break
                except Exception:
                    pass
                if i >= 2:
                    break
            if fired is None:
                break
            st = self.steps[fired]
            del self.steps[:fired + 1]
            if st.get("do"):
                st["do"](rt)
            if st.get("say"):
                self.caption = st["say"] if isinstance(st["say"], str) else st["say"](rt)
                self.cap_t = time.monotonic()
            if st.get("sub") is not None:
                self.sub = st["sub"] if isinstance(st["sub"], str) else st["sub"](rt)
        d = rt.drone_obs or {}
        finished = d.get("state") in ("COMPLETE", "LANDED", "FORCED") and rt.phase == "MISSION"
        if finished and self.done_at is None:
            self.done_at = now
        if self.done_at is not None and now - self.done_at > self.chapter.get("linger", 5.0):
            self._next()


def _t(rt):
    return rt.now() if rt.phase == "MISSION" else -1


def _dcodes(rt):
    return [e["code"] for e in (rt.drone_obs or {}).get("events", [])]


def _gcodes(rt):
    return [e["code"] for e in (rt.gcs.events if rt.gcs else [])]


def _upload_status(rt):
    return rt.gcs.uploads[-1]["status"] if rt.gcs and rt.gcs.uploads else None


def chapters():
    def at(sec):
        return lambda rt: _t(rt) >= sec

    def dcode(c):
        return lambda rt: rt.phase == "MISSION" and c in _dcodes(rt)

    def gcode(c):
        return lambda rt: rt.phase == "MISSION" and c in _gcodes(rt)

    def story(key, default=None):
        return lambda rt: (rt.scn or {}).get("story", {}).get(key, default)

    def outage_text(rt):
        o = rt.gcs.outages[-1] if rt.gcs and rt.gcs.outages else None
        if not o:
            return "Link restored."
        return (f"Link back after {o['t1'] - o['t0']:.1f} s. The ghost was {o['err']} cells from the truth; "
                f"the black box now draws the real blind track on the ground station.")

    def detect_text(rt):
        m = ((rt.drone_obs or {}).get("metrics") or {}).get("detect_cells", {})
        pid = (rt.scn or {}).get("story", {}).get("popup")
        n = m.get(pid)
        look = rt.profile["lookahead"] if rt.profile else "?"
        return (f"Uplinked picture arrived: the drone replanned {n} cells before the hazard. "
                f"Its own sensor looks only {look} cells ahead.") if n else "Uplinked picture arrived: drone replanning."

    def reject_text(rt):
        u = [x for x in rt.gcs.uploads if x["unsafe"]] if rt.gcs else []
        r = u[-1]["reason"] if u else "?"
        return f"The drone refused the operator's route: {r}. It has the final say on its own safety."

    small, air = "small", "aircraft"
    return [
        {"title": "1 · Upload over a real UDP link", "intro": "Ground station plans around known hazards, then uploads over UDP to a separate drone process.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "nominal", "seed": 1, "weather": "breezy"},
         "steps": [
             {"hold": 0, "when": gcode("UPLOAD"), "say": "Mission packed into numbered, CRC-checked datagrams",
              "sub": "Watch the strip below: each tick is one packet crossing the air relay."},
             {"when": lambda rt: _upload_status(rt) == "ACCEPTED", "say": "Drone validated the route against its own limits and accepted it",
              "sub": "Left: the ground station, drawn only from packets it received. Right: the drone's own computer."},
             {"when": at(8), "say": "Telemetry streams back at 5 Hz; heartbeats keep both watchdogs fed",
              "sub": "Process IDs on top: three separate programs talking over UDP."},
         ]},
        {"title": "2 · Hazard appears mid-flight", "intro": "A gust cell will pop up ahead of the drone, beyond its own sensor range.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "popup", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": lambda rt: _t(rt) >= (story("t_on", 999)(rt) or 999), "say": "A gust cell just formed ahead of the drone",
              "sub": "The ground station's weather feed will see it about one second later."},
             {"when": gcode("WX_NEW"), "say": "Ground station pushes a new hazard picture over the link", "sub": ""},
             {"when": dcode("REPLANNED"), "say": detect_text, "sub": "Old route dashed, new route solid. Replan time is shown on the drone screen."},
         ]},
        {"title": "3 · Radio shadow: flying blind", "intro": "A tower block stands between the antenna and part of the route. Nobody touches anything.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "shadow", "seed": 2, "weather": "breezy"},
         "linger": 7.0,
         "steps": [
             {"when": gcode("LINK_LOST"), "say": "Line of sight blocked: the ground station has lost the drone",
              "sub": "Left screen now shows a dead-reckoned ghost with a growing uncertainty ring. It is a guess, not the truth."},
             {"when": dcode("FAILSAFE"), "say": "The drone runs its own failsafe",
              "sub": "Right screen: what the drone actually does while nobody can hear it."},
             {"when": gcode("BLACKBOX"), "say": outage_text, "sub": "Compare the ghost path with the recovered track."},
         ]},
        {"title": "4 · Antenna pulled", "intro": "Mid-flight the air relay process is killed. There is no radio path at all.",
         "cfg": {"mode": small, "preset": "Swift-V  VTOL fixed-wing", "script": "antenna", "seed": 1, "weather": "breezy"},
         "linger": 6.0,
         "steps": [
             {"hold": 0, "when": lambda rt: _t(rt) >= (story("t_cut", 999)(rt) or 999), "do": lambda rt: rt.command("kill_relay"),
              "say": "Relay process terminated: no radio path exists",
              "sub": "Its PID turns grey at the top. Packets now go nowhere."},
             {"when": dcode("FAILSAFE"), "say": "Swift-V's policy is CONTINUE: it finishes the job on its last known map",
              "sub": "Different drones carry different failsafe policies."},
             {"hold": 0, "when": lambda rt: _t(rt) >= (story("t_back", 999)(rt) or 999), "do": lambda rt: rt.command("restart_relay"),
              "say": "Relay restarted as a new process", "sub": "Watch the black box fill the gap on the left."},
         ]},
        {"title": "5 · Bad link and an unsafe upload", "intro": "30% uplink loss, 12% corruption. First the operator sends a route drawn straight through a hazard.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "badlink", "seed": 1, "weather": "breezy"},
         "auto_upload": False,
         "steps": [
             {"hold": 0, "when": at(0.6), "do": lambda rt: rt.gcs.upload(unsafe=True, launch=False),
              "say": "Operator error: a straight-line route through a hazard", "sub": "Red ticks: corrupted packets caught by CRC. Retransmits fill the gaps."},
             {"when": lambda rt: _upload_status(rt) == "REJECTED", "say": reject_text, "sub": "Now the ground station uploads its safe plan."},
             {"hold": 4.5, "when": lambda rt: _upload_status(rt) == "REJECTED", "do": lambda rt: rt.gcs.upload(), "say": "Ground station now uploads its own safe plan",
              "sub": "Same bad link: watch the retransmissions."},
             {"when": lambda rt: _upload_status(rt) == "ACCEPTED", "say": "Safe route accepted despite the bad link",
              "sub": "Count the retransmissions and CRC drops in the stats panel."},
         ]},
        {"title": "6 · Aircraft mode: ash advisory", "intro": "Same software, aircraft scale: 0 to 40,000 ft, 125 km across.",
         "cfg": {"mode": air, "preset": "Sentinel  MALE", "script": "popup", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": lambda rt: _t(rt) >= (story("t_on", 999)(rt) or 999), "say": "Volcanic ash advisory issued ahead",
              "sub": "Ash is a hard exclusion for every drone class."},
             {"when": dcode("REPLANNED"), "say": detect_text, "sub": "Sentinel tolerates moderate turbulence but never ash."},
         ]},
        {"title": "7 · Detour forces a low-energy return", "intro": "Battery at partial charge. A detour will make the mission unaffordable.",
         "cfg": {"mode": small, "preset": "Hauler-H  heavy-lift hexacopter", "script": "bingo", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": dcode("REPLANNED"), "say": "Detour around a new hazard costs energy", "sub": "Watch the energy bar on the drone screen."},
             {"when": dcode("BINGO"), "say": "Bingo: the drone turns home on its own with its reserve intact",
              "sub": "It keeps going while it can still finish, and turns back at the point of no return."},
         ]},
    ]
