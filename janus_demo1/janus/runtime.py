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
from .world import box, circle, drift_velocity, grid_for

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
DEFAULT_CFG = {"mode": "small", "preset": "Scout-Q  light quadcopter", "script": "sensor", "seed": 1,
               "weather": "breezy", "overrides": {}, "uplink": True, "sar": False, "planner": "hybrid",
               "wx_opts": {}, "map_center": "bengaluru", "hold_pattern": "racetrack"}
DEFAULT_IMPAIR = {"loss_up": 0.0, "loss_dn": 0.0, "latency_ms": 20.0, "jitter_ms": 10.0,
                  "corrupt": 0.0, "dup": 0.0, "blackout": False}


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
        self.scn = self.profile = self.base_profile = None
        self.paused, self._pause_mono, self._drone_paused, self._pause_sent = False, 0.0, False, 0.0
        self._compare_cache = None
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
        if self.paused:
            return self._pause_mono - self.t0                      # clock stopped
        return time.monotonic() - self.t0

    def set_pause(self, on):
        """Stop / start the simulation clock. All three programs freeze their
        mission time together, so no watchdog sees a pause as a lost link."""
        with self.lock:
            on = bool(on)
            if on == self.paused:
                return {"ok": True, "paused": on}
            if on:
                self._pause_mono = time.monotonic()
                self.paused = True
            else:
                self.t0 += time.monotonic() - self._pause_mono
                self.paused = False
            send_json(self.s_obs, {"cmd": "pause", "on": on, "session": self.session}, self.drone_ctl)
            self._pause_sent = time.monotonic()
            if self.gcs is not None:
                self.gcs.log("info", "CLOCK", "Clock stopped" if on else "Clock running")
            self.message = ("Clock stopped: the simulation is frozen. Press Start clock to continue."
                            if on else "Clock running.")
            return {"ok": True, "paused": on}

    def prepare(self, cfg):
        """Build and verify a scenario, then start a preflight session."""
        with self.lock:
            self.tour.stop_if_manual()
            c = dict(DEFAULT_CFG)
            c.update({k: v for k, v in cfg.items() if v is not None})
            prof = make_profile(c["mode"], c["preset"] if c["preset"] in C.PRESETS[c["mode"]] else None)
            ov = {k: v for k, v in (c.get("overrides") or {}).items() if k in prof}
            prof.update(ov)
            prof["planner"] = c.get("planner") or "hybrid"
            prof["hold_pattern"] = c.get("hold_pattern") or "racetrack"
            prof = validate_profile(prof)
            if c["preset"] not in C.PRESETS[c["mode"]] or ov:
                prof["name"] = (c["preset"] if c["preset"] in C.PRESETS[c["mode"]] else "Custom") + (" (custom)" if ov else "")
            base_prof = dict(prof)
            if c.get("sar"):
                prof = C.apply_sar(prof)
                prof["name"] = prof["name"] + " · SAR"
        try:
            scn = build(c["script"], int(c["seed"]), c["mode"], prof, c["weather"], opts=c.get("wx_opts") or None)
        except Exception as e:
            with self.lock:
                self.error = str(e)
                self.message = f"Could not verify this story for {prof['name']}: {e}"
            return {"ok": False, "error": str(e)}
        with self.lock:
            self.cfg, self.profile, self.scn = c, prof, scn
            self.base_profile = base_prof
            self.scn_rev += 1
            self.error = None
            self._compare_cache = None
            self.impair.update(DEFAULT_IMPAIR)
            self.impair.update(scn.get("impair", {}))
            self._start_session(mission=False)
            self.message = f"{SCRIPTS[c['script']]} ready: {prof['name']}. Pre-flight link check running."
            return {"ok": True, "story": scn["story"], "profile": prof}

    def _start_session(self, mission):
        self.session += 1
        self.paused = False
        self._drone_paused = False
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
        if self._drone_paused != self.paused and self._drone_session == self.session and mono - self._pause_sent > 0.3:
            send_json(self.s_obs, {"cmd": "pause", "on": self.paused, "session": self.session}, self.drone_ctl)
            self._pause_sent = mono
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
            if not self.pm.alive("relay") and self.spawn:
                self.restart_relay()
            if upload:
                self.actions.append((0.4, lambda: self.gcs.upload()))
            st = self.scn.get("story") or {}
            if self.cfg.get("script") == "antenna" and st.get("t_cut"):
                # Round 1 only pulled the antenna inside the hands-free tour.
                self.actions.append((st["t_cut"], lambda: self.command("kill_relay")))
                self.actions.append((st["t_back"], lambda: self.command("restart_relay")))
                self.message = (f"Mission running. The antenna will be pulled at T+{st['t_cut']:.1f} s "
                                f"and plugged back in at T+{st['t_back']:.1f} s.")
            else:
                self.message = "Mission running."
            self.actions.sort(key=lambda a: a[0])
            return {"ok": True}

    def reset(self):
        """Stop everything and return to a clean pre-flight: drone back on the pad,
        clock and timers at zero, air link healthy, injected hazards removed."""
        with self.lock:
            self.tour.stop()
            if self.spawn and not self.pm.alive("relay"):
                self.restart_relay()
            self.impair.update(DEFAULT_IMPAIR)
            send_json(self.s_obs, {"cmd": "set", "settings": dict(self.impair)}, self.relay_ctl)
            if self.scn is None:
                self.t0 = time.monotonic()
                self.message = "Reset. Choose a drone and a story, then press Generate."
                return {"ok": True}
            self.scn["hazards"] = [h for h in self.scn["hazards"] if not h.get("manual")]
            self.scn_rev += 1
            self._start_session(mission=False)
            self.message = "Reset: drone on the pad, clock at zero, link healthy. Press Launch."
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
            if name == "reset":
                return self.reset()
            if name == "pause":
                return self.set_pause(not self.paused if arg is None else arg)
            if g is None:
                return {"ok": False, "error": "No scenario yet."}
            if name == "rtl":
                if self.phase != "MISSION":
                    return {"ok": False, "error": "Launch the mission first: the drone is still on the pad."}
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
                return self.popup_now(hidden=False)
            elif name == "popup_hidden":
                return self.popup_now(hidden=True)
            elif name == "edit":
                return self.edit(arg or {})
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

    def popup_now(self, hidden=False):
        """Put a HIGH-concentration ash plume on the route ahead of the drone's real
        position (truth). Reported: the ground station learns it through its
        advisory feed and uplinks it. Hidden: it never reaches the feed, so only
        the drone's own forward sensor can find it."""
        o = self.drone_obs
        if not o or not o.get("route") or self.scn is None or o.get("state") not in ("ENROUTE", "RTL", "RTLINK"):
            return {"ok": False, "error": "The drone is not flying a route right now."}
        route, seg = o["route"], o.get("seg", 0)
        want = int(self.profile["lookahead"]) + (4 if not hidden else 3)
        protected = [tuple(self.scn["home"][:2])] + [tuple(t[:2]) for t in self.scn["targets"]] + [tuple(o["cell"][:2])]
        t = self.now()
        hz, ahead = None, None
        # try the preferred distance first, then nearby distances, until the plume
        # covers neither the drone, home nor a waypoint
        for k in sorted(range(3, len(route) - seg - 1), key=lambda k: (abs(k - want), k)):
            c = route[seg + k]
            x0, y0 = max(0, c[0] - 1), max(0, c[1] - 1)
            if any(x0 <= px <= x0 + 2 and y0 <= py <= y0 + 2 for px, py in protected):
                continue
            z0, z1 = max(0, c[2] - 2), min(self.scn["layers"] - 1, c[2] + 2)
            hid = f"A{self.scn['next_hazard_id']}"
            hz = box(hid, "ash", x0, y0, 3, 3, z0, z1, t_on=round(t, 2), sev=3, conc_mg=5.0,
                     source="ADVISORY", manual=True, unreported=bool(hidden))
            ahead = k
            break
        if hz is None:
            return {"ok": False, "error": "No room for a plume on the rest of this route (too close to a waypoint)."}
        self.scn["next_hazard_id"] += 1
        self.scn["hazards"].append(hz)
        self.scn_rev += 1
        send_json(self.s_obs, {"cmd": "add_hazard", "hz": hz}, self.drone_ctl)
        how = "not reported to anyone: the drone's sensor must find it" if hidden else "reported by the advisory feed"
        self.gcs.log("warn", "INJECT", f"Ash plume {hid} (HIGH, ≥ 4 mg/m³) formed {ahead} cells ahead of the drone, {how}")
        return {"ok": True, "id": hid}

    # ---------------------------------------------------------- map editing
    ZONE_LEVEL = {"ash": {1: 1.0, 2: 3.0, 3: 6.0}, "rain": {1: 8.0, 2: 15.0, 3: 25.0}}

    def edit(self, a):
        """Map editing from the console. Start and goal can be moved before
        launch; zones can be added or removed any time (during flight they
        appear at once in the truth, and reach the drone by advisory unless
        marked unreported)."""
        if self.scn is None:
            return {"ok": False, "error": "Generate a scenario first."}
        scn, tool = self.scn, a.get("tool")
        try:
            x, y = int(round(float(a["x"]))), int(round(float(a["y"])))
        except (KeyError, TypeError, ValueError):
            return {"ok": False, "error": "Click inside the map."}
        if not (0 <= x < scn["cols"] and 0 <= y < scn["rows"]):
            return {"ok": False, "error": "Click inside the map."}
        flying = self.phase == "MISSION" and (self.drone_obs or {}).get("state") not in (None, "IDLE", "LANDED", "COMPLETE", "FORCED")
        t = self.now() if self.phase == "MISSION" else 0.0
        static = grid_for(scn, scn["static"], self.profile, horizon=0.0)
        if tool in ("start", "goal"):
            if flying:
                return {"ok": False, "error": "Move start and goal before launch (press RESET first)."}
            if tool == "start":
                if not static.free((x, y, 0)):
                    return {"ok": False, "error": "The start cannot be inside an obstacle."}
                if any(tuple(tg[:2]) == (x, y) for tg in scn["targets"]):
                    return {"ok": False, "error": "Start and goal must differ."}
                scn["home"] = [x, y, 0]
                scn["gcs"] = [x, y, scn["gcs"][2]]
            else:
                z = scn["targets"][-1][2]
                if not static.free((x, y, z)):
                    return {"ok": False, "error": "The goal cannot be inside an obstacle."}
                if (x, y) == tuple(scn["home"][:2]):
                    return {"ok": False, "error": "Start and goal must differ."}
                scn["targets"][-1] = [x, y, z]
            return self._edited(f"{'Start' if tool == 'start' else 'Goal'} moved to ({x}, {y}).")
        if tool == "zone":
            ztype = a.get("type", "ash")
            if ztype not in ("ash", "turb", "gust", "rain"):
                return {"ok": False, "error": "Unknown zone type."}
            sev = max(1, min(3, int(a.get("sev", 3))))
            size = max(1, min(4, int(a.get("size", 2))))
            hid = f"U{scn['next_hazard_id']}"
            kw = {"source": "OPERATOR", "user": True, "t_on": round(t, 2)}
            if ztype == "ash":
                kw.update(sev=sev, conc_mg=self.ZONE_LEVEL["ash"][sev])
            elif ztype == "turb":
                kw.update(sev=sev)
            elif ztype == "gust":
                kw.update(gust_kt=round(float(self.profile["gust_limit_kt"]) + (sev - 2) * 8 + 2))
            else:
                kw.update(rain_mmh=self.ZONE_LEVEL["rain"][sev])
            if a.get("hidden"):
                kw["unreported"] = True
            if flying:
                kw["manual"] = True
            z0, z1 = 0, scn["layers"] - 1
            if a.get("shape", "circle") == "circle":
                hz = circle(hid, ztype, x, y, size + 0.2, z0, z1, **kw)
            else:
                hz = box(hid, ztype, x - (size - 1), y - (size - 1), 2 * size - 1, 2 * size - 1, z0, z1, **kw)
            if a.get("drift"):
                vx, vy = drift_velocity(scn["mode"], scn["wind"], 0.5)
                hz.update(vx=vx, vy=vy, t_ref=round(t, 2))
            home = scn["home"]
            if P.zone_contains_xy(hz, home[0], home[1], t) and not flying:
                return {"ok": False, "error": "Keep zones off the start point (the drone would take off into it)."}
            scn["next_hazard_id"] += 1
            scn["hazards"].append(hz)
            label = f"{C.HAZARD_LABEL.get(ztype, ztype)} {hid} placed at ({x}, {y})" + (", drifting" if a.get("drift") else "") + \
                    (", unreported" if a.get("hidden") else "")
            if flying:
                self.scn_rev += 1
                send_json(self.s_obs, {"cmd": "add_hazard", "hz": hz}, self.drone_ctl)
                self.gcs.log("warn", "INJECT", label)
                return {"ok": True, "id": hid}
            return self._edited(label + ".")
        if tool == "erase":
            hits = [h for h in scn["hazards"] if h.get("t_on", 0.0) <= t < h.get("t_off", 1e9)
                    and P.zone_contains_xy(h, x, y, t)]
            if not hits:
                return {"ok": False, "error": "No weather zone there."}
            h = hits[-1]
            scn["hazards"] = [z for z in scn["hazards"] if z["id"] != h["id"]]
            if flying:
                self.scn_rev += 1
                send_json(self.s_obs, {"cmd": "remove_hazard", "id": h["id"]}, self.drone_ctl)
                self.gcs.log("info", "INJECT", f"Zone {h['id']} removed")
                return {"ok": True}
            return self._edited(f"Zone {h['id']} removed.")
        return {"ok": False, "error": "Pick a tool first."}

    def _edited(self, what):
        st = self.scn.setdefault("story", {})
        st["edited"] = True
        if "edited by you" not in " ".join(st.get("verified", [])):
            st["verified"] = list(st.get("verified", [])) + ["map edited by you: not pre-checked in the simulator"]
        self.scn_rev += 1
        self._compare_cache = None
        self._start_session(mission=False)
        self.message = what + " Pre-flight re-planned. Press Launch."
        return {"ok": True}

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
                if self.gcs is not None and not self.paused:
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
                    if not self.paused:
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
            self._drone_paused = bool(m.get("paused"))
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
                   "ports": self.ports, "tour": self.tour.view(), "impair": self.impair, "paused": self.paused}
            if self.scn is not None and scn_rev != self.scn_rev:
                s = self.scn
                out["scn"] = {k: s.get(k) for k in ("mode", "cols", "rows", "layers", "home", "targets", "gcs",
                                                     "static", "hazards", "wind", "story", "script", "seed", "soc")}
                out["scn"]["layer_ft"] = MODES[s["mode"]]["layer_ft"]
                out["scn"]["cell_m"] = MODES[s["mode"]]["cell_m"]
            if self.gcs is not None:
                out["gcs"] = self.gcs.view()
                out["gcs"]["ep"] = dict(self.ep.stats)
                out["gcs"]["pending"] = self.ep.pending()
                out["gcs"]["gaps"] = self.ep.gaps[-6:]
            out["drone"] = self.drone_obs
            out["actions"] = [round(a[0], 1) for a in self.actions]
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

    def report(self):
        """Mission report for the Reports tab. Every figure is from this simulated run."""
        with self.lock:
            g, d = self.gcs, self.drone_obs or {}
            if g is None or self.scn is None:
                return None
            m = d.get("metrics", {})
            ms = m.get("replan_ms") or []
            mode = MODES[self.scn["mode"]]
            known, sensed = set(d.get("known") or []), set(d.get("sensed") or [])
            hz = []
            for h in self.scn["hazards"]:
                if h["type"] != "ash":
                    continue
                band = C.ASH_BANDS[int(h.get("sev", 3))]
                how = ("own sensor" if h["id"] in sensed else "uplink (advisory feed)" if h["id"] in known
                       else "not encountered")
                hz.append({"id": h["id"], "band": band["name"], "range": band["range"], "conc_mg": h.get("conc_mg"),
                           "hidden": bool(h.get("unreported")), "t_on": h.get("t_on", 0.0), "known_by": how,
                           "detect_cells": (m.get("detect_cells") or {}).get(h["id"])})
            ev = [dict(e, who="GCS") for e in g.events] + [dict(e, who="DRN") for e in d.get("events", [])]
            key = {"LAUNCH", "SENSED", "BENDYRULER", "ASH_EDGE", "ASH_ENCOUNTER", "ESCAPE", "ESCAPED", "ASH_CLEAR",
                   "ASH_DAMAGE", "AIRSPEED", "SO2", "HOLD_PLAN", "HOLDING", "HOLD_CLEAR", "HOLD_EXIT", "DIVERT", "REPLANNED", "THREAT", "RTL", "LANDED", "COMPLETE", "FAILSAFE",
                   "LINK_LOST", "LINK_BACK", "BLACKBOX", "BINGO", "REJECTED", "ACCEPTED", "RELAY_KILLED", "RELAY_UP",
                   "INJECT", "WX_NEW", "DRONE_REPORT", "FORCED_LANDING", "RTL_IGNORED", "RESUME"}
            timeline = sorted([e for e in ev if e["code"] in key], key=lambda e: e["t"])[-40:]
            counts = self.air.get("counts") or {}
            cells = m.get("dist") or 0.0
            return {
                "note": "Simulated run on this laptop. Illustrative values, not flight data.",
                "profile": self.profile, "cfg": self.cfg, "story": self.scn.get("story"),
                "phase": self.phase, "t": round(self.now(), 1),
                "state": d.get("state"), "failsafe": d.get("failsafe"),
                "distance_km": round(cells * mode["cell_m"] / 1000.0, 2),
                "energy_left": d.get("energy"), "energy0": d.get("energy0"), "reserve": d.get("reserve"),
                "replans": m.get("replans", 0), "bendy": m.get("bendy", 0), "algo": m.get("algo", {}),
                "replan_ms_median": sorted(ms)[len(ms) // 2] if ms else None, "replan_ms_max": max(ms) if ms else None,
                "field_ms": (m.get("field_ms") or [None])[-1],
                "sensed": m.get("sensed", 0), "exposures": m.get("exposures", 0),
                "blocked_entries": m.get("blocked_entries", 0), "failsafes": m.get("failsafes", 0),
                "rtl_cmds": m.get("rtl_cmds", 0), "escapes": m.get("escapes", 0), "holds": m.get("holds", 0),
                "hold_s": round(m.get("hold_s", 0.0), 1), "ash_s": round(m.get("ash_s", 0.0), 1),
                "health": d.get("health"), "dose": d.get("dose"), "flags": d.get("flags", []),
                "sensors": d.get("sensors"), "fouling": d.get("fouling"),
                "outages": [{k: o.get(k) for k in ("t0", "t1", "err", "recovered", "bb_records")} for o in g.outages],
                "uploads": g.uploads, "ash": hz, "weather": g.wx, "timeline": timeline,
                "packets": counts, "gcs_ep": dict(self.ep.stats), "drone_ep": d.get("ep"),
            }

    def compare(self):
        """Run every planner on the same map (what is known at this moment) and
        the same A* with SAR on and off. Times are from this laptop."""
        with self.lock:
            if self.scn is None or self.profile is None:
                return None
            scn, prof, base = self.scn, dict(self.profile), dict(self.base_profile or self.profile)
            t = self.now() if self.phase == "MISSION" else 0.0
            hz = scn["static"] + [h for h in scn["hazards"] if h.get("t_on", 0.0) <= t]
            start = tuple((self.drone_obs or {}).get("cell") or scn["home"])
            targets = [tuple(c) for c in scn["targets"]]
        grid = grid_for(scn, hz, prof)
        rows = []
        for name, informed in (("A*", True), ("Dijkstra", False)):
            r, _ = P.plan_through(grid, start, targets, prof["budget"] * 10, informed=informed)
            rows.append({"algo": name, "ok": r.ok, "ms": round(r.ms, 2), "nodes": r.expansions,
                         "cost": None if r.cost is None else round(r.cost, 2), "cells": len(r.path or [])})
        f = P.CostField(grid, scn["home"], prof["budget"] * 10)
        rows.append({"algo": "Dijkstra cost-to-home field", "ok": f.cost(start) is not None, "ms": round(f.ms, 2),
                     "nodes": f.expansions, "cost": None if f.cost(start) is None else round(f.cost(start), 2),
                     "cells": len(f.path(start) or []), "note": f"answers 'can I get home?' for all {len(f.dist)} reachable cells"})
        local, info = P.bendy_ruler(grid, start, targets[0], length=int(prof.get("br_ruler", 3)))
        rows.append({"algo": "BendyRuler (one decision)", "ok": local is not None, "ms": round(info["ms"], 3),
                     "nodes": info["probes"], "cost": None, "cells": len(local or []),
                     "note": f"probes {info['probes']} rulers, picks {info['choice'] or 'none'}; local only, not optimal"})
        sar = []
        for label, p in (("SAR off", dict(base, sar=False)), ("SAR on", C.apply_sar(dict(base)))):
            gg = grid_for(scn, hz, p)
            r, _ = P.plan_through(gg, start, targets, p["budget"] * 10)
            sar.append({"mode": label, "ok": r.ok, "cost": None if r.cost is None else round(r.cost, 2),
                        "cells": len(r.path or []), "blocked_cells": sum(gg.blocked), "reason": r.reason})
        return {"t": round(t, 1), "from": list(start), "rows": rows, "sar": sar, "sar_rules": C.SAR_RULES}


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
        {"title": "1 · Inside unreported ash", "intro": "Mid-flight an ash column forms around the drone. No advisory has it. Its OPC and SO₂ sensors only sample the air it is in.",
         "cfg": {"mode": air, "preset": "Sentinel  MALE", "script": "escape", "seed": 1, "weather": "breezy"},
         "linger": 6.0,
         "steps": [
             {"when": dcode("ASH_ENCOUNTER"), "say": "OPC: the drone is inside volcanic ash",
              "sub": "Particle counter saturates; SO₂ lags behind (its cell takes up to 60 s to respond)."},
             {"when": dcode("ESCAPED"), "say": "Out: a 180° turn back along its own track to clean air, never climbing",
              "sub": "Right screen: engine health and the pitot warning show the modelled ash wear."},
             {"when": gcode("DRONE_REPORT"), "say": "The drone reports the plume back to the ground station",
              "sub": "Left screen: the estimated plume appears, labelled as a drone report."},
         ]},
        {"title": "1b · Ash over the landing site", "intro": "Ash fall settles over the goal while the drone is on its way.",
         "cfg": {"mode": small, "preset": "Swift-V  VTOL fixed-wing", "script": "holding", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": dcode("HOLD_PLAN"), "say": "It cannot land in ash: re-planned to a clear holding fix nearby", "sub": ""},
             {"when": dcode("HOLDING"), "say": "Holding pattern: orbiting until the landing site clears", "sub": ""},
             {"when": dcode("HOLD_CLEAR"), "say": "Landing site clear: leaving the hold to land", "sub": ""},
         ]},
        {"title": "2 · Upload over a real UDP link", "intro": "Ground station plans around known hazards, then uploads over UDP to a separate drone process.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "nominal", "seed": 1, "weather": "breezy"},
         "steps": [
             {"hold": 0, "when": gcode("UPLOAD"), "say": "Mission packed into numbered, CRC-checked datagrams",
              "sub": "Watch the strip below: each tick is one packet crossing the air relay."},
             {"when": lambda rt: _upload_status(rt) == "ACCEPTED", "say": "Drone validated the route against its own limits and accepted it",
              "sub": "Left: the ground station, drawn only from packets it received. Right: the drone's own computer."},
             {"when": at(8), "say": "Telemetry streams back at 5 Hz; heartbeats keep both watchdogs fed",
              "sub": "Process IDs on top: three separate programs talking over UDP."},
         ]},
        {"title": "3 · Ash advisory mid-flight", "intro": "A volcanic ash advisory will be issued ahead of the drone, beyond its own sensor range.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "popup", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": lambda rt: _t(rt) >= (story("t_on", 999)(rt) or 999), "say": "An ash plume just formed ahead of the drone",
              "sub": "The advisory feed will report it about one second later."},
             {"when": gcode("WX_NEW"), "say": "Ground station uplinks the new ash advisory", "sub": ""},
             {"when": dcode("REPLANNED"), "say": detect_text, "sub": "Old route dashed, new route solid. Replan time is shown on the drone screen."},
         ]},
        {"title": "4 · Radio shadow: flying blind", "intro": "A tower block stands between the antenna and part of the route. Nobody touches anything.",
         "cfg": {"mode": small, "preset": "Scout-Q  light quadcopter", "script": "shadow", "seed": 2, "weather": "breezy"},
         "linger": 7.0,
         "steps": [
             {"when": gcode("LINK_LOST"), "say": "Line of sight blocked: the ground station has lost the drone",
              "sub": "Left screen now shows a dead-reckoned ghost with a growing uncertainty ring. It is a guess, not the truth."},
             {"when": dcode("FAILSAFE"), "say": "The drone runs its own failsafe",
              "sub": "Right screen: what the drone actually does while nobody can hear it."},
             {"when": gcode("BLACKBOX"), "say": outage_text, "sub": "Compare the ghost path with the recovered track."},
         ]},
        {"title": "5 · Antenna pulled", "intro": "Mid-flight the air relay process is killed. There is no radio path at all.",
         "cfg": {"mode": small, "preset": "Swift-V  VTOL fixed-wing", "script": "antenna", "seed": 1, "weather": "breezy"},
         "linger": 6.0,
         "steps": [
             {"hold": 0, "when": gcode("RELAY_KILLED"),
              "say": "Antenna pulled: the relay process was terminated, no radio path exists",
              "sub": "Its PID turns grey at the top. Packets now go nowhere."},
             {"when": dcode("FAILSAFE"), "say": "Swift-V's policy is CONTINUE: it finishes the job on its last known map",
              "sub": "Different drones carry different failsafe policies."},
             {"hold": 0, "when": gcode("RELAY_UP"),
              "say": "Antenna back: relay restarted as a new process", "sub": "Watch the black box fill the gap on the left."},
         ]},
        {"title": "6 · Bad link and an unsafe upload", "intro": "30% uplink loss, 12% corruption. First the operator sends a route drawn straight through a hazard.",
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
        {"title": "7 · Aircraft mode: ash advisory", "intro": "Same software, aircraft scale: 0 to 40,000 ft, 125 km across.",
         "cfg": {"mode": air, "preset": "Sentinel  MALE", "script": "popup", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": lambda rt: _t(rt) >= (story("t_on", 999)(rt) or 999), "say": "Volcanic ash advisory issued ahead",
              "sub": "Ash is a hard exclusion for every drone class."},
             {"when": dcode("REPLANNED"), "say": detect_text, "sub": "Sentinel tolerates moderate turbulence but never ash."},
         ]},
        {"title": "8 · Detour forces a low-energy return", "intro": "Battery at partial charge. A detour will make the mission unaffordable.",
         "cfg": {"mode": small, "preset": "Hauler-H  heavy-lift hexacopter", "script": "bingo", "seed": 1, "weather": "breezy"},
         "steps": [
             {"when": dcode("REPLANNED"), "say": "Detour around a new hazard costs energy", "sub": "Watch the energy bar on the drone screen."},
             {"when": dcode("BINGO"), "say": "Bingo: the drone turns home on its own with its reserve intact",
              "sub": "It keeps going while it can still finish, and turns back at the point of no return."},
         ]},
    ]
