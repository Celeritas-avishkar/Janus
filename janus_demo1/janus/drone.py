"""Drone agent process. Talks to the ground station only through the radio
socket (via the air relay). The simulation-control socket receives the world
and profile at setup, and the observer stream (10 Hz) feeds the 'drone
onboard' screen. Neither of those is part of the radio link.

Run standalone (e.g. on a second laptop):
    python -m janus.drone --relay-host <gcs-ip> --gcs-host <gcs-ip> --host 0.0.0.0
"""
import argparse
import json
import select
import time

from . import config as C
from .drone_brain import DroneBrain
from .net import udp_socket, recv_all, send_bytes, send_json, parse_ports, pid
from .node import deliver, pump
from .protocol import Endpoint


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=C.HOST)
    ap.add_argument("--relay-host", default=C.HOST)
    ap.add_argument("--gcs-host", default=C.HOST)
    ap.add_argument("--ports", default=None)
    a = ap.parse_args()
    P = parse_ports(a.ports)
    s_radio = udp_socket(a.host, P["drone_radio"])
    s_ctl = udp_socket(a.host, P["drone_ctl"])
    relay_addr = (a.relay_host, P["air_drone"])
    relay_ctl = (a.relay_host, P["air_ctl"])
    obs_addr = (a.gcs_host, P["gcs_obs"])
    print(f"[drone] pid {pid()} radio={P['drone_radio']} ctl={P['drone_ctl']}", flush=True)

    brain, ep, cfg = None, None, None
    paused, pause_mono = False, 0.0
    next_obs_wall = 0.0
    t0 = time.monotonic()
    last = 0.0
    next_obs = 0.0
    session = None

    def new_session(m):
        nonlocal brain, ep, t0, last, next_obs, session, paused
        brain = DroneBrain(m["scn"], m["profile"], uplink_enabled=m.get("uplink", True))
        ep = Endpoint(1, lambda b: send_bytes(s_radio, b, relay_addr), peer=0)
        t0 = time.monotonic()
        last = 0.0
        next_obs = 0.0
        session = m.get("session")
        paused = False

    while True:
        try:
            select.select([s_radio, s_ctl], [], [], 0.02)
        except (OSError, ValueError):
            time.sleep(0.02)
        for data, _addr in recv_all(s_ctl):
            try:
                m = json.loads(data.decode())
            except Exception:
                continue
            cmd = m.get("cmd")
            if cmd == "start":
                cfg = m
                new_session(m)
            elif cmd == "add_hazard" and brain is not None:
                brain.scn["hazards"].append(m["hz"])
            elif cmd == "remove_hazard" and brain is not None:
                brain.scn["hazards"] = [h for h in brain.scn["hazards"] if h["id"] != m.get("id")]
            elif cmd == "pause" and m.get("session") == session:
                if m.get("on") and not paused:
                    paused, pause_mono = True, time.monotonic()
                elif not m.get("on") and paused:
                    t0 += time.monotonic() - pause_mono       # mission time did not move while stopped
                    paused = False
            elif cmd == "stop":
                brain, ep = None, None
            elif cmd == "quit":
                return
        now = (pause_mono if paused else time.monotonic()) - t0
        if brain is None:
            recv_all(s_radio)
            if now >= next_obs:
                next_obs = now + 0.5
                send_json(s_ctl, {"kind": "drone", "pid": pid(), "idle": True}, obs_addr)
            continue
        for data, _addr in recv_all(s_radio):
            deliver(brain, ep, data, now)
        dt = max(0.0, min(0.1, now - last))
        last = now
        if not paused:
            brain.step(dt, now)
            pump(brain, ep, now)
        wall = time.monotonic()                               # observer runs on wall time: keeps screens fed while stopped
        if wall >= next_obs_wall:
            next_obs_wall = wall + 1.0 / C.OBSERVER_HZ
            send_json(s_ctl, {"cmd": "pos", "p": brain.pos}, relay_ctl)
            o = brain.observer()
            o["ep"] = ep.stats
            send_json(s_ctl, {"kind": "drone", "pid": pid(), "session": session, "obs": o, "paused": paused}, obs_addr)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
