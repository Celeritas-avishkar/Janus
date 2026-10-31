"""The air relay: a separate OS process standing in for the radio link.

    GCS --udp--> [air_gcs]  relay  [air_drone] --udp--> drone
    GCS <--udp-- [air_gcs]  relay  [air_drone] <--udp-- drone

Every datagram gets a fate from AirModel (link budget with line-of-sight
shadowing, plus manual loss, latency, corruption, duplication, blackout).
Fates are reported to the ground station's observer port so the UI can draw
the packet strip. Run standalone:  python -m janus.relay --ports '{...}'
"""
import argparse
import heapq
import json
import select
import time

from . import config as C
from .air import AirModel
from .net import udp_socket, recv_all, send_bytes, send_json, parse_ports, pid
from .protocol import peek_header


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default=C.HOST)
    ap.add_argument("--gcs-host", default=C.HOST)
    ap.add_argument("--drone-host", default=C.HOST)
    ap.add_argument("--ports", default=None)
    ap.add_argument("--seed", type=int, default=7)
    a = ap.parse_args()
    P = parse_ports(a.ports)
    s_gcs = udp_socket(a.host, P["air_gcs"])
    s_drone = udp_socket(a.host, P["air_drone"])
    s_ctl = udp_socket(a.host, P["air_ctl"])
    gcs_addr = (a.gcs_host, P["gcs_radio"])
    drone_addr = (a.drone_host, P["drone_radio"])
    obs_addr = (a.gcs_host, P["gcs_obs"])
    air = AirModel(seed=a.seed)
    t0 = time.monotonic()
    queue, n = [], 0
    events = []
    counts = {"up": {}, "dn": {}}
    next_report = 0.0
    cfg_id = None
    print(f"[relay] pid {pid()} air_gcs={P['air_gcs']} air_drone={P['air_drone']} ctl={P['air_ctl']}", flush=True)

    def handle(direction, data, now):
        nonlocal n
        air.drone_pos = air.drone_pos or None
        fate, delay, outs, snr = air.decide(direction, now, data)
        h = peek_header(data) or {"type": "?", "seq": 0, "len": len(data)}
        events.append([round(now, 3), direction, h["type"], fate, h["seq"], h["len"], snr])
        c = counts[direction]
        c[fate] = c.get(fate, 0) + 1
        dest = drone_addr if direction == "up" else gcs_addr
        sock = s_drone if direction == "up" else s_gcs
        for d in outs:
            n += 1
            heapq.heappush(queue, (now + delay, n, sock, d, dest))

    while True:
        now = time.monotonic() - t0
        timeout = 0.01
        if queue:
            timeout = max(0.0, min(timeout, queue[0][0] - now))
        try:
            select.select([s_gcs, s_drone, s_ctl], [], [], timeout)
        except (OSError, ValueError):
            time.sleep(timeout)
        now = time.monotonic() - t0
        for data, addr in recv_all(s_gcs):
            gcs_addr = addr
            handle("up", data, now)
        for data, addr in recv_all(s_drone):
            drone_addr = addr
            handle("dn", data, now)
        for data, addr in recv_all(s_ctl):
            try:
                m = json.loads(data.decode())
            except Exception:
                continue
            cmd = m.get("cmd")
            if cmd == "pos":
                air.drone_pos = m.get("p")
            elif cmd == "config":
                if m.get("id") != cfg_id:
                    counts = {"up": {}, "dn": {}}
                    events = []
                cfg_id = m.get("id")
                air.configure(scn=m.get("scn"), drone_tx_dbm=m.get("tx"), drone_gain_dbi=m.get("gain"),
                              **m.get("settings", {}))
                air.drone_pos = None
                queue.clear()
            elif cmd == "set":
                air.configure(**m.get("settings", {}))
            elif cmd == "quit":
                return
        while queue and queue[0][0] <= now:
            _, _, sock, d, dest = heapq.heappop(queue)
            send_bytes(sock, d, dest)
        if now >= next_report:
            next_report = now + 0.1
            b = air.budget(now) if air.scn is not None and air.drone_pos is not None else None
            send_json(s_ctl, {"kind": "air", "pid": pid(), "cfg": cfg_id, "now": round(now, 3), "ev": events[-300:],
                              "settings": air.s, "counts": counts,
                              "budget": None if b is None else {k: (round(v, 1) if isinstance(v, float) else v)
                                                                for k, v in b.items()}}, obs_addr)
            events = []


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
