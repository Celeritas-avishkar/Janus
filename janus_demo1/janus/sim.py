"""Fast headless simulation: GCS brain + drone brain + protocol endpoints +
air model on a virtual clock. Same code paths as the real processes, minus
the sockets. Used by tests, scenario search and the A/B comparison."""
import heapq

from .air import AirModel
from .drone_brain import DroneBrain
from .gcs_brain import GcsBrain
from .node import deliver, pump
from .protocol import Endpoint, peek_header


class Sim:
    def __init__(self, scn, profile, uplink=True, seed=1, dt=0.05, deterministic=False, air=None):
        self.scn, self.p, self.dt, self.t = scn, profile, dt, 0.0
        self.det = deterministic
        self.q, self.n = [], 0
        self.air = AirModel(scn, profile["tx_dbm"], profile["gain_dbi"], seed=seed)
        if air:
            self.air.configure(**air)
        if scn.get("impair"):
            self.air.configure(**scn["impair"])
        self.gcs = GcsBrain(scn, profile)
        self.drone = DroneBrain(scn, profile, uplink_enabled=uplink)
        self.gep = Endpoint(0, lambda b: self._tx("up", b), peer=1)
        self.dep = Endpoint(1, lambda b: self._tx("dn", b), peer=0)
        self.fates = []
        self.relay_alive = True

    def _tx(self, direction, data):
        if not self.relay_alive:
            self.fates.append((self.t, direction, "relay_down"))
            return
        self.air.drone_pos = self.drone.pos
        fate, delay, outs, _snr = self.air.decide(direction, self.t, data, self.det)
        h = peek_header(data) or {}
        self.fates.append((round(self.t, 2), direction, fate, h.get("type")))
        for d in outs:
            self.n += 1
            heapq.heappush(self.q, (self.t + delay, self.n, direction, d))

    def step(self):
        self.t += self.dt
        while self.q and self.q[0][0] <= self.t:
            _, _, direction, data = heapq.heappop(self.q)
            if direction == "up":
                deliver(self.drone, self.dep, data, self.t)
            else:
                deliver(self.gcs, self.gep, data, self.t)
        self.gcs.step(self.dt, self.t)
        self.drone.step(self.dt, self.t)
        pump(self.gcs, self.gep, self.t, gcs=True)
        pump(self.drone, self.dep, self.t)

    def run(self, until, actions=None, stop=None):
        actions = sorted((actions or {}).items())
        while self.t < until:
            while actions and actions[0][0] <= self.t:
                actions.pop(0)[1](self)
            self.step()
            if stop and stop(self):
                break
        return self

    def done(self):
        return self.drone.state in ("COMPLETE", "LANDED", "FORCED")

    def codes(self, who="drone"):
        src = self.drone.events if who == "drone" else self.gcs.events
        return [e["code"] for e in src]
