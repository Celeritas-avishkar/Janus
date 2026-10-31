"""Onboard ash sensing.

On a real airframe this would read an Alphasense OPC-N3 (SPI) and an SO2-B4
electrochemical cell (via an ISB / ADC). For flight tests without real ash we
use VirtualOPC: it reads the zone file's ground truth (including zones marked
"reported": false, which the planner never sees) at the vehicle's real GPS
position. Swap in a driver with the same read() signature for real hardware.
"""
import random


class AshSensor:
    """Interface: read(n, e, alt, t) -> mass concentration in mg/m^3."""

    def read(self, n, e, alt, t):
        raise NotImplementedError


class VirtualOPC(AshSensor):
    def __init__(self, store, profile, seed=None, noise=0.10):
        self.store, self.p = store, profile
        self.rng = random.Random(seed)
        self.noise = noise

    def read(self, n, e, alt, t):
        c = 0.0
        for z in self.store.truth():
            c = max(c, z.conc_at(n, e, alt, t, self.p["edge_m"]))
        c *= 1.0 + self.rng.gauss(0.0, self.noise)
        c += abs(self.rng.gauss(0.0, 0.01))           # clean-air floor noise
        return max(0.0, min(c, 50.0))
