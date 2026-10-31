"""The air between ground station and drone: decides each datagram's fate.

Used by the relay process (real UDP) and by the headless simulator, so both
behave the same way. Fates: ok, rf_lost (link budget), dropped (manual loss
slider), blackout (link cut), corrupt (one bit flipped, delivered: the CRC
must catch it), dup (delivered twice).
"""
import random

from . import radio
from .config import MODES


class AirModel:
    def __init__(self, scn=None, drone_tx_dbm=20.0, drone_gain_dbi=2.0, seed=1):
        self.rng = random.Random(seed)
        self.scn = scn
        self.drone_tx, self.drone_gain = drone_tx_dbm, drone_gain_dbi
        self.s = {"rf": True, "loss_up": 0.0, "loss_dn": 0.0, "latency_ms": 20.0, "jitter_ms": 10.0,
                  "corrupt": 0.0, "dup": 0.0, "blackout": False, "cut_up": False, "cut_dn": False}
        self.drone_pos = None
        self.last_budget = None
        self._budget_t = -1.0

    def configure(self, scn=None, drone_tx_dbm=None, drone_gain_dbi=None, **settings):
        if scn is not None:
            self.scn = scn
        if drone_tx_dbm is not None:
            self.drone_tx = float(drone_tx_dbm)
        if drone_gain_dbi is not None:
            self.drone_gain = float(drone_gain_dbi)
        for k, v in settings.items():
            if k in self.s:
                self.s[k] = type(self.s[k])(v) if not isinstance(self.s[k], bool) else bool(v)

    def budget(self, now, deterministic=False):
        if self.scn is None or self.drone_pos is None:
            return None
        if now - self._budget_t < 0.05 and self.last_budget is not None:
            return self.last_budget
        b = radio.budget(MODES[self.scn["mode"]], self.scn["gcs"], self.drone_pos, self.scn["static"],
                         self.drone_tx, self.drone_gain, None if deterministic else self.rng)
        self.last_budget, self._budget_t = b, now
        return b

    def decide(self, direction, now, data, deterministic=False):
        """direction: 'up' (GCS -> drone) or 'dn' (drone -> GCS).
        Returns (fate, delay_s, [datagrams to deliver], snr)."""
        s = self.s
        b = self.budget(now, deterministic)
        snr = None if b is None else round(b["up_snr" if direction == "up" else "dn_snr"], 1)
        if s["blackout"] or (direction == "up" and s["cut_up"]) or (direction == "dn" and s["cut_dn"]):
            return "blackout", 0.0, [], snr
        if s["rf"] and b is not None:
            p = b["up_loss" if direction == "up" else "dn_loss"]
            if deterministic:
                if p > 0.5:
                    return "rf_lost", 0.0, [], snr
            elif self.rng.random() < p:
                return "rf_lost", 0.0, [], snr
        manual = s["loss_up"] if direction == "up" else s["loss_dn"]
        if manual > 0 and (self.rng.random() < manual):
            return "dropped", 0.0, [], snr
        delay = max(0.0, (s["latency_ms"] + self.rng.uniform(-1, 1) * s["jitter_ms"]) / 1000.0)
        if s["corrupt"] > 0 and self.rng.random() < s["corrupt"] and len(data) > 8:
            buf = bytearray(data)
            i = self.rng.randrange(len(buf))
            buf[i] ^= 1 << self.rng.randrange(8)
            return "corrupt", delay, [bytes(buf)], snr
        if s["dup"] > 0 and self.rng.random() < s["dup"]:
            return "dup", delay, [data, data], snr
        return "ok", delay, [data], snr
