"""Hazard zones at real GPS coordinates, loaded from a JSON file.

Zone file format (see examples/):

{
  "wind": {"from_deg": 270, "speed_ms": 6},          # optional; drives plumes and drift
  "zones": [
    {"id": "TFR1", "type": "tfr", "shape": "circle", "lat": .., "lon": .., "radius_m": 250,
     "alt_min_m": 0, "alt_max_m": 400},
    {"id": "GF1", "type": "geofence", "shape": "polygon", "points": [[lat, lon], ...]},
    {"id": "A1", "type": "ash", "shape": "circle", "lat": .., "lon": .., "radius_m": 150,
     "conc_mg": 6, "drift": true},
    {"id": "V1", "type": "ash", "shape": "plume", "lat": .., "lon": .., "conc_mg": 8,
     "r0_m": 60, "spread": 0.12, "decay_m": 1500},
    {"id": "T1", "type": "turbulence", "sev": 2, ...},
    {"id": "B1", "type": "obstacle", ...}
  ]
}

Optional per zone: "alt_min_m"/"alt_max_m" (relative to home, default 0..10000),
"reported": false (hidden from the planner; only the onboard sensor can find it),
"vn"/"ve" (drift m/s) or "drift": true (move with the file's wind),
"at_s": N (the coordinates are the forecast position N seconds after the file is loaded).

Altitudes are metres above the home position, matching the autopilot's
relative-altitude frame.
"""
import json
import math
import os
import socket
import time

from .geo import LocalFrame, wind_to_ne

TYPES = ("ash", "tfr", "geofence", "obstacle", "turbulence")
# EASA SIB 2010-17R4 ash concentration areas (mg/m^3)
ASH_LOW, ASH_MED, ASH_HIGH = 0.2, 2.0, 4.0


def ash_severity(conc):
    if conc >= ASH_HIGH:
        return 3
    if conc >= ASH_MED:
        return 2
    if conc >= ASH_LOW:
        return 1
    return 0


def _seg_dist(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    L2 = vx * vx + vy * vy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / L2))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def _in_poly(x, y, pts):
    inside = False
    j = len(pts) - 1
    for i in range(len(pts)):
        xi, yi = pts[i]
        xj, yj = pts[j]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi:
            inside = not inside
        j = i
    return inside


class Zone:
    """One hazard in the local frame. Geometry is in metres (n, e)."""

    def __init__(self, d, frame, wind=None, t_ref=None):
        self.raw = dict(d)
        self.id = str(d["id"])
        self.type = d.get("type", "obstacle")
        if self.type not in TYPES:
            raise ValueError(f"zone {self.id}: unknown type {self.type!r} (use one of {TYPES})")
        self.shape = d.get("shape", "circle")
        self.alt_min = float(d.get("alt_min_m", 0.0))
        self.alt_max = float(d.get("alt_max_m", 10000.0))
        self.reported = bool(d.get("reported", True))
        self.estimated = bool(d.get("estimated", False))
        # "at_s": the coordinates are where the zone will be N s after loading (a forecast position)
        self.t_ref = float(d.get("t_ref", (t_ref if t_ref is not None else time.time()) + float(d.get("at_s", 0.0))))
        wn, we = wind_to_ne(wind["from_deg"], wind["speed_ms"]) if wind else (0.0, 0.0)
        if "vn" in d or "ve" in d:
            self.vn, self.ve = float(d.get("vn", 0.0)), float(d.get("ve", 0.0))
        elif d.get("drift") and self.shape != "plume":
            self.vn, self.ve = wn, we
        else:
            self.vn, self.ve = 0.0, 0.0
        self.conc = float(d.get("conc_mg", 0.0))
        self.sev = int(d.get("sev", ash_severity(self.conc) if self.type == "ash" else 3))
        self.circles = []                                   # (n, e, r, conc) for circle / plume
        self.poly = None
        if self.shape == "circle":
            n, e = frame.to_ne(d["lat"], d["lon"])
            self.circles = [(n, e, float(d["radius_m"]), self.conc)]
        elif self.shape == "polygon":
            self.poly = [frame.to_ne(p[0], p[1]) for p in d["points"]]
            if len(self.poly) < 3:
                raise ValueError(f"zone {self.id}: polygon needs at least 3 points")
        elif self.shape == "plume":
            self._build_plume(d, frame, wind or {"from_deg": 270.0, "speed_ms": 5.0})
        else:
            raise ValueError(f"zone {self.id}: unknown shape {self.shape!r}")

    def _build_plume(self, d, frame, wind):
        """Steady volcanic plume: circles carried downwind from the vent, spreading
        (r = r0 + spread*x) and thinning (c = c0 (r0/r)^2 exp(-x/decay)) until the
        concentration drops below the EASA LOW threshold. Kinematic model, not a
        dispersion model; the shape is realistic, the numbers are illustrative."""
        n0, e0 = frame.to_ne(d["lat"], d["lon"])
        un, ue = wind_to_ne(wind["from_deg"], 1.0)
        r0 = float(d.get("r0_m", 60.0))
        spread = float(d.get("spread", 0.12))
        decay = float(d.get("decay_m", 1500.0))
        c0 = self.conc or 6.0
        x = 0.0
        while x < 50000:
            r = r0 + spread * x
            c = c0 * (r0 / r) ** 2 * math.exp(-x / decay)
            if c < ASH_LOW:
                break
            self.circles.append((n0 + un * x, e0 + ue * x, r, c))
            x += max(10.0, 0.6 * r)
        self.sev = ash_severity(c0)

    # ------------------------------------------------------------------ geometry
    def offset(self, t):
        dt = (t - self.t_ref) if t is not None else 0.0
        return self.vn * dt, self.ve * dt

    def in_alt(self, alt, margin=0.0):
        return self.alt_min - margin <= alt <= self.alt_max + margin

    def distance(self, n, e, t=None):
        """Signed horizontal distance to the zone edge (negative inside)."""
        dn, de = self.offset(t)
        n, e = n - dn, e - de
        if self.poly:
            d = min(_seg_dist(n, e, *self.poly[i], *self.poly[(i + 1) % len(self.poly)])
                    for i in range(len(self.poly)))
            return -d if _in_poly(n, e, self.poly) else d
        return min(math.hypot(n - cn, e - ce) - r for cn, ce, r, _ in self.circles)

    def conc_at(self, n, e, alt, t=None, edge_m=30.0):
        """Ash concentration (mg/m^3) a point sensor would read. Inside the core it
        is the zone value; outside it falls off with an e-folding length edge_m
        (diffuse plume edge: our modelling assumption)."""
        if self.type != "ash" or not self.in_alt(alt):
            return 0.0
        dn, de = self.offset(t)
        n, e = n - dn, e - de
        if self.poly:
            d = self.distance(n + dn, e + de, t)
            return self.conc if d <= 0 else self.conc * math.exp(-d / edge_m)
        best = 0.0
        for cn, ce, r, c in self.circles:
            d = math.hypot(n - cn, e - ce) - r
            best = max(best, c if d <= 0 else c * math.exp(-d / edge_m))
        return best

    def bounds(self, t=None, horizon=0.0):
        """(n_min, n_max, e_min, e_max) over [t, t + horizon]."""
        pts = self.poly or [(cn - r, ce - r) for cn, ce, r, _ in self.circles] + \
            [(cn + r, ce + r) for cn, ce, r, _ in self.circles]
        out = None
        for tt in (t, None if t is None else t + horizon):
            dn, de = self.offset(tt)
            ns = [p[0] + dn for p in pts]
            es = [p[1] + de for p in pts]
            b = (min(ns), max(ns), min(es), max(es))
            out = b if out is None else (min(out[0], b[0]), max(out[1], b[1]), min(out[2], b[2]), max(out[3], b[3]))
        return out

    def describe(self):
        kind = {"ash": f"ash {['', 'LOW', 'MEDIUM', 'HIGH'][self.sev]}", "tfr": "TFR", "geofence": "geofence",
                "obstacle": "obstacle", "turbulence": f"turbulence sev {self.sev}"}[self.type]
        return f"{self.id} ({kind})"


def blocks(zone, profile):
    """Does this zone block this vehicle? Returns (blocked, soft_cost)."""
    sar = profile.get("sar", False)
    if zone.type in ("geofence", "obstacle"):
        return True, 0.0
    if zone.type == "tfr":
        return (False, 6.0) if sar else (True, 0.0)
    if zone.type == "ash":
        if zone.sev >= 2:
            return True, 0.0
        return (False, 8.0) if sar else (True, 0.0)
    if zone.type == "turbulence":
        return (True, 0.0) if zone.sev >= profile.get("turb_block_sev", 2) else (False, 3.0)
    return True, 0.0


class ZoneStore:
    """Holds the hazard picture: the zone file (reloaded when it changes),
    advisories pushed over UDP, and zones the vehicle estimated itself."""

    def __init__(self, path, frame, advisory_port=None):
        self.path, self.frame = path, frame
        self.version = 0
        self.wind = None
        self.file_zones, self.pushed, self.estimated = [], {}, {}
        self._mtime = None
        self.sock = None
        if advisory_port:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", int(advisory_port)))
            self.sock.setblocking(False)
        if path:
            self.reload(force=True)

    def _parse(self, items):
        return [Zone(z, self.frame, self.wind) for z in items]

    def reload(self, force=False):
        """Re-read the zone file if it changed. Returns True on change. A broken
        file keeps the previous picture (never drop hazards on a typo)."""
        if not self.path:
            return False
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            return False
        if not force and m == self._mtime:
            return False
        self._mtime = m
        try:
            with open(self.path) as f:
                data = json.load(f)
            self.wind = data.get("wind")
            zones = self._parse(data.get("zones", []))
        except (ValueError, KeyError, TypeError) as exc:
            if force:
                raise
            print(f"[janus] zone file not reloaded ({exc}); keeping the previous zones")
            return False
        self.file_zones = zones
        self.version += 1
        return True

    def poll_advisories(self):
        """Non-blocking read of JSON advisories: {"op": "add"|"remove", "zone": {...}}
        or {"op": "remove", "id": "A2"}. Returns True on change."""
        if not self.sock:
            return False
        changed = False
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                break
            try:
                msg = json.loads(data)
                if msg.get("op") == "remove":
                    changed |= self.pushed.pop(str(msg.get("id")), None) is not None
                else:
                    z = Zone(msg["zone"], self.frame, msg.get("wind") or self.wind)
                    self.pushed[z.id] = z
                    changed = True
            except (ValueError, KeyError, TypeError) as exc:
                print(f"[janus] bad advisory ignored: {exc}")
        if changed:
            self.version += 1
        return changed

    def add_estimate(self, zone):
        self.estimated[zone.id] = zone
        self.version += 1

    def known(self):
        """What the planner may use: reported zones plus our own estimates."""
        return [z for z in self.file_zones if z.reported] + list(self.pushed.values()) + list(self.estimated.values())

    def truth(self):
        """Everything, including unreported zones: only the virtual sensor reads this."""
        return self.file_zones + list(self.pushed.values())
