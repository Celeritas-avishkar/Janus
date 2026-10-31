"""Seeded scenario generation, ground truth and the WeeWX-style weather feed.

A scenario is plain JSON so it can be sent to the drone and relay processes
over the simulation-control channel. Truth = static map + weather hazards
whose appearance time t_on has passed.
"""
import copy
import math
import random

from .config import COLS, ROWS, LAYERS, MODES
from . import planner as P
from . import radio

WEATHER_PROFILES = ["calm", "breezy", "stormy"]


# ---------------------------------------------------------------- helpers
def box(hid, htype, x0, y0, w, h, z0, z1, **kw):
    d = {"id": hid, "type": htype, "x0": int(x0), "x1": int(x0 + w - 1), "y0": int(y0),
         "y1": int(y0 + h - 1), "z0": int(z0), "z1": int(z1), "t_on": 0.0}
    d.update(kw)
    return d


def covers(h, x, y):
    return h["x0"] <= x <= h["x1"] and h["y0"] <= y <= h["y1"]


def truth_hazards(scn, t):
    return [h for h in scn["hazards"] if h.get("t_on", 0.0) <= t < h.get("t_off", 1e9)]


def truth_all(scn, t):
    return scn["static"] + truth_hazards(scn, t)


def wind_tuple(scn):
    return P.wind_vector(scn["wind"]["dir"])


def grid_for(scn, hazards, profile):
    return P.Grid(scn["cols"], scn["rows"], scn["layers"], hazards, profile, wind=wind_tuple(scn))


def alt_label(scn, z):
    ft = MODES[scn["mode"]]["layer_ft"] * z
    return f"{ft:,.0f} ft"


# ---------------------------------------------------------------- generation
def base_scenario(seed, mode, weather="breezy", script="nominal"):
    rng = random.Random(f"{seed}:{mode}:{weather}:{script}")
    m = MODES[mode]
    cruise = rng.randint(2, 4) if mode == "small" else rng.randint(2, 4)
    hy = rng.randint(3, ROWS - 4)
    home = [1, hy, 0]
    gy = rng.randint(2, ROWS - 3)
    goal = [COLS - 2, gy, 1]
    targets = []
    n_wp = rng.choice([0, 1, 1, 2])
    xs = sorted(rng.sample(range(7, COLS - 6), n_wp))
    for x in xs:
        targets.append([x, rng.randint(1, ROWS - 2), cruise])
    targets.append(goal)

    reserved = {(home[0], home[1])} | {(t[0], t[1]) for t in targets}
    # keep a 1-cell ring around home and targets free so missions are always possible
    ring = {(x + dx, y + dy) for (x, y) in reserved for dx in (-1, 0, 1) for dy in (-1, 0, 1)}

    def clear_of(x0, y0, w, h):
        return all((x, y) not in ring for x in range(x0, x0 + w) for y in range(y0, y0 + h))

    static, hazards = [], []
    sid = 1
    if mode == "small":
        for _ in range(rng.randint(4, 6)):
            for _a in range(60):
                w, h = rng.choice([(1, 1), (1, 2), (2, 2), (2, 1), (2, 3)])
                x0, y0 = rng.randint(3, COLS - w - 3), rng.randint(0, ROWS - h)
                if clear_of(x0, y0, w, h):
                    top = rng.choice([2, 3, 4, 5, 6, 8])
                    static.append(box(f"B{sid}", "building", x0, y0, w, h, 0, top))
                    sid += 1
                    break
        if rng.random() < 0.8:
            for _a in range(60):
                w, h = rng.choice([(2, 3), (3, 2), (3, 3)])
                x0, y0 = rng.randint(4, COLS - w - 4), rng.randint(0, ROWS - h)
                if clear_of(x0, y0, w, h):
                    static.append(box("GF1", "geofence", x0, y0, w, h, 0, LAYERS - 1, note="No-fly zone"))
                    break
    else:
        for _ in range(rng.randint(2, 4)):
            for _a in range(60):
                w, h = rng.choice([(1, 3), (2, 4), (1, 5), (2, 2)])
                x0, y0 = rng.randint(3, COLS - w - 3), rng.randint(0, ROWS - h)
                if clear_of(x0, y0, w, h):
                    static.append(box(f"R{sid}", "ridge", x0, y0, w, h, 0, rng.choice([1, 1, 2, 3])))
                    sid += 1
                    break
        for _ in range(rng.randint(1, 2)):
            for _a in range(60):
                w, h = rng.choice([(2, 2), (3, 2), (2, 3)])
                x0, y0 = rng.randint(4, COLS - w - 4), rng.randint(0, ROWS - h)
                if clear_of(x0, y0, w, h):
                    z0 = rng.randint(0, 3)
                    static.append(box(f"T{sid}", "tfr", x0, y0, w, h, z0, min(LAYERS - 1, z0 + rng.randint(2, 5)),
                                      note="Temporary flight restriction"))
                    sid += 1
                    break

    level = WEATHER_PROFILES.index(weather) if weather in WEATHER_PROFILES else 1
    base_kt = [rng.uniform(4, 8), rng.uniform(10, 16), rng.uniform(16, 24)][level]
    gust_add = [rng.uniform(2, 5), rng.uniform(6, 10), rng.uniform(10, 16)][level]
    wind = {"dir": float(rng.choice([180, 200, 225, 250, 270, 290, 315, 0, 45, 90, 135])),
            "kt": round(base_kt, 1), "gust": round(base_kt + gust_add, 1)}

    hid = 1
    n_cells = [1, 2, 4][level]
    for _ in range(n_cells):
        for _a in range(60):
            w, h = rng.choice([(2, 2), (3, 2), (2, 3), (3, 3)])
            x0, y0 = rng.randint(3, COLS - w - 3), rng.randint(0, ROWS - h)
            if not clear_of(x0, y0, w, h):
                continue
            if mode == "small":
                lo, hi = [(12, 18), (16, 26), (22, 38)][level]
                z0 = rng.randint(0, 3)
                hazards.append(box(f"W{hid}", "gust", x0, y0, w, h, z0, min(LAYERS - 1, z0 + rng.randint(2, 5)),
                                   gust_kt=round(rng.uniform(lo, hi), 0), source="WEATHER"))
            else:
                sev = rng.choices([1, 2, 3], weights=[[6, 3, 1], [3, 4, 2], [1, 3, 4]][level])[0]
                z0 = rng.randint(1, 5)
                hazards.append(box(f"W{hid}", "turb", x0, y0, w, h, z0, min(LAYERS - 1, z0 + rng.randint(1, 3)),
                                   sev=sev, source="WEATHER"))
            hid += 1
            break
    n_second = [0, 1, 2][level] if mode == "small" else [0, 1, 1][level]
    for _ in range(n_second):
        for _a in range(60):
            w, h = rng.choice([(2, 2), (3, 2), (2, 3)])
            x0, y0 = rng.randint(3, COLS - w - 3), rng.randint(0, ROWS - h)
            if not clear_of(x0, y0, w, h):
                continue
            if mode == "small":
                hazards.append(box(f"W{hid}", "rain", x0, y0, w, h, 0, LAYERS - 1,
                                   rain_mmh=round(rng.uniform(4, 25), 1), source="WEATHER"))
            else:
                z0 = rng.randint(2, 6)
                hazards.append(box(f"W{hid}", "ash", x0, y0, w, h, z0, LAYERS - 1 if rng.random() < 0.3 else min(LAYERS - 1, z0 + 2),
                                   sev=rng.randint(1, 3), source="ADVISORY"))
            hid += 1
            break

    return {
        "seed": seed, "mode": mode, "script": script, "weather_profile": weather,
        "cols": COLS, "rows": ROWS, "layers": LAYERS,
        "home": home, "targets": targets,
        "gcs": [home[0], home[1], radio.GCS_ANTENNA_LAYER],
        "static": static, "hazards": hazards, "wind": wind,
        "feed_latency": 1.0, "soc": 1.0, "story": {}, "impair": {},
        "next_hazard_id": hid,
    }


def initial_plan(scn, profile, known=None):
    known = scn["static"] + truth_hazards(scn, 0.0) if known is None else known
    grid = grid_for(scn, known, profile)
    res, legs = P.plan_through(grid, scn["home"], scn["targets"], profile["budget"])
    return res, legs, grid


# ---------------------------------------------------------------- weather feed (GCS side)
class WeatherFeed:
    """Seeded WeeWX-style feed. Field names follow the WeeWX archive schema
    (outTemp, outHumidity, barometer, windSpeed, windGust, windDir, rainRate);
    units here are metric/knots for display. Hazard volumes become visible to
    the feed feed_latency seconds after they appear in the truth."""

    def __init__(self, scn, seed_extra=0):
        self.scn = scn
        self.rng = random.Random(f"wx:{scn['seed']}:{seed_extra}")
        self.paused = False
        self.last = None
        self.last_good_t = 0.0
        self.temp = self.rng.uniform(24, 29) if scn["mode"] == "small" else self.rng.uniform(22, 30)
        self.baro = self.rng.uniform(1008, 1014)
        self.hum = self.rng.uniform(55, 80)

    def snapshot(self, t):
        if self.paused and self.last is not None:
            s = dict(self.last)
            s["age_s"] = round(t - self.last_good_t, 1)
            s["status"] = "STALE"
            return s
        w = self.scn["wind"]
        jitter = math.sin(t / 7.0) * 6.0 + self.rng.uniform(-3, 3)
        speed = max(0.0, w["kt"] + math.sin(t / 5.0) * 1.5 + self.rng.uniform(-1, 1))
        gust = max(speed, w["gust"] + self.rng.uniform(-2, 2))
        rain_cells = [h for h in truth_hazards(self.scn, t) if h["type"] == "rain"]
        visible = self._visible(t)
        s = {
            "t": round(t, 2), "status": "LIVE", "age_s": 0.0,
            "source": "SIM (WeeWX schema)" if self.scn["mode"] == "small" else "SIM (METAR / advisory schema)",
            "outTemp": round(self.temp + math.sin(t / 30.0) * 0.4, 1),
            "outHumidity": round(self.hum, 0),
            "barometer": round(self.baro - t * 0.002, 1),
            "windSpeed": round(speed, 1), "windGust": round(gust, 1),
            "windDir": round((w["dir"] + jitter) % 360, 0),
            "rainRate": round(max([h.get("rain_mmh", 0) for h in rain_cells] + [0.0]), 1),
            "hazards": [h["id"] for h in visible],
        }
        s["metar"] = metar_line(self.scn, s)
        self.last = s
        self.last_good_t = t
        return s

    def visible_hazards(self, t):
        if self.paused and self.last is not None:
            ids = set(self.last["hazards"])
            return [h for h in self.scn["hazards"] if h["id"] in ids]
        return self._visible(t)

    def _visible(self, t):
        """Hazards present before the mission are already in the feed; new ones
        show up feed_latency seconds after they form."""
        lat = self.scn["feed_latency"]
        return [h for h in self.scn["hazards"]
                if (h.get("t_on", 0.0) <= 0.0 or h.get("t_on", 0.0) + lat <= t) and t < h.get("t_off", 1e9)]


def metar_line(scn, s):
    """A METAR-format line synthesised from the simulated weather (labelled SIMULATED in the UI)."""
    d = int(round(s["windDir"] / 10.0)) * 10 % 360 or 360
    spd, gst = int(round(s["windSpeed"])), int(round(s["windGust"]))
    wind = f"{d:03d}{spd:02d}" + (f"G{gst:02d}" if gst - spd >= 10 else "") + "KT"
    wx = " -RA" if s["rainRate"] > 0 else ""
    vis = "6000" if s["rainRate"] > 0 else "9999"
    t = int(round(s["outTemp"]))
    dp = int(round(t - (100 - s["outHumidity"]) / 5.0))
    return f"{MODES[scn['mode']]['station']} 311030Z {wind} {vis}{wx} SCT025 {t:02d}/{dp:02d} Q{int(round(s['barometer']))}"


def clone(scn):
    return copy.deepcopy(scn)
