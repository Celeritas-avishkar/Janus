"""Seeded scenario generation, ground truth and the WeeWX-style weather feed.

A scenario is plain JSON so it can be sent to the drone and relay processes
over the simulation-control channel. Truth = static map + weather hazards
whose appearance time t_on has passed.
"""
import copy
import math
import random

from .config import COLS, ROWS, LAYERS, MODES, ASH_BANDS, ASH_SOURCE, FRINGE_CELLS
from . import planner as P
from . import radio

WEATHER_PROFILES = ["calm", "breezy", "stormy"]


# ---------------------------------------------------------------- helpers
PLAN_HORIZON_S = 4.0      # planners avoid where a drifting zone will be over the next few seconds


def box(hid, htype, x0, y0, w, h, z0, z1, **kw):
    d = {"id": hid, "type": htype, "x0": int(x0), "x1": int(x0 + w - 1), "y0": int(y0),
         "y1": int(y0 + h - 1), "z0": int(z0), "z1": int(z1), "t_on": 0.0}
    d.update(kw)
    return d


def circle(hid, htype, cx, cy, r, z0, z1, **kw):
    """Circular zone. x0..y1 hold the initial bounding box so older code that
    only needs a rough extent still works; containment uses cx, cy, r."""
    d = {"id": hid, "type": htype, "shape": "circle", "cx": float(cx), "cy": float(cy), "r": float(r),
         "x0": int(math.floor(cx - r)), "x1": int(math.ceil(cx + r)), "y0": int(math.floor(cy - r)),
         "y1": int(math.ceil(cy + r)), "z0": int(z0), "z1": int(z1), "t_on": 0.0}
    d.update(kw)
    return d


def drift_velocity(scn_mode, wind, factor):
    """Cells per demo-second for a zone carried by the wind (scaled by factor)."""
    if not factor:
        return 0.0, 0.0
    m = MODES[scn_mode]
    speed = factor * wind["kt"] * 0.514444 * m["time_scale"] / m["cell_m"]
    toward = math.radians((wind["dir"] + 180.0) % 360.0)
    return round(speed * math.sin(toward), 4), round(-speed * math.cos(toward), 4)


def covers(h, x, y, t=0.0):
    return P.zone_contains_xy(h, x, y, t)


def make_plume(pid, mode, wind, drift, rng, level, o, home, targets, static, others):
    """A volcanic vent and its downwind plume. Puffs leave the vent every few
    seconds, drift with the wind, spread and thin out (spreading + fall-out),
    so the plume is dense and narrow at the vent and wide and faint downwind.
    The vent is placed so the plume does not cover the launch point or a target."""
    m = MODES[mode]
    nominal = max(drift, 0.5)                                   # shape of a still plume = a ½-wind plume
    nvx, nvy = drift_velocity(mode, wind, nominal)
    speed = math.hypot(nvx, nvy) or 0.3
    ux, uy = nvx / speed, nvy / speed
    vx, vy = (nvx * drift / nominal, nvy * drift / nominal) if drift else (0.0, 0.0)
    conc = o.get("ash_conc", "auto")
    c0 = {"1": 3.0, "2": 6.0, "3": 11.0}.get(str(conc), [4.5, 7.0, 11.0][level] * rng.uniform(0.85, 1.15))
    length = rng.uniform(9.0, 13.0)
    real_speed = speed * drift / nominal if drift else speed
    life = length / real_speed                                   # seconds (demo) a puff lives
    r0, r_end = 0.8, rng.uniform(1.8, 2.4)
    dr = (r_end - r0) / life
    tau = life / max(0.3, math.log(c0 * (r0 / r_end) ** 2 / 0.2))
    every = max(0.8, min(3.0, 0.7 / max(real_speed, 0.05)))
    top = o.get("ash_top", "auto")
    if mode == "small":
        z0, z1 = 0, int(top) if str(top).isdigit() else rng.randint(5, LAYERS - 2)
    else:
        z1 = int(top) if str(top).isdigit() else rng.randint(5, 7)
        z0 = max(0, z1 - 3)                                     # umbrella cloud spreading at altitude
    protect = [tuple(home[:2])] + [tuple(t[:2]) for t in targets]
    for _a in range(120):
        vent = [rng.uniform(4, COLS - 5), rng.uniform(2, ROWS - 3)]
        if any(o2["x0"] - 1 <= vent[0] <= o2["x1"] + 1 and o2["y0"] - 1 <= vent[1] <= o2["y1"] + 1 for o2 in static):
            continue
        pl = {"id": pid, "type": "ash", "shape": "plume", "name": f"Vent {pid}", "vent": [round(vent[0], 2), round(vent[1], 2)],
              "ux": round(ux, 4), "uy": round(uy, 4), "speed": round(real_speed if drift else speed, 4),
              "vx": round(vx, 4), "vy": round(vy, 4), "every": round(every, 2), "life": round(life, 2),
              "t_start": round(-life, 2), "r0": r0, "dr": round(dr, 4), "tau": round(tau, 2), "c0": round(c0, 1),
              "phase": round(rng.uniform(0, 6.28), 2), "meander": 0.3, "z0": z0, "z1": z1, "t_on": 0.0,
              "sev": 3 if c0 >= 4 else 2 if c0 >= 2 else 1, "conc_mg": round(c0, 1), "source": "ADVISORY"}
        if not drift:
            pl["freeze"] = 0.0
            pl["speed"] = round(speed, 4)
        # how far does the plume reach (at the frozen / representative time 0)?
        if any(P.zone_distance(pl, px_, py_, 0.0) < 1.6 for px_, py_ in protect):
            continue
        if any(o2.get("shape") == "plume" and math.dist(o2["vent"], pl["vent"]) < 6 for o2 in others):
            continue
        b = P.zone_bounds(pl, 0.0)
        pl.update({"x0": int(math.floor(b[0])), "y0": int(math.floor(b[1])), "x1": int(math.ceil(b[2])), "y1": int(math.ceil(b[3]))})
        return pl
    return None


def freeze_weather(scn, t=0.0):
    """Stop all weather where it is at time t (stories about something else)."""
    for h in scn["hazards"]:
        if h.get("shape") == "plume":
            h["freeze"] = t
        else:
            h.pop("vx", None), h.pop("vy", None)


def ash_concentration(scn, pos, t):
    """True ash concentration (mg/m3) at a continuous position, and the plume it
    comes from. Inside a plume: its concentration. Outside: it falls off with
    distance from the edge (diffuse plume edge, FRINGE_CELLS e-folding)."""
    best, hid = 0.0, None
    for h in [p for z in truth_hazards(scn, t) if z["type"] == "ash" for p in P.plume_puffs(z, t)]:
        z0, z1 = P.hazard_zrange(h, scn["layers"])
        dz = max(0.0, z0 - pos[2], pos[2] - z1)
        d = P.zone_distance(h, pos[0], pos[1], t) + dz
        c0 = P.zone_conc(h, t)
        c = c0 * math.exp(-d / FRINGE_CELLS) if d > 0 else c0
        if c > best:
            best, hid = c, h["id"]
    return best, hid


def truth_hazards(scn, t):
    return [h for h in scn["hazards"] if h.get("t_on", 0.0) <= t < h.get("t_off", 1e9)]


def truth_all(scn, t):
    return scn["static"] + truth_hazards(scn, t)


def wind_tuple(scn):
    return P.wind_vector(scn["wind"]["dir"])


def grid_for(scn, hazards, profile, t=0.0, horizon=PLAN_HORIZON_S):
    return P.Grid(scn["cols"], scn["rows"], scn["layers"], hazards, profile, wind=wind_tuple(scn), t=t, horizon=horizon)


def alt_label(scn, z):
    ft = MODES[scn["mode"]]["layer_ft"] * z
    return f"{ft:,.0f} ft"


# ---------------------------------------------------------------- generation
def base_scenario(seed, mode, weather="breezy", script="nominal", opts=None):
    """opts (all optional): wind_dir, wind_kt, gust_kt, ash_plumes, ash_conc (1-3),
    ash_top (layer), cells (gust/turbulence cells), rain (rain cells, small UAS),
    temp_c, vis_km, drift (0 = still, 0.5 = half wind speed (default), 1 = wind speed).
    None or missing = seeded random."""
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
    o = dict(opts or {})
    base_kt = [rng.uniform(4, 8), rng.uniform(10, 16), rng.uniform(16, 24)][level]
    gust_add = [rng.uniform(2, 5), rng.uniform(6, 10), rng.uniform(10, 16)][level]
    wdir = float(rng.choice([180, 200, 225, 250, 270, 290, 315, 0, 45, 90, 135]))
    if o.get("wind_dir") is not None:
        wdir = float(o["wind_dir"]) % 360
    if o.get("wind_kt") is not None:
        base_kt = float(o["wind_kt"])
    gust = base_kt + gust_add
    if o.get("gust_kt") is not None:
        gust = max(base_kt, float(o["gust_kt"]))
    wind = {"dir": wdir, "kt": round(base_kt, 1), "gust": round(gust, 1)}

    hid = 1
    # ---- volcanic ash: a vent and its plume (not random blobs) ---------------
    drift = float(o["drift"]) if o.get("drift") is not None else 0.5
    n_vents = int(o["ash_plumes"]) if o.get("ash_plumes") is not None else 1
    for _ in range(max(0, n_vents)):
        pl = make_plume(f"V{hid}", mode, wind, drift, rng, level, o, home, targets, static, hazards)
        if pl is not None:
            hazards.append(pl)
            hid += 1

    # ---- secondary weather ---------------------------------------------------
    n_cells = int(o["cells"]) if o.get("cells") is not None else [1, 2, 3][level]
    for _ in range(max(0, n_cells)):
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
    n_rain = int(o["rain"]) if o.get("rain") is not None else [0, 1, 2][level]
    if mode == "small":
        for _ in range(max(0, n_rain)):
            for _a in range(60):
                w, h = rng.choice([(2, 2), (3, 2), (2, 3)])
                x0, y0 = rng.randint(3, COLS - w - 3), rng.randint(0, ROWS - h)
                if not clear_of(x0, y0, w, h):
                    continue
                hazards.append(box(f"W{hid}", "rain", x0, y0, w, h, 0, LAYERS - 1,
                                   rain_mmh=round(rng.uniform(4, 25), 1), source="WEATHER"))
                hid += 1
                break

    # ---- shapes and motion: some zones are circles; weather drifts with the wind
    vx, vy = drift_velocity(mode, wind, drift)
    for i, h in enumerate(hazards):
        if h.get("shape") == "plume":
            continue
        if h["type"] == "turb" and rng.random() < 0.5:
            w, hh = h["x1"] - h["x0"] + 1, h["y1"] - h["y0"] + 1
            c = circle(h["id"], h["type"], h["x0"] + (w - 1) / 2.0, h["y0"] + (hh - 1) / 2.0,
                       max(w, hh) / 2.0 + 0.1, h["z0"], h["z1"])
            c.update({k: v for k, v in h.items() if k not in c})
            hazards[i] = h = c
        if vx or vy:
            h["vx"], h["vy"], h["t_ref"] = vx, vy, 0.0

    env = {"temp_c": o.get("temp_c"), "vis_km": o.get("vis_km"), "level": level, "drift": drift}

    return {
        "seed": seed, "mode": mode, "script": script, "weather_profile": weather,
        "cols": COLS, "rows": ROWS, "layers": LAYERS,
        "home": home, "targets": targets,
        "gcs": [home[0], home[1], radio.GCS_ANTENNA_LAYER],
        "static": static, "hazards": hazards, "wind": wind,
        "feed_latency": 1.0, "soc": 1.0, "story": {}, "impair": {},
        "next_hazard_id": hid, "env": env, "wx_opts": o,
    }


def initial_plan(scn, profile, known=None, t=0.0):
    known = scn["static"] + truth_hazards(scn, t) if known is None else known
    grid = grid_for(scn, known, profile, t=t)
    res, legs = P.plan_through(grid, scn["home"], scn["targets"], profile["budget"])
    return res, legs, grid


# ---------------------------------------------------------------- weather feed (GCS side)
class WeatherFeed:
    """Seeded WeeWX-style feed. Field names follow the WeeWX archive schema
    (outTemp, outHumidity, barometer, windSpeed, windGust, windDir, rainRate);
    units here are metric/knots for display. Hazard volumes become visible to
    the feed feed_latency seconds after they appear in the truth. Hazards
    marked `unreported` never reach the feed: only the drone's own sensor can
    find them (it then reports them back over the radio)."""

    def __init__(self, scn, seed_extra=0):
        self.scn = scn
        self.rng = random.Random(f"wx:{scn['seed']}:{seed_extra}")
        self.paused = False
        self.last = None
        self.last_good_t = 0.0
        env = scn.get("env") or {}
        t_def = self.rng.uniform(24, 29) if scn["mode"] == "small" else self.rng.uniform(22, 30)
        self.temp = float(env["temp_c"]) if env.get("temp_c") is not None else t_def
        self.vis = float(env["vis_km"]) if env.get("vis_km") is not None else 10.0
        self.level = int(env.get("level", 1))
        self.baro = self.rng.uniform(1008, 1014) - 2.5 * self.level
        self.hum = self.rng.uniform(55, 80) + 6 * self.level

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
        visible = self._visible(t)
        rain_cells = [h for h in visible if h["type"] == "rain"]
        ash = [h for h in visible if h["type"] == "ash"]
        rain = max([h.get("rain_mmh", 0) for h in rain_cells] + [0.0])
        temp = self.temp + math.sin(t / 30.0) * 0.4
        hum = min(100.0, self.hum)
        dew = temp - (100 - hum) / 5.0
        vis = self.vis
        if rain > 0:
            vis = min(vis, 6.0 if rain < 10 else 3.0)
        surface_ash = [h for h in ash if h["z0"] == 0]
        if surface_ash:
            vis = min(vis, [8.0, 5.0, 2.5][max(int(h.get("sev", 2)) for h in surface_ash) - 1])
        layer_ft = MODES[self.scn["mode"]]["layer_ft"]
        cloud_ft = [3500, 2500, 1500][self.level]
        adv = []
        for h in ash:
            band = ASH_BANDS[int(h.get("sev", 3))]
            full = h["z1"] >= self.scn["layers"] - 1 and self.scn["mode"] == "aircraft"
            base = "SFC" if (h["z0"] == 0 or full) else _fl(h["z0"] * layer_ft, self.scn["mode"])
            top = _fl(h["z1"] * layer_ft, self.scn["mode"]) + ("+" if full else "")
            adv.append({"id": h["id"], "band": band["name"], "range": band["range"], "conc_mg": h.get("conc_mg"),
                        "base": base, "top": top, "mov": (_mov(w["dir"], w["kt"] * float((self.scn.get("env") or {}).get("drift", 0.5)))
                                if (h.get("vx") or h.get("vy")) else "STNR"),
                        "cells": len(P.zone_cells(h, self.scn["cols"], self.scn["rows"], t)),
                        "shape": h.get("shape", "box"), "moving": bool(h.get("vx") or h.get("vy"))})
        s = {
            "t": round(t, 2), "status": "LIVE", "age_s": 0.0,
            "source": "SIM (WeeWX schema)" if self.scn["mode"] == "small" else "SIM (METAR / advisory schema)",
            "outTemp": round(temp, 1), "dewpoint": round(dew, 1),
            "outHumidity": round(hum, 0),
            "barometer": round(self.baro - t * 0.002, 1),
            "windSpeed": round(speed, 1), "windGust": round(gust, 1),
            "windDir": round((w["dir"] + jitter) % 360, 0),
            "rainRate": round(rain, 1), "visibility_km": round(vis, 1), "cloud_ft": cloud_ft,
            "hazards": [h["id"] for h in visible],
            "counts": {k: sum(1 for h in visible if h["type"] == k) for k in ("ash", "gust", "rain", "turb")},
            "ash": adv, "ash_source": ASH_SOURCE,
        }
        s["metar"] = metar_line(self.scn, s)
        s["vaa"] = vaa_lines(adv)
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
        show up feed_latency seconds after they form. Unreported ones never do."""
        lat = self.scn["feed_latency"]
        return [h for h in self.scn["hazards"]
                if not h.get("unreported")
                and (h.get("t_on", 0.0) <= 0.0 or h.get("t_on", 0.0) + lat <= t) and t < h.get("t_off", 1e9)]


def _fl(ft, mode):
    if mode == "aircraft":
        return f"FL{int(round(ft / 100)):03d}"
    return f"{int(ft)} ft AGL"


def _mov(wind_from, kt):
    toward = (wind_from + 180) % 360
    names = ["N", "NE", "E", "SE", "S", "SW", "W", "NW"]
    return f"MOV {names[int((toward + 22.5) // 45) % 8]} {int(round(kt))}KT"


def vaa_lines(adv):
    """One line per plume, laid out like a volcanic ash advisory (labelled SIMULATED in the UI)."""
    return [f"OBS VA CLD {a['id']}: {a['base']}/{a['top']} {a['mov']} CONC {a['band']} ({a['range']})" for a in adv]


def metar_line(scn, s):
    """A METAR-format line synthesised from the simulated weather (labelled SIMULATED in the UI)."""
    d = int(round(s["windDir"] / 10.0)) * 10 % 360 or 360
    spd, gst = int(round(s["windSpeed"])), int(round(s["windGust"]))
    wind = f"{d:03d}{spd:02d}" + (f"G{gst:02d}" if gst - spd >= 10 else "") + "KT"
    wx = ""
    if s["rainRate"] > 0:
        wx += " -RA" if s["rainRate"] < 10 else " RA"
    if any(a["base"] == "SFC" for a in s.get("ash", [])):
        wx += " VA"
    vis_m = int(min(9999, s.get("visibility_km", 10) * 1000))
    vis = "9999" if vis_m >= 9999 else f"{vis_m // 100 * 100:04d}"
    cloud = s.get("cloud_ft", 2500) // 100
    cov = "SCT" if cloud >= 25 else "BKN"
    t = int(round(s["outTemp"]))
    dp = int(round(s.get("dewpoint", t - (100 - s["outHumidity"]) / 5.0)))
    return f"{MODES[scn['mode']]['station']} 311030Z {wind} {vis}{wx} {cov}{cloud:03d} {t:02d}/{dp:02d} Q{int(round(s['barometer']))}"


def clone(scn):
    return copy.deepcopy(scn)
