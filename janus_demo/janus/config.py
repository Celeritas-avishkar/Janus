"""Modes, drone presets, network ports and shared constants.

Every number here is an illustrative demonstrator value. None of them are
aircraft specifications or flight-control limits.
"""
import copy

# ---------------------------------------------------------------- grid
COLS, ROWS, LAYERS = 25, 17, 9          # same footprint as main.py (1000x680 px at 40 px cells)

MODES = {
    "small": {
        "key": "small",
        "name": "Small UAS",
        "layer_ft": 50,                 # 9 layers -> 0..400 ft
        "cell_m": 100,                  # 25 x 17 cells -> 2.5 km x 1.7 km
        "freq_mhz": 2400.0,
        "noise_dbm": -100.0,
        "gcs_tx_dbm": 27.0,
        "gcs_gain_dbi": 5.0,
        "physical": ["building"],      # blocks radio line of sight
        "solid": ["building", "geofence"],
        "weather": ["gust", "rain"],
        "station": "VOBL",
    },
    "aircraft": {
        "key": "aircraft",
        "name": "Aircraft",
        "layer_ft": 5000,               # 9 layers -> 0..40,000 ft (main.py scale)
        "cell_m": 5000,                 # 25 x 17 cells -> 125 km x 85 km
        "freq_mhz": 900.0,
        "noise_dbm": -100.0,
        "gcs_tx_dbm": 43.0,
        "gcs_gain_dbi": 9.0,
        "physical": ["ridge"],
        "solid": ["ridge", "tfr"],
        "weather": ["ash", "turb"],
        "station": "VOBL",
    },
}

HAZARD_LABEL = {
    "building": "Building", "geofence": "Geofence", "ridge": "Terrain ridge", "tfr": "TFR",
    "gust": "Gust cell", "rain": "Rain cell", "ash": "Volcanic ash", "turb": "Turbulence",
}
TURB_NAME = {1: "LOW", 2: "MODERATE", 3: "HIGH"}

FAILSAFES = ["CONTINUE", "RETURN_TO_LINK", "RTL", "HOLD"]
FAILSAFE_LABEL = {
    "CONTINUE": "Continue mission on last known map",
    "RETURN_TO_LINK": "Fly back to last good link, then resume",
    "RTL": "Return to launch and land",
    "HOLD": "Hold position, then return to launch",
}

# ---------------------------------------------------------------- drone profiles
# speed: cells/s (main.py used 1.2 .. 6.5); climb_mult scales main.py VERTICAL_COST = 2
# energy: illustrative units; burn = move cost x 0.5 x burn  (main.py ENERGY_PER_COST_UNIT = 0.5)
BASE_PROFILE = {
    "name": "Custom", "mode": "small", "ceiling": 8, "speed": 2.0, "climb_mult": 1.0,
    "lookahead": 6, "gust_limit_kt": 20.0, "turb_block": 2, "rain_rated": False,
    "energy": 60.0, "reserve_pct": 20.0, "burn": 1.0, "loiter_burn": 0.35,
    "wind_penalty": 3.0, "tx_dbm": 20.0, "gain_dbi": 2.0, "lost_link_s": 4.0,
    "failsafe": "RETURN_TO_LINK", "budget": 20000,
}

PRESETS = {
    "small": {
        "Scout-Q  light quadcopter": dict(ceiling=8, speed=2.0, climb_mult=1.0, lookahead=4,
            gust_limit_kt=18, rain_rated=False, energy=48, reserve_pct=20, burn=1.0, loiter_burn=0.45,
            tx_dbm=20, gain_dbi=2, lost_link_s=4.0, failsafe="RETURN_TO_LINK", budget=20000),
        "Hauler-H  heavy-lift hexacopter": dict(ceiling=6, speed=1.4, climb_mult=1.6, lookahead=5,
            gust_limit_kt=26, rain_rated=True, energy=70, reserve_pct=25, burn=1.3, loiter_burn=0.6,
            tx_dbm=24, gain_dbi=2, lost_link_s=5.0, failsafe="RTL", budget=20000),
        "Swift-V  VTOL fixed-wing": dict(ceiling=8, speed=2.8, climb_mult=2.0, lookahead=6,
            gust_limit_kt=32, rain_rated=True, energy=90, reserve_pct=15, burn=0.8, loiter_burn=0.3,
            tx_dbm=27, gain_dbi=3, lost_link_s=4.0, failsafe="CONTINUE", budget=20000),
    },
    "aircraft": {
        "Sentinel  MALE": dict(ceiling=6, speed=2.6, climb_mult=2.5, lookahead=6, turb_block=3,
            energy=110, reserve_pct=15, burn=1.0, loiter_burn=0.35, tx_dbm=37, gain_dbi=3,
            lost_link_s=5.0, failsafe="CONTINUE", budget=30000),
        "Stratos  HALE": dict(ceiling=8, speed=3.4, climb_mult=2.5, lookahead=8, turb_block=3,
            energy=150, reserve_pct=15, burn=0.9, loiter_burn=0.3, tx_dbm=40, gain_dbi=3,
            lost_link_s=6.0, failsafe="HOLD", budget=30000),
        "Ranger  tactical": dict(ceiling=4, speed=2.0, climb_mult=1.6, lookahead=5, turb_block=2,
            energy=70, reserve_pct=20, burn=1.0, loiter_burn=0.4, tx_dbm=33, gain_dbi=2,
            lost_link_s=4.0, failsafe="RETURN_TO_LINK", budget=30000),
    },
}

# Limits used to validate custom profiles (UI sliders use the same ranges).
LIMITS = {
    "ceiling": (1, 8), "speed": (0.8, 6.5), "climb_mult": (0.5, 4.0), "lookahead": (1, 12),
    "gust_limit_kt": (5, 50), "turb_block": (1, 3), "energy": (15, 400), "reserve_pct": (0, 50),
    "burn": (0.3, 3.0), "loiter_burn": (0.0, 2.0), "wind_penalty": (0.0, 10.0),
    "tx_dbm": (0, 47), "gain_dbi": (0, 12), "lost_link_s": (1.0, 30.0), "budget": (50, 200000),
}


def make_profile(mode, preset=None, **overrides):
    """Build a validated profile dict. Unknown keys are rejected."""
    p = copy.deepcopy(BASE_PROFILE)
    p["mode"] = mode
    if preset:
        if preset not in PRESETS[mode]:
            raise ValueError(f"unknown preset {preset!r} for mode {mode!r}")
        p.update(PRESETS[mode][preset])
        p["name"] = preset
    for k, v in overrides.items():
        if k not in p:
            raise ValueError(f"unknown profile field {k!r}")
        p[k] = v
    return validate_profile(p)


def validate_profile(p):
    """Clamp numeric fields to LIMITS and coerce types. Returns a new dict."""
    q = dict(p)
    for k, (lo, hi) in LIMITS.items():
        q[k] = min(hi, max(lo, float(q[k])))
    for k in ("ceiling", "lookahead", "turb_block", "budget"):
        q[k] = int(round(q[k]))
    q["rain_rated"] = bool(q["rain_rated"])
    if q["failsafe"] not in FAILSAFES:
        q["failsafe"] = "RTL"
    if q["mode"] not in MODES:
        raise ValueError("mode must be 'small' or 'aircraft'")
    return q


# ---------------------------------------------------------------- network
# Radio path:   GCS :radio  ->  relay :air_gcs  ->  relay :air_drone  ->  drone :radio
# Sim channels (not radio, clearly labelled in the UI): observer, relay control, drone control.
DEFAULT_PORTS = {
    "gcs_radio": 47800, "air_gcs": 47801, "air_drone": 47802, "drone_radio": 47803,
    "gcs_obs": 47810, "air_ctl": 47811, "drone_ctl": 47812,
}
HOST = "127.0.0.1"

# ---------------------------------------------------------------- timing
TELEMETRY_HZ = 5.0
GCS_HEARTBEAT_HZ = 2.0
OBSERVER_HZ = 10.0
LINK_DEGRADED_S = 1.2
LINK_LOST_S = 2.5
REPLAN_PAUSE_S = 0.6        # visible "computing" hold, as in main.py (1.0 s there)
BLACKBOX_PERIOD_S = 0.25
RETRY_S = 0.35
MAX_ATTEMPTS = 40
