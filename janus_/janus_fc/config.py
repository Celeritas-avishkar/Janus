"""Vehicle profiles. Every value can be overridden from a JSON file (--profile)
or on the command line. The defaults are starting points for a ~2 kg quad and
a ~3 kg foam fixed-wing; tune them to your airframe before flying."""
import copy
import json
import math

G = 9.80665

PROFILES = {
    "copter": {
        "class": "copter",
        "cruise_ms": 8.0,            # set as WPNAV_SPEED / MPC_XY_CRUISE on the autopilot too
        "cell_m": 20.0,              # planning grid resolution
        "alt_layers_m": [30, 45, 60, 75, 90, 110],   # cruise layers, metres above home
        "takeoff_alt_m": 30.0,
        "safety_margin_m": 15.0,     # horizontal clearance kept from every blocking zone
        "vert_margin_m": 10.0,       # vertical clearance above/below a zone's band
        "gps_error_m": 5.0,          # added to the margin
        "accept_radius_m": 5.0,      # waypoint acceptance radius
        "max_gradient": None,        # copters can climb vertically
        "climb_cost": 1.5,           # cost per metre climbed relative to a metre flown
        "bank_deg": None,
        "loiter_radius_m": 40.0,     # hold pattern size
        "pct_per_min": 4.0,          # battery use prior (~20 min endurance) until it is learned in flight
        "reserve_pct": 25.0,         # never plan to arrive home with less
        "on_arrival": "land",        # land (at the goal) | loiter (at the goal) | home (planned route back, then land)
        "hold_pattern": "racetrack", # racetrack | racetrack_left | figure8 | orbit
        "turb_block_sev": 2,
        "sar": False,
        "opc_period_s": 1.0,         # Alphasense OPC-N3 histogram period is 1-30 s
        "opc_alarm_mg": 0.2,         # EASA LOW threshold
        "opc_core_mg": 2.0,          # above this we are inside, not at the edge
        "edge_m": 30.0,              # diffuse plume edge e-folding length (modelling assumption)
        "est_radius_m": 60.0,        # first estimate of an unreported plume
        "replan_horizon_s": 20.0,    # swept volume for drifting zones
    },
    "plane": {
        "class": "plane",
        "cruise_ms": 18.0,           # AIRSPEED_CRUISE / FW_AIRSPD_TRIM
        "cell_m": 60.0,
        "alt_layers_m": [80, 120, 160, 200, 250, 300],
        "takeoff_alt_m": 60.0,
        "safety_margin_m": 40.0,
        "vert_margin_m": 25.0,
        "gps_error_m": 5.0,
        "accept_radius_m": 30.0,
        "max_gradient": 0.15,        # ~8.5 deg flight-path angle; steeper legs get a loiter-to-alt
        "climb_cost": 3.0,
        "bank_deg": 30.0,            # used for the turn radius
        "loiter_radius_m": None,     # None -> 1.5 x turn radius
        "pct_per_min": 2.0,          # ~40 min endurance prior
        "reserve_pct": 30.0,
        "on_arrival": "home",        # planned route back, then a planned landing at home (into the wind)
        "approach_alt_m": 40.0,      # orbit down to this height, then a straight final approach
        "approach_len_m": 600.0,     # final approach length: 40 m over 600 m = 3.8 deg (PX4 FW_LND_ANG default 5)
        "hold_pattern": "racetrack",
        "turb_block_sev": 2,
        "sar": False,
        "opc_period_s": 1.0,
        "opc_alarm_mg": 0.2,
        "opc_core_mg": 2.0,
        "edge_m": 60.0,
        "est_radius_m": 200.0,
        "replan_horizon_s": 30.0,
    },
}

HOLD_KINDS = ("racetrack", "racetrack_left", "figure8", "orbit")


def turn_radius(p):
    if not p.get("bank_deg"):
        return 0.0
    return p["cruise_ms"] ** 2 / (G * math.tan(math.radians(p["bank_deg"])))


def make_profile(cls, overrides=None, sar=False):
    if cls not in PROFILES:
        raise ValueError(f"vehicle class must be one of {list(PROFILES)}")
    p = copy.deepcopy(PROFILES[cls])
    if overrides:
        unknown = set(overrides) - set(p)
        if unknown:
            raise ValueError(f"unknown profile keys: {sorted(unknown)}")
        p.update(overrides)
    p["sar"] = bool(sar or p.get("sar"))
    if p["hold_pattern"] not in HOLD_KINDS:
        raise ValueError(f"hold_pattern must be one of {HOLD_KINDS}")
    p["turn_radius_m"] = turn_radius(p)
    if not p.get("loiter_radius_m"):
        p["loiter_radius_m"] = max(1.5 * p["turn_radius_m"], 60.0)
    if p["on_arrival"] == "rtl":
        p["on_arrival"] = "home"     # never a raw straight-line RTL: Janus plans the way back
    if p["on_arrival"] not in ("land", "loiter", "home"):
        raise ValueError("on_arrival must be land, loiter or home")
    if cls == "plane" and p["on_arrival"] == "land":
        raise ValueError("on_arrival=land is not supported for fixed-wing (needs an approach); use home or loiter")
    # total horizontal clearance used by the planner
    p["margin_m"] = p["safety_margin_m"] + p["gps_error_m"] + p["turn_radius_m"]
    return p


def load_profile(cls, path=None, sar=False):
    overrides = None
    if path:
        with open(path) as f:
            overrides = json.load(f)
    return make_profile(cls, overrides, sar)
