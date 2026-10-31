"""Generate example zone files around the SITL default home positions.
Geometry is written in metres north/east of home, converted to lat/lon."""
import json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from janus_fc.geo import LocalFrame

HOMES = {"px4": (47.397742, 8.545594), "ardupilot": (-35.363262, 149.165237)}

def ll(f, n, e):
    lat, lon = f.to_ll(n, e)
    return round(lat, 7), round(lon, 7)

def copter(f, AT=110):
    return {"wind": {"from_deg": 250, "speed_ms": 4},
            "zones": [
        {"id": "TFR1", "type": "tfr", "shape": "circle", **dict(zip(("lat", "lon"), ll(f, 300, 250))), "radius_m": 120,
         "alt_min_m": 0, "alt_max_m": 400},
        {"id": "V1", "type": "ash", "shape": "plume", **dict(zip(("lat", "lon"), ll(f, 80, 520))), "conc_mg": 8,
         "r0_m": 35, "spread": 0.18, "decay_m": 600, "alt_min_m": 0, "alt_max_m": 300},
        {"id": "GF1", "type": "geofence", "shape": "polygon",
         "points": [list(ll(f, 520, -80)), list(ll(f, 620, -80)), list(ll(f, 620, 120)), list(ll(f, 520, 120))]},
        {"id": "HIDDEN1", "type": "ash", "shape": "circle", **dict(zip(("lat", "lon"), ll(f, 440, 530))), "radius_m": 50,
         "conc_mg": 6, "alt_min_m": 0, "alt_max_m": 200, "reported": False, "drift": True, "at_s": AT},
    ]}

def plane(f, AT=100):
    return {"wind": {"from_deg": 250, "speed_ms": 6},
            "zones": [
        {"id": "TFR1", "type": "tfr", "shape": "circle", **dict(zip(("lat", "lon"), ll(f, 900, 700))), "radius_m": 300,
         "alt_min_m": 0, "alt_max_m": 1000},
        {"id": "V1", "type": "ash", "shape": "plume", **dict(zip(("lat", "lon"), ll(f, 200, 1500))), "conc_mg": 8,
         "r0_m": 100, "spread": 0.15, "decay_m": 1500, "alt_min_m": 0, "alt_max_m": 600},
        {"id": "HIDDEN1", "type": "ash", "shape": "circle", **dict(zip(("lat", "lon"), ll(f, 1450, 1690))), "radius_m": 140,
         "conc_mg": 6, "alt_min_m": 0, "alt_max_m": 500, "reported": False, "drift": True, "at_s": AT},
    ]}

GOALS = {"copter": (650, 650, 40), "plane": (2200, 2200, 120)}

if __name__ == "__main__":
    out = os.path.join(os.path.dirname(__file__), "..", "examples")
    for stack, home in HOMES.items():
        f = LocalFrame(*home)
        for cls, fn in (("copter", copter), ("plane", plane)):
            path = os.path.join(out, f"zones_{stack}_{cls}.json")
            # when the unreported cloud is forecast to be on the route: tuned to each SITL's
            # measured arrival time (PX4's SIH plane takes off and climbs much more slowly)
            at = {("px4", "plane"): 260, ("ardupilot", "plane"): 100}.get((stack, cls), 110)
            json.dump(fn(f, AT=at), open(path, "w"), indent=2)
            g = GOALS[cls]
            lat, lon = f.to_ll(g[0], g[1])
            print(f"{path}: home {home[0]},{home[1]}  goal {lat:.6f},{lon:.6f},{g[2]}")
