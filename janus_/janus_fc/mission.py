"""Turn a planned route into MAVLink mission items (relative-altitude frame).

Mission shape:
  [TAKEOFF]                    first upload only
  WAYPOINT ...                 the smoothed route (start point omitted)
  [LOITER_TO_ALT]              fixed-wing, before a leg steeper than max_gradient
  arrival: LAND | RTL | LOITER_UNLIM      or, when the goal is blocked:
  hold:    pattern WAYPOINTs + DO_JUMP back to the first pattern item

Each item carries a "tag" so the agent knows which route point the autopilot
is flying to (MISSION_CURRENT seq -> tag).
"""
import math

from pymavlink import mavutil

M = mavutil.mavlink
REL = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT


def _wp(frame, n, e, alt, accept, tag, px4):
    lat, lon = frame.to_ll(n, e)
    return dict(cmd=M.MAV_CMD_NAV_WAYPOINT, frame=REL, p1=0, p2=accept, p3=0, p4=float("nan") if px4 else 0,
                lat=lat, lon=lon, alt=alt, tag=tag)


def build(frame, profile, stack, route, takeoff=False, arrival=None, hold=None, grid=None, home_ll=None, landing=None):
    """route: [(n, e, alt), ...] with route[0] = where the vehicle is now.
    hold: dict from planner.choose_hold (the route then ends at its fix).
    Returns (items, notes)."""
    px4 = stack == "px4"
    plane = profile["class"] == "plane"
    acc = profile["accept_radius_m"]
    items, notes = [], []
    if takeoff:
        alt = max(profile["takeoff_alt_m"], 10.0)
        if plane and len(route) > 1:
            # climb-out point along the first leg (PX4 FW takes off toward the takeoff waypoint)
            (n0, e0, _), (n1, e1, _) = route[0], route[1]
            L = math.hypot(n1 - n0, e1 - e0) or 1.0
            d = min(L, 300.0)
            tn, te = n0 + (n1 - n0) * d / L, e0 + (e1 - e0) * d / L
        else:
            tn, te = route[0][0], route[0][1]
        lat, lon = frame.to_ll(tn, te)
        items.append(dict(cmd=M.MAV_CMD_NAV_TAKEOFF, frame=REL, p1=(15.0 if plane else 0.0), p4=float("nan") if px4 else 0,
                          lat=lat, lon=lon, alt=alt, tag="takeoff"))
    # fly at the profile's cruise speed: the bingo maths and the drift horizon assume it
    items.append(dict(cmd=M.MAV_CMD_DO_CHANGE_SPEED, frame=M.MAV_FRAME_MISSION, p1=(0 if plane else 1),
                      p2=profile["cruise_ms"], p3=-1, tag="speed"))
    prev = route[0]
    for k, pt in enumerate(route[1:], start=1):
        if plane and profile.get("max_gradient") and grid is not None:
            h = math.hypot(pt[0] - prev[0], pt[1] - prev[1])
            if abs(pt[2] - prev[2]) > profile["max_gradient"] * h + 1.0:
                r = profile["loiter_radius_m"]
                if _circle_clear(grid, prev, r, pt[2]):
                    lat, lon = frame.to_ll(prev[0], prev[1])
                    items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_TO_ALT, frame=REL, p1=0, p2=r, p3=0, p4=0,
                                      lat=lat, lon=lon, alt=pt[2], tag=f"climb{k}"))
                else:
                    notes.append(f"leg {k} is steeper than the climb gradient and there is no clear room to spiral; "
                                 "the autopilot will climb as fast as it can")
        items.append(_wp(frame, pt[0], pt[1], pt[2], acc, f"wp{k}", px4))
        prev = pt
    if hold:
        fix_alt = hold["fix"][2]
        first = len(items)
        if hold["kind"] == "orbit" and plane:
            lat, lon = frame.to_ll(*hold["fix"][:2])
            items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_UNLIM, frame=REL, p3=hold["radius"], p4=float("nan") if px4 else 0,
                              lat=lat, lon=lon, alt=fix_alt, tag="hold0"))
        else:
            for j, (n, e) in enumerate(hold["pts"]):
                items.append(_wp(frame, n, e, fix_alt, acc, f"hold{j}", px4))
            items.append(dict(cmd=M.MAV_CMD_DO_JUMP, frame=M.MAV_FRAME_MISSION, p1=first, p2=100, tag="hold_jump"))
        # first is an index into OUR list; the autopilot seq offset is applied by fix_jumps()
        return items, notes
    if landing:
        # fixed-wing landing: DO_LAND_START, orbit down, straight final, touchdown
        cn, ce = landing["centre"]
        lat, lon = frame.to_ll(cn, ce)
        if px4:
            # PX4 accepts DO_LAND_START only in the mission frame; it marks the landing for
            # mission-landing RTL. ArduPlane lands without it, and with it would need RTL_AUTOLAND set.
            items.append(dict(cmd=M.MAV_CMD_DO_LAND_START, frame=M.MAV_FRAME_MISSION, tag="land_start"))
        items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_TO_ALT, frame=REL, p1=1, p2=landing["radius"], p3=0, p4=0,
                          lat=lat, lon=lon, alt=landing["alt"], tag="approach"))
        tlat, tlon = frame.to_ll(*landing["td"])
        items.append(dict(cmd=M.MAV_CMD_NAV_LAND, frame=REL, p1=0, p2=0, p4=float("nan") if px4 else 0,
                          lat=tlat, lon=tlon, alt=0, tag="land"))
        return items, notes
    arrival = arrival or ("loiter" if profile["on_arrival"] == "loiter" else "land")
    last = route[-1]
    lat, lon = frame.to_ll(last[0], last[1])
    if arrival == "land":
        items.append(dict(cmd=M.MAV_CMD_NAV_LAND, frame=REL, p4=float("nan") if px4 else 0, lat=lat, lon=lon, alt=0, tag="land"))
    elif arrival == "rtl":
        items.append(dict(cmd=M.MAV_CMD_NAV_RETURN_TO_LAUNCH, frame=M.MAV_FRAME_MISSION, tag="rtl"))
    else:
        items.append(dict(cmd=M.MAV_CMD_NAV_LOITER_UNLIM, frame=REL, p3=profile["loiter_radius_m"] if plane else 0,
                          p4=float("nan") if px4 else 0, lat=lat, lon=lon, alt=last[2], tag="loiter"))
    return items, notes


def fix_jumps(items, offset):
    """DO_JUMP targets are autopilot seq numbers: shift by the home slot (ArduPilot)."""
    out = []
    for it in items:
        it = dict(it)
        if it["cmd"] == M.MAV_CMD_DO_JUMP:
            it["p1"] = it["p1"] + offset
        out.append(it)
    return out


def _circle_clear(grid, centre, r, alt):
    pts = [(centre[0] + r * math.cos(a), centre[1] + r * math.sin(a)) for a in [k * math.pi / 6 for k in range(12)]]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        for z in (centre[2], alt):
            if grid.segment_conflict((a[0], a[1], z), (b[0], b[1], z)):
                return False
    return True


def fence_items(frame, zones, min_alt, max_alt):
    """Exclusion fences for hard zones that cover the whole flight altitude band
    (fences are 2-D in both stacks). A backstop if the companion computer dies."""
    items = []
    for z in zones:
        if z.type not in ("tfr", "geofence", "obstacle") or z.vn or z.ve:
            continue
        if not (z.alt_min <= min_alt and z.alt_max >= max_alt):
            continue
        if z.poly:
            for (n, e) in z.poly:
                lat, lon = frame.to_ll(n, e)
                items.append(dict(cmd=M.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION, frame=M.MAV_FRAME_GLOBAL,
                                  p1=len(z.poly), lat=lat, lon=lon, alt=0))
        elif len(z.circles) == 1:
            n, e, r, _ = z.circles[0]
            lat, lon = frame.to_ll(n, e)
            items.append(dict(cmd=M.MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION, frame=M.MAV_FRAME_GLOBAL, p1=r, lat=lat, lon=lon, alt=0))
    return items
