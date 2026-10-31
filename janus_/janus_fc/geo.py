"""Local flat-earth frame around the home position.

Janus plans in metres north/east of home and altitude above home (the same
"relative altitude" frame the autopilots use for MAV_FRAME_GLOBAL_RELATIVE_ALT).
An equirectangular projection is accurate to well under a metre over the few
kilometres a companion-computer planner works in; it is not meant for
routes longer than ~50 km.
"""
import math

EARTH_R = 6378137.0  # WGS-84 equatorial radius, m


class LocalFrame:
    def __init__(self, lat0, lon0):
        self.lat0, self.lon0 = float(lat0), float(lon0)
        self._kx = math.cos(math.radians(self.lat0)) * EARTH_R * math.pi / 180.0
        self._ky = EARTH_R * math.pi / 180.0

    def to_ne(self, lat, lon):
        return ((lat - self.lat0) * self._ky, (lon - self.lon0) * self._kx)

    def to_ll(self, n, e):
        return (self.lat0 + n / self._ky, self.lon0 + e / self._kx)


def distance_m(lat1, lon1, lat2, lon2):
    """Great-circle distance (haversine)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_R * math.asin(min(1.0, math.sqrt(a)))


def wind_to_ne(from_deg, speed):
    """Meteorological wind (direction it blows FROM, degrees true) -> velocity
    the air moves with, as (north, east) m/s."""
    to = math.radians(from_deg + 180.0)
    return speed * math.cos(to), speed * math.sin(to)


def heading_ne(dn, de):
    return math.degrees(math.atan2(de, dn)) % 360.0
