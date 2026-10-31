"""MAVLink link to the flight controller: one class for ArduPilot and PX4.

Everything the two stacks do differently is in this file and nowhere else:

                         ArduPilot                         PX4
  mission seq 0          home (ignored as a nav item)      first real item
  mode encoding          custom_mode = mode number         main_mode<<16 | sub_mode<<24
  "hold here"            GUIDED (copter) / LOITER (plane)  AUTO.LOITER ("Hold")
  DO_REPOSITION z        honours the frame (relative alt)  always AMSL (frame ignored)
  DO_REPOSITION param4   plane: loiter direction (0=CW)    yaw (NaN = keep)
  takeoff                copter: GUIDED + NAV_TAKEOFF      mission takeoff item
                         plane: mission takeoff item

These were checked against the ArduPilot 4.6.3 and PX4 v1.17.0 sources.
"""
import collections
import math
import struct
import time

from pymavlink import mavutil

M = mavutil.mavlink

COPTER_TYPES = {M.MAV_TYPE_QUADROTOR, M.MAV_TYPE_HEXAROTOR, M.MAV_TYPE_OCTOROTOR, M.MAV_TYPE_TRICOPTER,
                M.MAV_TYPE_HELICOPTER, M.MAV_TYPE_COAXIAL, M.MAV_TYPE_DODECAROTOR}
PLANE_TYPES = {M.MAV_TYPE_FIXED_WING}
VTOL_TYPES = {getattr(M, n) for n in dir(M) if n.startswith("MAV_TYPE_VTOL")}

# generic name -> stack/vehicle specific mode
APM_MODES = {
    "copter": {"mission": 3, "hold": 4, "guided": 4, "rtl": 6, "land": 9, "smart_rtl": 21, "brake": 17,
               "loiter": 5, "stabilize": 0, "althold": 2, "poshold": 16},
    "plane": {"mission": 10, "hold": 12, "guided": 15, "rtl": 11, "loiter": 12, "takeoff": 13, "manual": 0,
              "fbwa": 5, "circle": 1},
}
PX4_MODES = {  # (main, sub)
    "mission": (4, 4), "hold": (4, 3), "guided": (4, 3), "rtl": (4, 5), "land": (4, 6), "takeoff": (4, 2),
    "manual": (1, 0), "altctl": (2, 0), "posctl": (3, 0), "offboard": (6, 0), "stabilized": (7, 0), "acro": (5, 0),
}


class MavError(RuntimeError):
    pass


class Autopilot:
    def __init__(self, conn, source_system=1, source_component=M.MAV_COMP_ID_ONBOARD_COMPUTER, baud=57600, log=print):
        self.conn_str = conn
        self.log = log
        self.m = mavutil.mavlink_connection(conn, source_system=source_system, source_component=source_component,
                                            baud=baud, autoreconnect=True)
        self.stack = None          # "ardupilot" | "px4"
        self.vclass = None         # "copter" | "plane"
        self.tsys, self.tcomp = None, None
        self.s = {"lat": None, "lon": None, "rel_alt": None, "amsl": None, "vn": 0.0, "ve": 0.0, "vz": 0.0,
                  "hdg": None, "armed": False, "custom_mode": None, "landed": None, "batt_pct": None,
                  "volt": None, "home": None, "seq": None, "reached": None, "wind": None, "gps_fix": 0,
                  "sats": 0, "hb_t": 0.0, "pos_t": 0.0}
        self.status_text = collections.deque(maxlen=50)
        self.acks = {}
        self.mission_q = collections.deque()
        self.on_text = None            # callback(severity, text) for flight-controller messages

    # ------------------------------------------------------------------ connection
    def connect(self, timeout=30):
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if not msg:
                continue
            if msg.type in (M.MAV_TYPE_GCS, M.MAV_TYPE_ONBOARD_CONTROLLER) or msg.autopilot == M.MAV_AUTOPILOT_INVALID:
                continue
            self.tsys, self.tcomp = msg.get_srcSystem(), msg.get_srcComponent()
            if msg.autopilot == M.MAV_AUTOPILOT_PX4:
                self.stack = "px4"
            elif msg.autopilot == M.MAV_AUTOPILOT_ARDUPILOTMEGA:
                self.stack = "ardupilot"
            else:
                raise MavError(f"unsupported autopilot type {msg.autopilot} (need ArduPilot or PX4)")
            if msg.type in COPTER_TYPES:
                self.vclass = "copter"
            elif msg.type in PLANE_TYPES:
                self.vclass = "plane"
            elif msg.type in VTOL_TYPES:
                self.vclass = "plane"
                self.log("[janus] VTOL detected: flying it as a fixed-wing (transitions are left to the autopilot)")
            else:
                raise MavError(f"unsupported vehicle type {msg.type}")
            self._on(msg)
            self._request_streams()
            return self
        raise MavError(f"no autopilot heartbeat on {self.conn_str} within {timeout} s")

    def _request_streams(self):
        rates = {M.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5, M.MAVLINK_MSG_ID_SYS_STATUS: 1, M.MAVLINK_MSG_ID_MISSION_CURRENT: 2,
                 M.MAVLINK_MSG_ID_EXTENDED_SYS_STATE: 1, M.MAVLINK_MSG_ID_HOME_POSITION: 0.5, M.MAVLINK_MSG_ID_BATTERY_STATUS: 1,
                 M.MAVLINK_MSG_ID_GPS_RAW_INT: 1, M.MAVLINK_MSG_ID_WIND: 1, M.MAVLINK_MSG_ID_WIND_COV: 1}
        for mid, hz in rates.items():
            self.command_long(M.MAV_CMD_SET_MESSAGE_INTERVAL, mid, int(1e6 / hz), wait=False)
        if self.stack == "ardupilot":   # older firmware / SR parameters at zero
            self.m.mav.request_data_stream_send(self.tsys, self.tcomp, M.MAV_DATA_STREAM_ALL, 4, 1)
        self.command_long(M.MAV_CMD_REQUEST_MESSAGE, M.MAVLINK_MSG_ID_HOME_POSITION, wait=False)

    # ------------------------------------------------------------------ receive
    def _on(self, msg):
        t = msg.get_type()
        s = self.s
        if t == "BAD_DATA":
            return
        if msg.get_srcSystem() != self.tsys and self.tsys is not None:
            return
        if t == "HEARTBEAT":
            if msg.get_srcComponent() != self.tcomp or msg.type == M.MAV_TYPE_GCS:
                return
            s["armed"] = bool(msg.base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED)
            s["custom_mode"] = msg.custom_mode
            s["hb_t"] = time.time()
        elif t == "GLOBAL_POSITION_INT":
            s.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, rel_alt=msg.relative_alt / 1000.0, amsl=msg.alt / 1000.0,
                     vn=msg.vx / 100.0, ve=msg.vy / 100.0, vz=msg.vz / 100.0,
                     hdg=None if msg.hdg == 65535 else msg.hdg / 100.0, pos_t=time.time())
        elif t == "SYS_STATUS":
            if msg.battery_remaining >= 0:
                s["batt_pct"] = float(msg.battery_remaining)
            s["volt"] = msg.voltage_battery / 1000.0
        elif t == "BATTERY_STATUS":
            if msg.battery_remaining >= 0:
                s["batt_pct"] = float(msg.battery_remaining)
        elif t == "EXTENDED_SYS_STATE":
            s["landed"] = msg.landed_state
        elif t == "HOME_POSITION":
            s["home"] = (msg.latitude / 1e7, msg.longitude / 1e7, msg.altitude / 1000.0)
        elif t == "MISSION_CURRENT":
            s["seq"] = msg.seq
        elif t == "MISSION_ITEM_REACHED":
            s["reached"] = msg.seq
        elif t == "GPS_RAW_INT":
            s["gps_fix"], s["sats"] = msg.fix_type, msg.satellites_visible
        elif t == "WIND":                                   # ArduPilot: direction wind is coming FROM
            s["wind"] = (msg.direction % 360.0, msg.speed)
        elif t == "WIND_COV":                               # PX4: wind velocity components (towards)
            if not (math.isnan(msg.wind_x) or math.isnan(msg.wind_y)):
                spd = math.hypot(msg.wind_x, msg.wind_y)
                s["wind"] = ((math.degrees(math.atan2(msg.wind_y, msg.wind_x)) + 180.0) % 360.0, spd)
        elif t == "STATUSTEXT":
            self.status_text.append((time.time(), msg.severity, msg.text))
            if self.on_text:
                self.on_text(msg.severity, msg.text)
            if msg.severity <= M.MAV_SEVERITY_WARNING:
                self.log(f"[fc] {msg.text}")
        elif t == "COMMAND_ACK":
            self.acks[msg.command] = (msg.result, time.time())
        elif t in ("MISSION_REQUEST", "MISSION_REQUEST_INT", "MISSION_ACK"):
            self.mission_q.append(msg)

    def pump(self, timeout=0.0):
        """Drain incoming messages. With timeout > 0, wait up to that long for the first one."""
        msg = self.m.recv_match(blocking=timeout > 0, timeout=timeout if timeout > 0 else None)
        while msg:
            self._on(msg)
            msg = self.m.recv_match(blocking=False)

    def link_ok(self, max_age=3.0):
        return time.time() - self.s["hb_t"] < max_age

    # ------------------------------------------------------------------ commands
    def command_long(self, cmd, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, wait=True, timeout=3.0, retries=2):
        for attempt in range(retries + 1):
            self.acks.pop(cmd, None)
            self.m.mav.command_long_send(self.tsys, self.tcomp, cmd, attempt, p1, p2, p3, p4, p5, p6, p7)
            if not wait:
                return None
            t0 = time.time()
            while time.time() - t0 < timeout:
                self.pump(0.1)
                if cmd in self.acks:
                    res = self.acks[cmd][0]
                    if res == M.MAV_RESULT_IN_PROGRESS:
                        continue
                    return res
        return None

    def command_int(self, cmd, frame, p1, p2, p3, p4, lat, lon, z, timeout=3.0, retries=2):
        for _ in range(retries + 1):
            self.acks.pop(cmd, None)
            self.m.mav.command_int_send(self.tsys, self.tcomp, frame, cmd, 0, 0, p1, p2, p3, p4,
                                        int(round(lat * 1e7)), int(round(lon * 1e7)), z)
            t0 = time.time()
            while time.time() - t0 < timeout:
                self.pump(0.1)
                if cmd in self.acks and self.acks[cmd][0] != M.MAV_RESULT_IN_PROGRESS:
                    return self.acks[cmd][0]
        return None

    # ------------------------------------------------------------------ modes
    def mode_name(self):
        """Generic name of the current mode ('mission', 'hold', 'rtl', ...)."""
        cm = self.s["custom_mode"]
        if cm is None:
            return None
        if self.stack == "ardupilot":
            for name, num in APM_MODES[self.vclass].items():
                if num == cm and not (name == "hold" and self.vclass == "copter"):
                    return name
            return f"mode{cm}"
        main, sub = (cm >> 16) & 0xFF, (cm >> 24) & 0xFF
        for name, (mm, ss) in PX4_MODES.items():
            if name != "guided" and mm == main and (mm != 4 or ss == sub):
                return name
        return f"px4:{main}.{sub}"

    def mode_is(self, name):
        cm = self.s["custom_mode"]
        if cm is None:
            return False
        if self.stack == "ardupilot":
            return APM_MODES[self.vclass].get(name) == cm
        mm, ss = PX4_MODES[name]
        return ((cm >> 16) & 0xFF) == mm and (mm != 4 or ((cm >> 24) & 0xFF) == ss)

    def set_mode(self, name, timeout=5.0):
        if self.stack == "ardupilot":
            if name not in APM_MODES[self.vclass]:
                raise MavError(f"mode {name} not available on ArduPilot {self.vclass}")
            p2, p3 = APM_MODES[self.vclass][name], 0
        else:
            p2, p3 = PX4_MODES[name]
        t0 = time.time()
        while time.time() - t0 < timeout:
            res = self.command_long(M.MAV_CMD_DO_SET_MODE, M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, p2, p3, wait=True,
                                    timeout=1.0, retries=0)
            if res == M.MAV_RESULT_ACCEPTED:
                return True            # acknowledged; the 1 Hz heartbeat will confirm shortly
            end = time.time() + 1.0
            while time.time() < end:
                self.pump(0.1)
                if self.mode_is(name):
                    return True
        return False

    # ------------------------------------------------------------------ arming
    def arm(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.command_long(M.MAV_CMD_COMPONENT_ARM_DISARM, 1, wait=True, timeout=2.0, retries=0)
            end = time.time() + 2.0
            while time.time() < end:
                self.pump(0.1)
                if self.s["armed"]:
                    return True
            time.sleep(1.0)
        last = [t for (_, sev, t) in list(self.status_text)[-5:]]
        raise MavError("arming refused: " + "; ".join(last))

    def wait_ready(self, timeout=120.0):
        """GPS 3-D fix, a position and a home position: prerequisites for planning."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.2)
            s = self.s
            if s["lat"] is not None and s["gps_fix"] >= 3 and s["home"] is not None:
                return True
            if s["home"] is None and int(time.time() - t0) % 5 == 0:
                self.command_long(M.MAV_CMD_REQUEST_MESSAGE, M.MAVLINK_MSG_ID_HOME_POSITION, wait=False)
        raise MavError(f"vehicle not ready (fix {self.s['gps_fix']}, home {self.s['home']})")

    # ------------------------------------------------------------------ missions
    def upload(self, items, mission_type=0, timeout=3.0, retries=4):
        """Mission protocol upload. items: dicts from mission.py. ArduPilot gets a
        home placeholder at seq 0. Returns the seq offset of items[0]."""
        if self.stack == "ardupilot" and mission_type == M.MAV_MISSION_TYPE_MISSION:
            h = self.s["home"] or (self.s["lat"], self.s["lon"], 0.0)
            items = [dict(cmd=M.MAV_CMD_NAV_WAYPOINT, frame=M.MAV_FRAME_GLOBAL, lat=h[0], lon=h[1], alt=h[2] if len(h) > 2 else 0.0)] + list(items)
            offset = 1
        else:
            offset = 0
        n = len(items)
        for attempt in range(retries + 1):
            self.mission_q.clear()
            self.m.mav.mission_count_send(self.tsys, self.tcomp, n, mission_type)
            sent = set()
            last = time.time()
            while time.time() - last < timeout:
                self.pump(0.05)
                while self.mission_q:
                    msg = self.mission_q.popleft()
                    if getattr(msg, "mission_type", 0) != mission_type:
                        continue
                    if msg.get_type() == "MISSION_ACK":
                        if msg.type == M.MAV_MISSION_ACCEPTED and len(sent) == n:
                            return offset
                        if msg.type != M.MAV_MISSION_ACCEPTED:
                            raise MavError(f"mission rejected: {mavutil.mavlink.enums['MAV_MISSION_RESULT'][msg.type].name}")
                        continue
                    seq = msg.seq
                    if seq >= n:
                        continue
                    it = items[seq]
                    self.m.mav.mission_item_int_send(
                        self.tsys, self.tcomp, seq, it.get("frame", M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT), it["cmd"],
                        0, int(it.get("autocontinue", 1)),
                        float(it.get("p1", 0)), float(it.get("p2", 0)), float(it.get("p3", 0)), float(it.get("p4", 0)),
                        int(round(it.get("lat", 0) * 1e7)), int(round(it.get("lon", 0) * 1e7)), float(it.get("alt", 0)),
                        mission_type)
                    sent.add(seq)
                    last = time.time()
            self.log(f"[janus] mission upload timed out (attempt {attempt + 1}), retrying")
        raise MavError("mission upload failed: no acknowledgement from the flight controller")

    def set_current(self, seq, timeout=4.0):
        """MISSION_SET_CURRENT message: supported by both stacks (PX4 1.17 does not
        accept MAV_CMD_DO_SET_MISSION_CURRENT)."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.m.mav.mission_set_current_send(self.tsys, self.tcomp, seq)
            end = time.time() + 1.0
            while time.time() < end:
                self.pump(0.1)
                if self.s["seq"] == seq:
                    return True
        return False

    def clear_fence(self):
        self.m.mav.mission_clear_all_send(self.tsys, self.tcomp, M.MAV_MISSION_TYPE_FENCE)

    # ------------------------------------------------------------------ guidance
    def goto(self, lat, lon, rel_alt, speed=-1):
        """Fly to a point now (DO_REPOSITION, switches to the guided/hold mode).
        PX4 reads z as AMSL whatever the frame, so convert there."""
        if self.stack == "px4":
            if not self.s["home"]:
                raise MavError("no home position: cannot convert altitude for PX4")
            z = self.s["home"][2] + rel_alt
            frame, p4 = M.MAV_FRAME_GLOBAL_INT, float("nan")
        else:
            z = rel_alt
            frame, p4 = M.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0.0
        res = self.command_int(M.MAV_CMD_DO_REPOSITION, frame, speed, M.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE, 0, p4, lat, lon, z)
        return res == M.MAV_RESULT_ACCEPTED

    def hold(self):
        """Stop and hold here (copter hovers, plane circles)."""
        if self.stack == "ardupilot" and self.vclass == "copter":
            return self.set_mode("guided")       # entering GUIDED holds at the stopping point
        return self.set_mode("hold")

    def takeoff_copter_apm(self, alt, timeout=60.0):
        if not self.set_mode("guided"):
            raise MavError("could not enter GUIDED for takeoff")
        self.arm()
        res = self.command_long(M.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, alt)
        if res != M.MAV_RESULT_ACCEPTED:
            raise MavError(f"takeoff refused (result {res})")
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pump(0.2)
            if (self.s["rel_alt"] or 0) >= 0.9 * alt:
                return True
        raise MavError("takeoff did not reach altitude")

    def statustext(self, text, severity=M.MAV_SEVERITY_INFO):
        """Show a Janus message in QGroundControl / Mission Planner."""
        self.m.mav.statustext_send(severity, text[:50].encode())

    def param_set(self, name, value, ptype=M.MAV_PARAM_TYPE_REAL32, timeout=3.0):
        v = float(value)
        if self.stack == "px4" and ptype == M.MAV_PARAM_TYPE_INT32:
            v = struct.unpack("<f", struct.pack("<i", int(value)))[0]     # PX4 sends int bits in the float field
        for _ in range(3):
            self.m.mav.param_set_send(self.tsys, self.tcomp, name.encode(), v, ptype)
            t0 = time.time()
            while time.time() - t0 < timeout:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if msg and msg.param_id == name:
                    return True
        return False
