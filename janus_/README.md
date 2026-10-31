# Janus flight-controller backend (ArduPilot + PX4)

Janus runs on a **companion computer** (Raspberry Pi, Jetson, any Linux board) wired to the flight controller. It plans hazard-aware routes, uploads them as ordinary missions, watches the hazard picture and an onboard ash sensor, and replaces the mission in flight when the route stops being safe. **One code base for ArduPilot and PX4, multirotor and fixed-wing.** It speaks plain MAVLink (pymavlink), so no firmware changes are needed.

```
 zone file / UDP advisories ──►┌──────────────────────── companion computer ─────────────────────────┐
                               │  zones.py   hazard picture (TFR, geofence, ash plume, drift, ...)   │
   OPC / SO2 sensor (virtual ─►│  sensor.py  ash sensing (virtual OPC now, real driver slot)         │
   for now, real driver slot)  │  planner.py 3-D grid A*, Dijkstra home field, LOS smoothing,        │
                               │             exits from margins, hold patterns, fixed-wing landing   │
                               │  mission.py route ─► MAVLink mission items                          │
                               │  agent.py   state machine: replan / escape / hold / bingo / override│
                               │  mav.py     ALL ArduPilot-vs-PX4 differences live here              │
                               └──────────────────────────────┬──────────────────────────────────────┘
                                                              │ MAVLink 2 (serial 921600, or UDP/TCP in SITL)
                               ┌──────────────────────────────▼──────────────────────────────────────┐
                               │ flight controller: ArduPilot 4.6 / PX4 1.17 (flies, own failsafes)  │
                               └─────────────────────────────────────────────────────────────────────┘
```

## Safety model (read this first)

| Who | Does what |
|---|---|
| **Autopilot** | Flies: stabilisation, navigation, landing, and **its own failsafes** (battery, RC loss, GCS loss, fence). Janus never disables them. |
| **Janus** | Plans and supervises. It uploads missions and replaces them when needed. |
| **Pilot / GCS** | Can take over at any time. **Any mode change Janus did not command makes Janus stop commanding** (`OVERRIDE`). Switching back to Mission/AUTO re-engages it. |

- **The mission lives on the autopilot.** If the companion computer crashes or is unplugged, the vehicle keeps flying the last *safe* route, and its failsafes still work.
- **Janus never keeps flying a route it knows is unsafe.** If a replan fails, it holds position (copter) or turns back to the last clean air, then retries. If there is still no way after 2 minutes, or the battery says so, it goes home.
- **No straight-line RTL.** Janus plans the way home around hazards, and fixed-wings get a planned landing. The autopilot's RTL is only a last resort. On ArduCopter that last resort is SMART_RTL, which retraces the flown, known-clear path.
- **Optional `--fence`** uploads TFRs, geofences and obstacles as exclusion fences, so the autopilot enforces them even without Janus.
- **`--sitl` is simulator-only.** It turns off PX4's RC requirement and makes the simulated battery realistic. Never use it on an aircraft.

## Tested on real SITL (both stacks, both vehicle classes)

Every row is a full flight with no human input, using the real ArduPilot / PX4 firmware compiled for SITL on this machine. Each flight launches, flies around the reported hazards, finds an **unreported drifting ash cloud** with the onboard sensor, replans, and lands. `tools/sitl_test.py` then checks the black box against the *true* zones, including the one Janus could not see.

| Stack | Vehicle (simulator) | Result | What happened | Closest approach to any blocking zone | Longest replan |
|---|---|---|---|---|---|
| ArduCopter 4.6.3 | quad (SITL) | PASS | ash found ×2, escape to clean air, racetrack hold (goal under ash), bingo return, landed | 22 m (margin 20 m) | 0.23 s |
| PX4 1.17.0 | quad (SIH) | PASS | ash found, replan, landed at goal | 22 m | 1.9 s |
| ArduPlane 4.6.3 | plane (SITL) | PASS | ash found, escape, replan, planned route home, planned landing into wind | 104 m (margin 102 m) | 1.3 s |
| PX4 1.17.0 | plane (SIH) | PASS | ash found, replan, planned route home, planned landing, landed | 134 m | 1.1 s |

The black boxes of these four flights are in `evidence/`. The unreported cloud's forecast time (`at_s`) in each example file is set so it is on the route when that simulator's vehicle gets there (PX4's SIH plane climbs out much more slowly). These are **simulation results**, not flight tests. Replan time is wall-clock time from "decide to replan" to "new mission flying". On PX4 it is dominated by mode switches (PX4 confirms modes in its 1 Hz heartbeat).

## Install on the companion computer

```bash
sudo apt install python3-venv
git clone <this repo> janus && cd janus
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest -q tests          # 20 geometry/planning tests, no hardware needed
```

Tested with Python 3.13, and it should run on 3.9 or newer. It needs `pymavlink` and `numpy`.

### Wiring and flight-controller settings

Connect a spare telemetry port (TELEM2) to the companion UART: TX→RX, RX→TX, GND. **Do not power the Pi from the FC port.**

| | ArduPilot | PX4 |
|---|---|---|
| Port as MAVLink 2 | `SERIAL2_PROTOCOL = 2` | `MAV_1_CONFIG = TELEM 2`, `MAV_1_MODE = Onboard` |
| Baud 921600 | `SERIAL2_BAUD = 921` | `SER_TEL2_BAUD = 921600` |
| Fixed-wing landing | lands with `NAV_LAND`; Janus does not use `DO_LAND_START` on ArduPlane | `MIS_TKO_LAND_REQ = 2` (PX4 FW default) is satisfied: Janus plans the landing |
| Wind for ash drift | ArduPlane estimates wind; copters use the zone file's forecast | same (needs an airspeed sensor) |

Then on the Pi: `--connect /dev/ttyAMA0 --baud 921600` (on a Pi 4/5, enable the UART and disable the serial console with `raspi-config`). `examples/janus.service` starts Janus at boot.

## Zones (the hazard picture)

A JSON file with real coordinates. Altitudes are metres above home (the autopilot's relative-altitude frame).

```json
{
  "wind": {"from_deg": 250, "speed_ms": 4},
  "zones": [
    {"id": "TFR1", "type": "tfr", "shape": "circle", "lat": -35.3605, "lon": 149.1680, "radius_m": 120, "alt_max_m": 400},
    {"id": "GF1",  "type": "geofence", "shape": "polygon", "points": [[lat, lon], [lat, lon], [lat, lon]]},
    {"id": "V1",   "type": "ash", "shape": "plume", "lat": ..., "lon": ..., "conc_mg": 8, "r0_m": 35, "spread": 0.18, "decay_m": 600},
    {"id": "A1",   "type": "ash", "shape": "circle", "lat": ..., "lon": ..., "radius_m": 50, "conc_mg": 6, "drift": true, "at_s": 110},
    {"id": "T1",   "type": "turbulence", "sev": 2, "shape": "circle", ...},
    {"id": "B1",   "type": "obstacle", "shape": "polygon", ...}
  ]
}
```

- **Types:** `ash` (severity from concentration, using the EASA SIB 2010-17R4 bands: LOW 0.2–2, MEDIUM 2–4, HIGH ≥4 mg/m³), `tfr`, `geofence`, `obstacle`, `turbulence`.
- **Shapes:** `circle`, `polygon`, `plume`. A plume is a vent whose cloud is carried downwind, spreading and thinning.
- **Motion:** `drift: true` moves a zone with the file's wind; `vn`/`ve` give an explicit velocity; `at_s` means "these coordinates are the forecast position N s after loading".
- `reported: false` hides a zone from the planner. Only the onboard sensor can find it, which is how unreported ash is tested.
- **Live updates:** edit the file (it is re-read every second, and a broken file keeps the previous zones), or send JSON to `--advisory-port`: `{"op": "add", "zone": {...}}` / `{"op": "remove", "id": "A1"}`.
- **SAR mode** (`--sar`) lets search-and-rescue cross TFRs and LOW ash at a cost. Everything else still blocks.

## Running

```bash
# 1. plan only, no aircraft: check the zones and route; writes a file QGroundControl / Mission Planner can open
python -m janus_fc plan --vehicle copter --home -35.363262,149.165237 \
    --goal -35.357423,149.172397,40 --zones examples/zones_ardupilot_copter.json --out check.waypoints

# 2. on the aircraft (the operator arms and switches to AUTO/Mission; Janus supervises)
python -m janus_fc fly --connect /dev/ttyAMA0 --baud 921600 --goal LAT,LON,ALT --zones zones.json --fence

# 3. SITL
python -m janus_fc fly --connect tcp:127.0.0.1:5760 --goal ... --zones ... --auto-launch           # ArduPilot SITL
python -m janus_fc fly --connect udpin:0.0.0.0:14540 --goal ... --zones ... --auto-launch --sitl  # PX4 SITL
```

`--goal` can be repeated for several waypoints. Without `--auto-launch`, Janus uploads the mission and waits for the operator to arm and switch to Mission. That is the recommended way to start on real hardware.

Every Janus decision also appears in QGroundControl / Mission Planner as a status message (`Janus REPLAN: ...`), and the full black box is written to `janus_<time>.jsonl`.

### SITL from scratch

```bash
# ArduPilot (copter and plane)
git clone --recurse-submodules -b Copter-4.6.3 https://github.com/ArduPilot/ardupilot && cd ardupilot
./waf configure --board sitl && ./waf copter plane
python tools/sitl_test.py ardupilot copter --ap-root ~/ardupilot          # from this repo

# PX4 (SIH: PX4's own simulator, no Gazebo needed)
git clone --recurse-submodules -b v1.17.0 https://github.com/PX4/PX4-Autopilot && cd PX4-Autopilot
make px4_sitl_default
python tools/sitl_test.py px4 copter --px4-root ~/PX4-Autopilot
# or with Gazebo: make px4_sitl gz_x500, then: python -m janus_fc fly --connect udpin:0.0.0.0:14540 ... --sitl
```

## What Janus does in flight

| Situation | Response |
|---|---|
| A new/changed zone (file or advisory) blocks the rest of the route | Hold briefly, upload a new mission around it, continue |
| A drifting zone will cross the route within the horizon (20 s copter / 30 s plane) | Same, planned against where the zone is going (swept volume, planned for twice the horizon so it stays valid) |
| OPC reads ≥ 0.2 mg/m³ twice, at the edge (< 2 mg/m³) | Estimate the plume ahead, drift it with the wind, replan around it |
| OPC in the core (≥ 2 mg/m³), or no route out from inside a margin | **180° turn back to the last clean air, never climbing**, then replan from there |
| Goal covered by a hazard | Fly to the nearest clear fix and **hold** (racetrack right/left, figure-8 or orbit, fitted to clear air). Leave straight for the goal when it clears |
| Battery below what the planned way home needs plus the reserve (learned %/min) | Return home on a planned route (bingo) |
| Fixed-wing arrival | Planned route home, orbit down to 40 m, 600 m straight final into the wind, land |
| Pilot / GCS / autopilot failsafe changes mode | Stop commanding (`OVERRIDE`). Resume if put back in Mission |

## ArduPilot vs PX4: what differs (all in `mav.py`)

| | ArduPilot | PX4 |
|---|---|---|
| Mission seq 0 | home (ignored) | first real item |
| Mode encoding | mode number | `main<<16 \| sub<<24` |
| "Hold here" | GUIDED (copter) / LOITER (plane) | AUTO.LOITER |
| `DO_REPOSITION` altitude | honours the frame (relative) | **always AMSL** (frame ignored), so Janus converts |
| Set current item | `MISSION_SET_CURRENT` (both) | `MAV_CMD_DO_SET_MISSION_CURRENT` not accepted in 1.17 |
| `DO_LAND_START` | not used (would need `RTL_AUTOLAND`) | mission frame only |
| Takeoff | copter: GUIDED + `NAV_TAKEOFF`; plane: mission takeoff item | mission takeoff item |

Each of these was checked against the ArduPilot 4.6.3 and PX4 1.17.0 source, and three of them were found because the SITL tests failed.

## Limitations (be honest about these)

- **The ash sensor is virtual.** It reads the zone file at the real GPS position. `sensor.py` has the interface for a real OPC-N3 / SO2-B4 driver, which is not written yet.
- **No terrain.** Altitudes are relative to home. Over hills, set the cruise layers high enough, or add terrain as `obstacle` zones.
- **Flat-earth frame.** Accurate over a few kilometres, not for routes over ~50 km.
- **Brief pauses.** A replan pauses the vehicle in hold for 0.1–2 s while the mission is swapped. Fixed-wings circle for that time.
- **No reactive obstacle avoidance** (lidar, cameras). Only known zones and the ash sensor. Use the autopilot's own avoidance for that.
- **VTOL** is flown as a fixed-wing (transitions left to the autopilot). This is untested.
- **Wind.** Multirotors rarely estimate wind, so their ash drift uses the forecast in the zone file.
- **Not yet flown on a real aircraft.** First flights: open field, an RC pilot ready to take over, no `--auto-launch`.

## Files

```
janus_fc/geo.py       local frame, wind conversion
janus_fc/zones.py     zone file, plume model, advisories, blocking rules (SAR)
janus_fc/planner.py   grid, A*, Dijkstra home field, smoothing, margin exits, holds, landing
janus_fc/mission.py   route -> MAVLink items (speed, takeoff, climb spirals, hold loops, landing)
janus_fc/mav.py       MAVLink link: connect, modes, arming, upload, goto, params (stack differences)
janus_fc/sensor.py    ash sensor interface + virtual OPC
janus_fc/agent.py     the supervisor state machine and black box
janus_fc/__main__.py  command line (plan / fly)
tools/sitl_test.py    end-to-end SITL test with black-box check
tools/make_examples.py  regenerates the example zone files
examples/             zone files for both SITL homes, systemd unit
tests/test_core.py    geometry and planning tests
```
