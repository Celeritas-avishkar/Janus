# Janus

**Adaptive path planning for drones that fly near volcanic ash, weather and restricted airspace, and keep flying when the radio link goes quiet.**

Built by **Team Celeritas** for AVISHKAR 2026 (JAIN Faculty of Engineering & Technology).

![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue) ![UI Gradio 6](https://img.shields.io/badge/UI-Gradio%206-orange) ![Autopilots ArduPilot and PX4](https://img.shields.io/badge/autopilots-ArduPilot%20%7C%20PX4-green)

Janus has two parts that share one planning core:

| Part | What it is | Where |
|---|---|---|
| **Demonstrator** | A ground station, a relay and a drone running as three separate processes over real UDP, with a browser console. Used to show and test the planning and failsafe logic. | this folder (`janus/`, `run.sh`) |
| **Flight-controller backend** | A companion-computer program that plans and supervises missions on real ArduPilot and PX4 autopilots (multirotor and fixed-wing) over MAVLink. | `janus_fc/` (see [Flight-controller backend](#flight-controller-backend-janus_fc)) |

All results below are from simulation. Nothing has been flown on hardware yet.

---

## Contents

- [Why Janus](#why-janus)
- [Quick start](#quick-start)
- [How it works](#how-it-works)
- [Using the console](#using-the-console)
- [Stories and tour](#stories-and-tour)
- [Drone profiles](#drone-profiles)
- [Flight-controller backend](#flight-controller-backend-janus_fc)
- [Tests](#tests)
- [What is real and what is modelled](#what-is-real-and-what-is-modelled)
- [Limitations](#limitations)
- [Troubleshooting](#troubleshooting)
- [Repository layout](#repository-layout)
- [Roadmap](#roadmap)
- [References](#references)

---

## Why Janus

A route that was safe at take-off can stop being safe in flight. A volcanic ash plume drifts, a weather cell moves, an advisory arrives late, or the radio link drops behind a building. Janus is built around four ideas:

1. **Plan, then keep re-planning.** Routes are checked against what the drone knows and re-planned in flight, including around zones that move.
2. **Trust the drone's own sensor over a stale map.** An onboard particle counter can find ash that no advisory reported. The drone escapes, reports it and re-plans.
3. **Never lose the plot when the link drops.** The drone runs its own failsafes. The ground station shows a clearly marked prediction, not a picture that claims to be current.
4. **Never keep flying a route known to be unsafe.** If no safe route exists the drone holds, retries, and falls back to returning home.

## Quick start

Requires **Python 3.10 or newer** (Gradio 6 does not run on 3.8 or 3.9).

```bash
cd janus_demo
./run.sh setup        # creates .venv and installs pinned requirements (--dev adds pytest and pygame)
./run.sh doctor       # checks Python, packages, ports, main.py, a headless sim and a live 3-process UDP link
./run.sh tour         # opens the console with the hands-free tour playing
```

Open **http://127.0.0.1:7860** and press **Play hands-free tour**.

| Command | What it does |
|---|---|
| `./run.sh setup [--dev]` | Create `.venv` and install requirements |
| `./run.sh doctor [--quick]` | Health check; exits non-zero if something needs fixing. `--quick` skips the live UDP test |
| `./run.sh ui [args]` | Start the console |
| `./run.sh tour` | Start with the hands-free tour playing |
| `./run.sh share` | Also create a temporary public Gradio link (venue Wi-Fi often blocks this) |
| `./run.sh lan` | Serve the console to other devices on the same Wi-Fi or hotspot and print the address |
| `./run.sh test [args]` | Run the test suite (needs `setup --dev`) |
| `./run.sh logs` | Show the latest relay and drone process logs |
| `./run.sh clean` | Remove `.venv` and caches |

Useful arguments for `ui`, `tour`, `share` and `lan`: `--port 7861` (another web port) and `--port-base 49800` (if UDP ports 47800 to 47812 are busy).

<details>
<summary><b>Windows</b></summary>

Run `run.sh` from **Git Bash** or **WSL**. Without them:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m janus.doctor
python run_janus.py --tour
```

If `./run.sh` says "permission denied", run `bash run.sh setup` or `chmod +x run.sh` once. On first run Windows may ask whether Python can use the network. Everything binds to `127.0.0.1` by default, so "Private networks" is enough.
</details>

---

## How it works

### Architecture

```
  Ground station                 Relay                      Drone
  (console + gcs_brain)     (link budget, loss)        (drone_brain)
  hazard picture, uploads   <------ UDP ------>   <------ UDP ------>   own map, validation,
  link monitor, ghost                                                   replanning, failsafes, black box
```

Three operating-system processes talk over real UDP sockets (ports 47800 to 47812). The relay stands in for the radio: it applies free-space loss, obstruction loss when a building or ridge blocks line of sight, and random loss and corruption. Killing the relay is a genuine disconnection.

The protocol (`janus/protocol.py`) adds framing, CRC32, fragmentation, selective retransmit and latest-only messages on top of UDP.

### Planning

A\* is the core planner, ported from the team's `main.py` (10 moves, octile heuristic, ash ceiling rule) and parity-tested against it. Three planner modes are available:

| Planner | What runs |
|---|---|
| **Hybrid (default)** | A\* for the route and for threats that arrive by uplink. When the drone's own sensor finds something ahead, a BendyRuler-style local planner veers off at once, with no replan pause, then A\* rejoins the mission. |
| A\* only | A\* for everything. |
| Dijkstra + BendyRuler | ArduPilot-style split (`OA_TYPE 3`): Dijkstra for the map, BendyRuler for what the sensor finds. |

Why both A\* and Dijkstra: A\* answers "best route from here to the goal". A reverse Dijkstra **cost-to-home field** answers "can I still get home from every cell, and how far is it?" in one pass. That makes return-to-launch and low-energy checks instant and gives the route home without another search.

The BendyRuler here probes a fixed-length ruler along 8 bearings plus up and down, keeps the rulers that are clear, and picks the one closest to the target bearing. It is modelled on ArduPilot's object avoidance, not a port of its code, and like the original it does not guarantee a shortest path.

Routes are flown with rounded corners (a quadratic Bézier from mid-leg to mid-leg). The curve stays inside the corner cell and A\* never cuts a blocked corner, so rounding cannot clip an obstacle (tested).

### Sensing volcanic ash

The drone carries an **optical particle counter** (modelled on the Alphasense OPC-N3: 0.35 to 40 µm, 1 to 30 s histogram period, saturates at 10,000 particles/s) and an **SO₂ cell** (modelled on the SO2-B4: 0 to 100 ppm, t90 under 60 s). Both measure the air the drone is in. Neither can see ahead.

- The **OPC** reacts within one sample. Plume edges are diffuse, so a dense plume is usually picked up at its edge.
- The **SO₂ cell** follows the true value with a first-order lag. At small-UAS speed, 60 s is several hundred metres, so SO₂ confirms the ash is volcanic but is too slow to steer by.
- **Forward warning comes only from advisories over the uplink.** That is the main argument for the ground-station link.

Demo time is compressed (one demo second is about 13 real seconds for small UAS and about 186 for aircraft). Sensor lags, drift and ash dose are converted with that factor so they stay in proportion.

**Encounter response.** OPC above 0.2 mg/m³ (the lower edge of EASA's LOW band) for two samples in a row triggers one of two responses:

- **Edge (below 2 mg/m³):** the plume is assumed ahead and the drone veers off.
- **Core (2 mg/m³ or more):** the drone is inside. It makes a **180° turn back along its own track to the last clean air, never climbing**, then plans around the plume from there. This follows the published advice for inadvertent ash encounters: turn out, do not climb. The plume estimate is reported to the ground station and grows on each re-encounter.

**Measured-wind tracking.** When the OPC finds ash, the drone measures the wind (direction ±4°, speed ±6%, modelled), advects its own ash samples with it, and centres its plume estimate on them. Both views draw the projected track with +N s ticks and the swept swath. The estimate is downlinked, so the ground station sees the same forecast.

**Degradation (illustrative).** Cumulative ash dose wears motors or engines, which reduces thrust and raises power draw. Below 60% health the mission is aborted home; below 25% the drone makes an emergency landing. Fixed-wing drones get a pitot-erosion warning, and OPC optics foul slowly.

### The ash model

Each world has 0 to 2 active vents, placed at least 1.6 cells from launch and every waypoint. A vent emits a puff every 0.8 to 3 s. The wind carries each puff downwind; as it ages it spreads (radius r₀ + ṙ·age) and thins (c = c₀·(r₀/r)²·e^(−age/τ)). Puffs below 0.2 mg/m³ are dropped. *Eruption strength* sets c₀ (Weak 3, Moderate 6, Strong 11 mg/m³ at the vent).

For small UAS the plume starts at the surface; for aircraft it is an umbrella cloud just under the column top. The ground station, relay and drone expand the same vent description, so they all see the same plume.

This is a kinematic puff model of our own (advection, spreading, decay), not a dispersion model such as HYSPLIT or NAME. The shape is realistic; the numbers are illustrative.

Severity follows the three concentration areas in **EASA Safety Information Bulletin 2010-17R4**:

| Band | Concentration | Colour | Effect |
|---|---|---|---|
| LOW | 0.2 to 2 mg/m³ | cyan | Passable only in SAR mode, at a high cost |
| MEDIUM | 2 to 4 mg/m³ | grey | Blocks every drone |
| HIGH | 4 mg/m³ and above | red | Blocks every drone |

### Holding patterns

If the goal, a waypoint or home is covered by a zone the drone cannot enter, it flies to the nearest clear fix and holds a **flown path**. It leaves the hold straight for the site when the site clears. If the site never clears, it returns home at its point of no return, or lands at the fix if home was never reachable.

| Pattern | Shape |
|---|---|
| Standard racetrack (right turns), default | ICAO standard hold: inbound leg to the fix, right turns, outbound leg |
| Racetrack, left turns | Non-standard hold, as published where terrain or airspace requires |
| Figure-8 | Two loops behind the fix along the inbound course, for a narrow box |
| Orbit | Constant-radius circle about the fix |

The pattern must fit in clear cells. If it does not, other headings and sizes are tried, then the other patterns, then a tight orbit.

### Moving zones

Weather zones are boxes or circles and can drift with the wind (still, half wind, full wind). Advisories carry the motion (for example `MOV E 9KT`), so the ground station and drone both know where a zone is going. Planners avoid the area each zone will sweep over the next 4 s and re-check the route twice a second.

### Search and rescue mode

SAR mode keeps working in degraded conditions: gust limit +8 kt, only HIGH turbulence blocks, rain-rated, may cross LOW ash and temporary flight restrictions (TFRs) at a high cost, landing reserve cut to 60%, lost-link timeout doubled with failsafe set to continue, sensor range +2 cells. MEDIUM and HIGH ash and solid obstacles still block. *Reports* compares the same route with SAR off and on.

### Link loss: two truths

- The **ground station** shows only what has arrived. With a live link it shows what the drone has confirmed. When the link is lost it switches to a **PREDICTED VIEW**: a banner with the time since the last packet, a grey veil, predicted zone envelopes, the drone as a ghost with a growing uncertainty ring, and drone-reported ash grown with age.
- The **drone** runs its failsafe on its own (return to link, return to launch, hold then RTL, or continue, depending on the profile) and records a black box.
- When the link returns, the black box is downlinked. The real blind track is drawn in magenta over the estimate and the error is printed.

---

## Using the console

- **Mission setup** (left panel, collapsible): platform, drone, story, weather, SAR mode, route planner, seed, uplink quality, *Customize drone*, map location, **Generate & pre-flight**.
- **Command bar:** Launch, Return to launch, Resume. Ash ahead (reported), Ash ahead (sensor only), Unsafe upload. Pull antenna, Cut link. Clock, **Stop clock**, **RESET**.
- **RESET** stops the mission, puts the drone back on the pad, zeroes the clock and timers, restores the antenna and link, and removes injected ash.
- **Stop clock** freezes the mission clock in both the ground station and the drone process. The drone stops in place and the link is not declared lost, since packets keep flowing.
- **Drone views:** ground station and drone onboard side by side, a grid or OpenStreetMap backdrop, a telemetry table (ground station against drone, row by row) and a weather card (wind, gusts, visibility, temperature, dew point, QNH, cloud base, rain, a simulated METAR, and volcanic ash advisories). **Focus view** opens either view or both full screen.
- **Edit map:** place Start, Goal and zones (ash, turbulence, gust, rain; low, medium or high; circle or box; drifting; unreported) or erase them. Zones placed in flight form at once.
- **Telemetry:** link statistics, packet statistics, uplink and downlink controls (loss, latency, corruption), event log.
- **Reports:** mission outcome, time, distance, energy, course changes, ash found, exposures, failsafes, algorithms used, ash table, timeline. *Compare planners on this map* runs all planners on the same map. *Download report (JSON)*.

The OpenStreetMap backdrop loads standard OSM tiles in the browser, so it needs internet access (offline it falls back to the grid). It is a backdrop only: buildings, ridges and weather are simulated and do not match the real map. © OpenStreetMap contributors.

---

## Stories and tour

Each story is generated from a seed for the drone you pick, then **run in the fast simulator before it is offered**, including runs with random radio fading. If the story does not actually happen for that drone, the console says so and asks for another seed.

| Story | What happens | What it shows |
|---|---|---|
| **Inside unreported ash: detect and escape** (default) | An ash column forms around the drone mid-route | OPC saturates, 180° turn to clean air, modelled wear, plume reported |
| OPC finds the edge of unreported ash | A plume's diffuse edge is detected | Local planner veers off before the core |
| Ash over the landing site: holding pattern | Ash fall covers the goal, then clears | The drone holds at a clear fix, then lands |
| Nominal mission | Plan, upload, ACK, fly | Real UDP upload, telemetry, validation |
| Ash advisory arrives mid-flight | A plume forms ahead and the advisory feed reports it | Uplink replans 7 to 9 cells ahead, against 4 to 6 for the sensor |
| Radio shadow | A tower block blocks line of sight | Ghost estimate against black-box truth; failsafe with nobody touching anything |
| Bad link and unsafe upload | 30% loss, 12% corruption; an unsafe route is sent | CRC catches corruption, retransmits recover, the drone **rejects** the unsafe route with a reason |
| Detour forces low-energy return | Partial battery and a detour | The drone turns home at its point of no return |
| Antenna pulled | The relay is killed mid-flight, then restarted | Failsafe policy, ghost, recovery, reconciliation |

**Hands-free tour** (about 4 minutes, loops): onboard sensor finds ash; upload over real UDP; ash advisory mid-flight; radio shadow; antenna pulled; bad link and unsafe upload; aircraft-mode advisory; detour forces low-energy return.

### Proving the disconnection is real

1. **Separate processes.** Three PIDs are shown. *Pull antenna* terminates the relay with the operating system's kill.
2. **Real sockets.** In Wireshark on the loopback adapter, filter `udp.port >= 47800 && udp.port <= 47812` to see every telemetry, mission and ACK datagram.
3. **Two truths.** During an outage the left screen shows a dead-reckoned estimate, never the truth.
4. **Reconciliation.** The black box draws the true blind track over the estimate when the link returns.
5. **The air itself.** The relay applies a link budget, so disconnections also happen on their own when the drone flies into a radio shadow.

---

## Drone profiles

Presets by mode. Every value can be changed in *Customize drone*. All values are illustrative, not aircraft specifications.

**Small UAS** (0 to 400 ft, 2.5 × 1.7 km map)

| | Scout-Q quad | Hauler-H hexa | Swift-V VTOL |
|---|---|---|---|
| Ceiling | 400 ft | 300 ft | 400 ft |
| Speed (cells/s) | 2.0 | 1.4 | 2.8 |
| Sensor lookahead | 4 | 5 | 6 |
| Gust limit | 18 kt | 26 kt | 32 kt |
| Rain-rated | no | yes | yes |
| Radio power | 20 dBm | 24 dBm | 27 dBm |
| Lost-link failsafe | return to link | return to launch | continue |

**Aircraft** (0 to 40,000 ft, 125 × 85 km map)

| | Sentinel MALE | Stratos HALE | Ranger tactical |
|---|---|---|---|
| Ceiling | 30,000 ft | 40,000 ft | 20,000 ft |
| Speed (cells/s) | 2.6 | 3.4 | 2.0 |
| Sensor lookahead | 6 | 8 | 5 |
| Turbulence that blocks | HIGH | HIGH | MODERATE and above |
| Radio power | 37 dBm | 40 dBm | 33 dBm |
| Lost-link failsafe | continue | hold, then RTL | return to link |

Volcanic ash, buildings, ridges, geofences and TFRs block every drone (SAR may enter TFRs and LOW ash at a cost). LOW turbulence is passable at a cost by default. `main.py` itself blocks all turbulence; here it is a per-drone setting.

---

## Flight-controller backend (`janus_fc`)

`janus_fc` runs on a **companion computer** (Raspberry Pi or Jetson class) wired to the flight controller over MAVLink. It needs no firmware changes. The autopilot keeps flying and keeps its own failsafes; Janus plans the route, uploads it as a normal mission, watches the ash sensor and zone updates, and replaces the mission when the route stops being safe.

- One code base for **ArduPilot 4.6.3** and **PX4 1.17.0**, multirotor and fixed-wing. The vehicle is identified from the autopilot's heartbeat.
- Any mode change Janus did not command puts it in OVERRIDE: it stops commanding immediately.
- If Janus crashes or the serial link drops, the autopilot keeps flying the last uploaded mission.
- Janus never keeps flying a route it knows is unsafe: it holds and retries, then returns home along a **planned** route, not a straight line.
- Hazards come from a JSON zone file (circle, polygon and plume shapes, with drift), reloaded live, plus optional UDP advisories.

```bash
# plan only, no flight controller
python -m janus_fc plan --home LAT,LON --goal LAT,LON,ALT --zones zones.json --stack px4 --out test.waypoints

# simulator
python -m janus_fc fly --connect udpin:0.0.0.0:14540 --goal LAT,LON,ALT --zones examples/zones_px4_copter.json --auto-launch --sitl
```

**Simulator results** (a hidden ash cloud placed on the route; each flight unattended and checked afterwards against the true zone positions):

| Stack | Vehicle | Closest approach to a zone | Longest replan |
|---|---|---|---|
| ArduPilot 4.6.3 | copter | 22.4 m (margin 20 m) | 0.23 s |
| PX4 1.17.0 (SIH) | copter | 22.5 m | 1.9 s |
| ArduPilot 4.6.3 | plane | 104 m (margin about 102 m) | 1.3 s |
| PX4 1.17.0 (SIH) | plane | 134 m | 1.1 s |

These are simulation results. The ash sensor in them is virtual (zone file read at the GPS position). Before any real flight, use an open field with an RC pilot ready to take over, and do not use `--auto-launch` or `--sitl`. See the backend README for wiring, autopilot parameters, the zone-file format, and the full ArduPilot-against-PX4 comparison.

---

## Tests

```bash
./run.sh setup --dev           # pytest + pygame (pygame only for the main.py parity test)
./run.sh test                  # about 5 minutes; the multi-process test runs in real time
```

| Test file | What it checks |
|---|---|
| `test_planner.py` | Equal optimal cost to `main.py`'s `grid_astar` on 200 random scenarios (195 equal-cost routes, 4 agreed no-route, 1 skipped); ash ceiling rule; per-profile limits; validation codes |
| `test_protocol.py` | A fragmented message delivered exactly once under 30% loss and 20% corruption; latest-only replacement |
| `test_stories.py` | Every preset completes a nominal mission with zero exposures; uplink detects beyond sensor range; black-box reconciliation; unsafe route rejected; bingo; relay loss |
| `test_round2.py` | RTL during a replan pause, after mission complete and after a lost-link failsafe; sensor finds unreported ash for every drone and planner; Dijkstra and cost field agree with A\*; BendyRuler paths are flyable; SAR rules; weather options; live reset and scheduled antenna pull |
| `test_round3.py` | In-situ sensing and SO₂ lag; escape for every drone, never climbing; wear aborts and forces landing; holding for every drone; drift and swept volume; curved path stays in corner cells; map editing |
| `test_round4.py` | Plume dense at the vent and fading downwind; vents clear of launch and targets; SAR crosses the faint tail; every hold pattern is a closed loop; standard racetrack turns right; ash estimate drifts with measured wind; Stop clock freezes both processes without a lost link |
| `test_processes.py` | Three real processes over UDP: pre-flight link, full mission, kill relay then failsafe then ghost then restart then black box |

`main.py` is found automatically when this folder sits inside the Janus repo or next to a clone of it. Otherwise set `JANUS_MAIN_PY`. The backend has its own suite: `python -m pytest tests` inside `janus_fc`'s folder (20 tests).

---

## What is real and what is modelled

| Item | Status |
|---|---|
| Three OS processes, UDP sockets, framing, CRC32, retransmission | **Real** |
| A\* planner and in-flight replanning | **Real**, ported from `main.py` and parity-tested |
| Backend flights on ArduPilot and PX4 | **Real firmware, simulated vehicle** |
| OpenStreetMap tiles | Real map tiles, backdrop only |
| Weather feed (WeeWX field names, METAR line format) | Simulated and seeded; field names follow the WeeWX schema as we understand it |
| Ash advisories, TFRs, geofences | Simulated (ash bands follow EASA SIB 2010-17R4) |
| Ash plume | Our kinematic puff model, not a dispersion model |
| Gust and turbulence severity | Our heuristic |
| BendyRuler | Our grid version, modelled on ArduPilot's; not its code |
| Radio link budget | Simplified: free-space loss plus obstruction; no multipath, Fresnel zones or earth curvature |
| Speeds, energy, limits | Illustrative values |
| Flight dynamics (demonstrator) | Not modelled; grid cells |
| Ash sensor (backend) | Virtual; driver slot for a real OPC is empty |
| Link security | Not implemented: packets are not authenticated |
| Replan times | Measured on a laptop, not on a flight computer |

## Limitations

- Not flown on hardware. All results are simulation.
- The backend has no terrain data; altitudes are relative to home, so cruise heights must clear any hills.
- The backend uses a flat-earth frame around home, suited to local flights.
- Replans pause the drone for 0.1 to 2 s in simulation.
- No reactive obstacle avoidance in the backend beyond the zone map and the ash sensor.
- VTOL is flown as fixed-wing and is untested.
- On multirotors the backend uses the zone-file forecast for drift unless the autopilot reports wind.
- Two-machine mode of the demonstrator (`--no-spawn-drone`, `python -m janus.drone --relay-host …`) exists but is untested.
- The planner has not been benchmarked on a Raspberry Pi or Jetson.

## Troubleshooting

| Symptom | Fix |
|---|---|
| "Cannot bind UDP …" | Another Janus is running or the ports are taken: `./run.sh ui --port-base 49800`. `./run.sh doctor` names the busy ports |
| Link never connects | Run `./run.sh doctor`; its last check starts all three processes and waits for a two-way link. Then `./run.sh logs` |
| "Could not verify this story for this drone" | That drone cannot tell that story with that seed (for example, too fast for the shadow). Try another seed or drone |
| Fonts look plain | Google Fonts are not reachable offline; system fonts are used and nothing else changes |
| `share` link fails at the venue | Run locally on a phone hotspot with `./run.sh lan` and open the laptop's IP from other devices |

## Repository layout

```
run.sh                setup / doctor / ui / tour / share / lan / test / logs / clean
run_janus.py          start everything
janus/doctor.py       health check used by ./run.sh doctor
janus/planner.py      A* ported from main.py + route validation
janus/world.py        seeded scenarios, ground truth, weather feed, simulated METAR line
janus/radio.py        link budget: free-space loss + line-of-sight obstruction
janus/air.py          packet fate model (shared by relay and simulator)
janus/protocol.py     UDP framing, CRC32, fragmentation, selective retransmit, latest-only messages
janus/drone_brain.py  drone logic: known map vs truth, validation, replanning, energy, failsafes, black box
janus/gcs_brain.py    ground-station logic: hazard picture, uploads, link monitor, ghost, reconciliation
janus/relay.py        air relay process
janus/drone.py        drone agent process
janus/runtime.py      ground-station runtime, process manager, commands, hands-free tour
janus/scenarios.py    story builders with simulation checks
janus/sim.py          fast headless simulator (same code paths, no sockets)
janus/ui/             Gradio app and the live console (HTML/CSS/JS)
janus_fc/             flight-controller backend for ArduPilot and PX4 (MAVLink)
tests/                planner parity, protocol, stories, real multi-process UDP
```

## Roadmap

- Live weather adapters with saved fixtures: Open-Meteo, AWC METAR, a WeeWX station
- A parser for a saved, real volcanic-ash advisory
- A real OPC-N3 and SO₂ driver for the backend, then bench and field tests
- Gazebo and hardware-in-the-loop runs of the backend
- Two or three drones at once on one map
- Seeded A/B comparison table over many runs
- Two-machine mode (drone on a second laptop or a Pi), tested

## References

- EASA, *Safety Information Bulletin 2010-17R4*, volcanic ash contamination concentration areas.
- Alphasense OPC-N3 and SO2-B4 datasheets (sensor ranges used in the sensor models).
- ArduPilot object avoidance documentation (BendyRuler and Dijkstra, `OA_TYPE`), for the planner split.
- ICAO holding-pattern conventions, for the racetrack hold.
- MAVLink protocol, ArduPilot and PX4 documentation, for the backend.

## Credits

Team Celeritas, AVISHKAR hackathon, JAIN Faculty of Engineering & Technology. Licence: to be added by the team before publishing.
