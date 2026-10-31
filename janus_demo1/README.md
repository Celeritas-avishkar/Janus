# Janus demonstrator (Round 4)

A ground station and a drone, running as **separate programs that talk over real UDP**, with a third program in between standing in for the radio. The ground station builds a weather and hazard picture and uploads missions. The drone checks every route against its own limits, replans around hazards in real time, and runs its own failsafes when the link goes quiet. The planner is ported from the team's `main.py` and tested against it.

Everything runs on one laptop. Open it in a browser and press **Play hands-free tour**.

---

## Run it

Needs **Python 3.10 or newer** (Gradio 6 does not run on 3.8 or 3.9).

```bash
cd janus_demo
./run.sh setup        # creates .venv and installs the pinned requirements (add --dev for pytest + pygame)
./run.sh doctor       # checks Python, packages, ports, main.py, a headless sim and a live 3-process UDP link
./run.sh tour         # opens the console with the hands-free tour playing
```

Then open **http://127.0.0.1:7860**.

| Command | What it does |
|---|---|
| `./run.sh setup [--dev]` | Create `.venv` and install requirements; `--dev` adds pytest and pygame |
| `./run.sh doctor [--quick]` | Health check; exits non-zero if something needs fixing. `--quick` skips the live UDP link test |
| `./run.sh ui [args]` | Start the console (arguments go to `run_janus.py`) |
| `./run.sh tour` | Start with the hands-free tour already playing |
| `./run.sh share` | Also create a temporary public Gradio link (venue Wi-Fi often blocks this: test early) |
| `./run.sh lan` | Serve the console to phones and laptops on the same Wi-Fi or hotspot; prints the address |
| `./run.sh test [args]` | Run the test suite (needs `setup --dev`) |
| `./run.sh logs` | Show the latest relay and drone process logs |
| `./run.sh clean` | Remove `.venv` and caches |

Useful arguments for `ui`, `tour`, `share` and `lan`: `--port 7861` (another web port), `--port-base 49800` (if UDP ports 47800–47812 are busy).

**Windows:** run `run.sh` from **Git Bash** or **WSL**. Without them, do it by hand:

```bash
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
python -m janus.doctor
python run_janus.py --tour
```

If `./run.sh` says "permission denied", use `bash run.sh setup`, or run `chmod +x run.sh` once.

On first run, Windows may ask whether Python can use the network. Everything binds to `127.0.0.1` by default, so "Private networks" is enough.

---

## What is on screen

- **Title:** JANUS, centred. Nothing else in the heading.
- **Mission setup (left, burger ☰ to collapse):** platform, drone, story, weather (with *Weather and ash options*), SAR mode, route planner, seed, uplink, *Customize drone* (four plain settings, the rest under *More settings*), map location, **Generate & pre-flight**.
- **Command bar:** Launch, Return to launch, Resume · Ash ahead (reported), Ash ahead (sensor only), Unsafe upload · Pull antenna, Cut link · clock · **Stop clock** · **RESET**.
- **RESET** stops the mission, puts the drone back on the pad, zeroes the clock and timers, plugs the antenna back in, heals the link and removes injected ash.
- **Drone views tab:** ground station (only what crossed the radio) and drone onboard (what it knows, senses and does) side by side; Grid / OpenStreetMap backdrop toggle; a telemetry table underneath (ground station vs drone, row by row) and a weather card (wind, gusts, visibility, temperature and dew point, QNH, cloud base, rain, a simulated METAR and volcanic ash advisories).
- **Telemetry tab:** chips toggle four panels: Link statistics (PIDs, ports, link states, SNR, line of sight, antenna), Packet statistics (per endpoint), Uplink & downlink (packet seismograph, per-direction delivery, loss/latency/corruption sliders), Event log.
- **Reports tab:** mission report (outcome, time, distance, energy, course changes, ash found, exposures, failsafes, algorithms used, ash table, timeline), *Compare planners on this map*, *Download report (JSON)*.

The Round 1 Evidence and How it works tabs are gone; their numbers live in Reports and Telemetry.

---

## Round 4: volcanic plumes, flown holds, predicted view, focus view

### Volcanic ash is a plume from a vent, not random blobs
Random ash boxes are gone. Each world has 0–2 **active vents** (*Active volcanic vents*), placed at least 1.6 cells from launch and every waypoint. A vent emits a puff every 0.8–3 s. The wind carries each puff downwind; as it ages it spreads (radius r₀ + ṙ·age) and thins (c = c₀·(r₀/r)²·e^(−age/τ)). Puffs below 0.2 mg/m³ (the lower edge of EASA's LOW band) are dropped. The result is the familiar shape: dense and red at the vent, a grey then cyan tail that widens downwind and wanders slightly with the wind. *Eruption strength* sets c₀ (Weak 3, Moderate 6, Strong 11 mg/m³ at the vent). For small UAS the plume starts at the surface; for aircraft it is an umbrella cloud just under the column top. The ground station, relay and drone expand the same vent description, so they all see the same plume. This is a **kinematic puff model of our own** (advection, spreading, decay), not a dispersion model such as HYSPLIT or NAME; the shape is realistic, the numbers are illustrative.

### Trajectory mapping of ash from measured wind
When the OPC finds ash, the drone measures the wind itself (direction ±4°, speed ±6% error, modelled), advects its own ash samples with that wind, and centres its plume estimate on them. The estimate drifts with the **measured** wind, not the true one. Both views draw the projected track as a dashed line with +N s ticks and the swept swath, plus the sample dots the drone took (colour by mg/m³). The estimate is downlinked, so the ground station sees the same forecast track.

### Holding patterns are flown paths
The hold is now a real flight path, chosen in *Mission setup → Holding pattern*:

| Pattern | Shape |
|---|---|
| **Standard racetrack (right turns)** | ICAO standard hold: inbound leg to the fix, right turns, outbound leg. Default |
| Racetrack, left turns | Non-standard hold, as published where terrain or airspace requires |
| Figure-8 | Two loops behind the fix along the inbound course; used by military and survey aircraft to stay in a narrow box |
| Orbit (circle) | Constant-radius circle about the fix, as loitering UAS do |

The pattern is fitted around the fix and must stay in clear cells. If it does not fit, other headings and sizes are tried, then the other patterns, then a tight orbit. The drone flies the pattern along its arc length at its own speed (slower when its motors are worn), and both views draw it with arrows, the fix and the pattern name. **When the site clears, the drone leaves the pattern straight for the site**: no return to an earlier waypoint. The hold route passes through any waypoints before the blocked one first.

### SAR may enter TFRs
With SAR on, temporary flight restrictions are passable at a cost (emergency operations, coordinated with the controlling authority). Geofences, buildings, ridges and MEDIUM/HIGH ash still block.

### Ground station: confirmed picture only while connected
With a live link, the ground station shows what the drone has confirmed: ash it detected, sample dots and the projected track. When the link is lost (in flight), the whole pane switches to a **PREDICTED VIEW**: a banner with the time since the last packet, a grey veil, every zone drawn as a light predicted envelope, the drone as a ghost with a growing uncertainty ring, and drone-reported ash grown with age. Nothing on that pane claims to be current until packets arrive again.

### Focus view and Stop clock
- **Focus view** (button on Drone views) opens full screen with the ground station and drone **side by side**, or either one alone (Split / Ground station / Drone). A control bar keeps Launch, Stop clock, Return to launch, Pull antenna, the clock and Reset at hand. Esc or ✕ leaves.
- **Stop clock** (next to RESET) freezes the mission clock in the ground station **and** in the drone process: the drone stops in place, the clock blinks, and the link is not declared lost because packets keep flowing. **Start clock** carries on from the same instant. RESET is still the replay.

### Smaller fixes
- Routes climb out right after launch and descend only at the end. Before, A* was free to "taxi" along layer 0 for many cells, so an aircraft could show 0 ft en route (a surface-skimming cost now prevents that).
- Dropdowns are pick-only (no typing).
- The Chrome-style tabs no longer have curved "feet" sticking out under the selected tab.
- Map labels stay inside the canvas, and the altitude profile uses the spare height in the focus view.
- The radio shadow story has no vent, so its blind leg is about the link, not ash.

---

## Round 3: realistic ash sensing, escape, holding, moving zones

### Onboard sensors are in-situ (they cannot see ahead)
The drones carry an **OPC** (optical particle counter, e.g. Alphasense OPC-N3: 0.35–40 µm, histogram period 1–30 s, saturates at 10,000 particles/s) and an **SO₂ electrochemical cell** (e.g. Alphasense SO2-B4: 0–100 ppm, t90 < 60 s). Both measure the air the drone is in, so neither gives look-ahead. Round 2's "sensor sees 4–8 cells ahead" is gone.

- The **OPC** reacts within one sample. Plume edges are modelled as diffuse (concentration falls off outside the core with a 0.35-cell e-folding length; that is our assumption), so a dense plume is usually picked up at its edge.
- The **SO₂** cell follows the true value with a first-order lag (t90 from the datasheet). At small-UAS speed, 60 s is several hundred metres, so SO₂ **confirms** the ash is volcanic but is too slow to steer by.
- Demo time is compressed (one demo second ≈ 13 real seconds for small UAS, ≈ 186 for aircraft). Sensor lags, drift and ash dose are converted with that factor so they stay in proportion.
- **Forward warning comes only from advisories over the uplink.** That is now the main argument for the ground-station link.

### Ash encounter: detect and escape
OPC above the alarm (0.2 mg/m³, the lower edge of EASA's LOW band) for two samples in a row means:
- **Edge (reading < 2 mg/m³):** the plume is assumed ahead and the drone veers off (BendyRuler, then A*).
- **Core (≥ 2 mg/m³):** the drone is inside. It makes a **180° turn back along its own track to the last air it sampled clean, never climbing**. That matches the published advice for inadvertent ash encounters: a 180° (descending) turn out, never a climb out. It then plans around the plume from clean air. The plume estimate is reported to the ground station and grows on every re-encounter.

**Degradation (illustrative model):** cumulative ash dose wears motors or engines, which reduces thrust (speed) and raises power draw. Below 60 % health the mission is aborted home; below 25 % the drone makes an emergency landing. Fixed-wing drones get a pitot-erosion warning ("airspeed unreliable"), and OPC optics foul slowly.

### Holding pattern
If the goal, a waypoint or home is covered by a zone this drone cannot enter, it flies to the nearest clear fix and holds (Round 4: a flown racetrack, figure-8 or orbit). It leaves the hold when the site clears. It returns home at its point of no return, or lands at the fix if home was never reachable. The ground station plans the hold before launch if the site is already blocked.

### Moving and circular zones
Weather zones are boxes or circles and **drift with the wind** (Still / ½ wind / full wind in *Weather and ash options*). Advisories carry the motion (`MOV E 9KT`), so the ground station and drone both know where a zone is going. Planners avoid the area each zone will sweep over the next 4 s, and re-check the route twice a second.

### Curved paths
Routes are flown and drawn with rounded corners (a quadratic Bézier from mid-leg to mid-leg). The curve stays inside the corner cell, and A* never cuts a blocked corner, so rounding cannot clip an obstacle (tested).

### Map editing
*Drone views → Edit map*: **Start** and **Goal** (before launch), **Zone** (ash / turbulence / gust / rain; low, medium or high; circle or box; size; drifting; unreported), **Erase**. Click either map. Zones placed in flight form at once.

### UI
Chrome-style tabs, liquid-glass panels (turned off when the OS asks for reduced transparency), a cleaner burger icon, centred zone labels, a full-screen button on each view, and an altitude profile with Auto / 100 / 1,000 / 10,000 ft gridlines and S / M / L height.

### New stories
| Story | What happens |
|---|---|
| **Inside unreported ash: detect and escape** (default) | An ash column forms around the drone mid-route; OPC saturates, 180° turn to clean air, modelled wear, reports the plume |
| OPC finds the edge of unreported ash | The OPC picks up a plume's diffuse edge; BendyRuler veers off before the core |
| Ash over the landing site: holding pattern | Ash fall covers the goal, then clears; the drone holds at a clear fix and lands |

## Volcanic ash is the primary hazard

- Every generated world has ash in both modes: a vent plume from the surface for small UAS, an umbrella cloud aloft for aircraft (Round 4 puff model above).
- Severity follows the three ash concentration areas in **EASA Safety Information Bulletin 2010-17R4**: LOW 0.2–2 mg/m³ (cyan), MEDIUM 2–4 mg/m³ (grey), HIGH ≥ 4 mg/m³ (red). The concentrations themselves are simulated.
- Ash at MEDIUM or HIGH blocks every drone. Only SAR mode may cross LOW ash, at a high cost.
- *Ash ahead · unreported* puts a HIGH plume ahead that no advisory reports: the drone only finds it with its OPC/SO₂ sensors when it gets there (Round 3). It reports the plume back and the ground station's picture updates.

## Route planners (A* stays the core)

| Planner | What runs |
|---|---|
| **Hybrid (default)** | A* for the route and for threats that arrive by uplink. When the drone's **own sensor** finds something ahead, a **BendyRuler-style** local planner veers off at once (no replan pause), then A* rejoins the mission. A reverse **Dijkstra cost-to-home field** answers "can I still get home?" for every cell and gives the route home without another search (RTL, bingo checks). |
| A* only | Round 1 behaviour. |
| Dijkstra + BendyRuler | ArduPilot-style split (OA_TYPE 3): Dijkstra for the map, BendyRuler for what the sensor finds. |

BendyRuler here probes a fixed-length ruler along 8 bearings plus up and down, keeps rulers that are clear and lead somewhere, and picks the one closest to the target's bearing. It is modelled on ArduPilot's object avoidance, not a port of its code, and like the original it does not guarantee a shortest path. *Compare planners on this map* (Reports) runs all of them on the same map and shows time, nodes and cost.

## SAR mode (toggle)

Search and rescue keeps working in degraded conditions: gust limit +8 kt, only HIGH turbulence blocks, rain-rated, may cross LOW ash at a high cost, landing reserve cut to 60 %, lost-link timeout doubled with failsafe = continue, sensor range +2 cells. MEDIUM/HIGH ash and solid obstacles still block. Reports compares the same route with SAR off and on.

## OpenStreetMap backdrop (toggle)

*Drone views → OpenStreetMap* draws standard OSM tiles under the grid (Bengaluru city or Kempegowda Airport, chosen in Mission setup). Tiles load in the browser, so it needs internet; offline it says so and shows the grid. It is a backdrop only: buildings, ridges and weather are simulated and do not match the real map. Attribution "© OpenStreetMap contributors" is drawn on the map.

---

## The stories

Each story is generated from a seed for the drone you pick. It is then **run in the fast simulator before it is offered**, including runs with random radio fading like the live relay. If the story does not actually happen for that drone, you are told so and asked to try another seed.

| Story | What happens | What it proves |
|---|---|---|
| **Onboard sensor finds ash mid-flight** (default) | An unreported HIGH ash plume forms halfway along the route | The drone's own sensor finds it 4–8 cells ahead and changes course mid-flight on its own; checked for every drone and planner |
| Nominal mission | Plan, upload, ACK, fly | Real UDP upload, telemetry, validation |
| Ash advisory arrives mid-flight | An ash plume forms ahead and the advisory feed reports it | The uplink lets the drone replan **7–9 cells ahead**, versus its sensor's 4–6 |
| Radio shadow | A tower block blocks line of sight mid-route | Ghost estimate vs black-box truth; failsafe runs with nobody touching anything |
| Bad link + unsafe upload | 30% loss, 12% corruption; operator sends a route through a hazard | CRC catches corruption, retransmits recover, and the drone **rejects** the unsafe route with a reason |
| Detour forces low-energy return | Partial battery; a detour makes the mission unaffordable | Drone turns home on its own at the point of no return |
| Antenna pulled | The relay process is killed mid-flight, then restarted, **on its own** (the times show in the caption); *Pull antenna* does it by hand at any time | Failsafe policy, ghost, recovery, black-box reconciliation |

### Hands-free tour (about 4 minutes, loops)
1. Onboard sensor finds ash (aircraft)
2. Upload over a real UDP link
3. Ash advisory mid-flight
4. Radio shadow: flying blind
5. Antenna pulled (relay process killed and restarted)
6. Bad link and an unsafe upload
7. Aircraft mode: ash advisory
8. Detour forces a low-energy return

### A 5-minute manual script
1. **Onboard sensor finds ash**, Scout-Q → Generate → Launch. Watch the sensor beam on the right turn red when it finds the plume, BendyRuler veer off, and the plume appear on the left as "reported by drone".
2. Press **Ash ahead · reported**: the advisory reaches the ground station first and the drone replans long before its sensor could see it.
3. Press **Pull antenna**. Both link states go red; the left screen switches to a ghost with a growing ring while the right screen runs the failsafe. Press **Plug antenna back in**: the black box draws the true blind track over the ghost.
4. Press **Return to launch**, then **RESET**: the drone is back on the pad and the clock reads T+0.0.
5. Switch to **Aircraft**, tick **SAR**, Generate. LOW ash now shows "SAR: passable". Open **Reports → Compare planners on this map**.

---

## Proving the disconnection is real
1. **Separate processes.** Three PIDs on screen. "Pull antenna" terminates the relay with the operating system's kill, and nothing can cross until it is restarted.
2. **Real sockets.** Open Wireshark on the loopback adapter and filter `udp.port >= 47800 && udp.port <= 47812`. Every telemetry, mission and ACK datagram is visible.
3. **Two truths.** The left screen only knows what arrived. During an outage it shows a **dead-reckoned estimate**, never the truth.
4. **Reconciliation.** When the link returns, the drone downlinks its **black box**. The real blind track is drawn in magenta over the estimate, and the error is printed.
5. **The air itself.** The relay applies a link budget: free-space loss plus obstruction loss when a building or ridge blocks line of sight. Disconnections happen on their own when the drone flies into a radio shadow. The right screen draws the line of sight and names the obstacle.

---

## Drone profiles

Presets by mode. Every value can be changed in **Customize drone**. All values are illustrative, not aircraft specifications.

| Small UAS (0–400 ft, 2.5 × 1.7 km) | Scout-Q quad | Hauler-H hexa | Swift-V VTOL |
|---|---|---|---|
| Ceiling | 400 ft | 300 ft | 400 ft |
| Speed (cells/s) | 2.0 | 1.4 | 2.8 |
| Sensor lookahead | 4 | 5 | 6 |
| Gust limit | 18 kt | 26 kt | 32 kt |
| Rain-rated | no | yes | yes |
| Radio power | 20 dBm | 24 dBm | 27 dBm |
| Lost-link failsafe | return to link | return to launch | continue |

| Aircraft (0–40,000 ft, 125 × 85 km) | Sentinel MALE | Stratos HALE | Ranger tactical |
|---|---|---|---|
| Ceiling | 30,000 ft | 40,000 ft | 20,000 ft |
| Speed (cells/s) | 2.6 | 3.4 | 2.0 |
| Sensor lookahead | 6 | 8 | 5 |
| Turbulence that blocks | HIGH | HIGH | MODERATE+ |
| Radio power | 37 dBm | 40 dBm | 33 dBm |
| Lost-link failsafe | continue | hold, then RTL | return to link |

Volcanic ash, buildings, ridges, geofences and TFRs block every drone (SAR may enter TFRs and LOW ash at a cost). LOW turbulence is passable at a cost by default, as the README of `main.py` describes. (`main.py` itself blocks all turbulence; the setting is per drone here.)

---

## Files

```
run.sh                setup / doctor / ui / tour / share / lan / test / logs / clean
run_janus.py          start everything
janus/doctor.py       health check used by ./run.sh doctor
janus/planner.py      A* ported from main.py (10 moves, octile heuristic, ash ceiling rule) + route validation
janus/world.py        seeded scenarios, ground truth, WeeWX-style weather feed, simulated METAR line
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
tests/                planner parity, protocol, stories, real multi-process UDP
```

## Tests

```bash
./run.sh setup --dev           # pytest + pygame (pygame only for the main.py parity test)
./run.sh test                  # about 5 minutes; the multi-process test runs in real time
```

| Test | What it checks | Result on the build machine |
|---|---|---|
| `test_planner.py` | Equal optimal cost to `main.py`'s own `grid_astar` on 200 random scenarios; ash ceiling rule; per-profile limits; validation codes | 195 equal-cost routes, 4 agreed no-route, 1 skipped (start blocked) |
| `test_protocol.py` | Fragmented message delivered exactly once under 30% loss and 20% corruption; latest-only replacement | pass |
| `test_stories.py` | Every preset completes a nominal mission with zero exposures; uplink detects beyond sensor range; black-box reconciliation; unsafe route rejected; bingo; relay loss | pass |
| `test_round3.py` | In-situ sensing and SO₂ lag; escape for every drone, never climbing; wear aborts and forces a landing; holding for every drone; drift, swept volume, circles; nominal missions safe with full-wind drift; curved path stays in corner cells; map editing | pass |
| `test_round4.py` | Plume is dense at the vent and fades and spreads downwind; frozen plumes stay put; vents clear of launch and targets; SAR may cross the faint tail; every hold pattern is a closed loop with no jumps; standard racetrack turns right; the drone flies the chosen pattern and leaves straight for the goal; SAR through TFR; ash estimate drifts with the measured wind; Stop clock freezes both processes without a lost link | pass |
| `test_round2.py` | RTL during a replan pause, after mission complete and after a lost-link failsafe; sensor finds unreported ash for every drone and planner; Dijkstra and the cost field agree with A*; BendyRuler paths are flyable; SAR rules; weather options; live reset and scheduled antenna pull | pass |
| `test_processes.py` | Three real processes over UDP: pre-flight link, full mission, kill relay → failsafe → ghost → restart → black box | pass |

`main.py` is found automatically when this folder sits inside the Janus repo, or next to a clone of it. Otherwise set `JANUS_MAIN_PY`.

---

## What is real, and what is modelled

| Item | Status |
|---|---|
| Three OS processes, UDP sockets, framing, CRC32, retransmission | **Real** |
| A* planner and in-flight replanning | **Real**, ported from `main.py` and parity-tested |
| Weather feed (WeeWX field names, METAR line format) | Simulated, seeded. Field names follow the WeeWX schema as we understand it; verify against your station |
| Gust and turbulence severity | Our heuristic |
| Ash advisories, TFRs, geofences | Simulated (ash bands follow EASA SIB 2010-17R4) |
| BendyRuler | Our grid version, modelled on ArduPilot's; not its code |
| OpenStreetMap tiles | Real map tiles, backdrop only |
| Radio link budget | Simplified model: free-space loss + obstruction; no multipath, Fresnel zones or earth curvature |
| Speeds, energy, limits | Illustrative values |
| Flight dynamics | Not modelled (grid cells) |
| Link security | Not implemented: packets are not authenticated |
| Replan times | Measured on the laptop running the demo, not on a flight computer |

## Likely judge questions
- **Does it run on a real flight computer?** The planner is Python and is meant for a companion computer (Raspberry Pi or Jetson class) next to the autopilot. Autopilot firmware such as ArduPilot or PX4 is not Python. We have not benchmarked a Pi yet.
- **Why UDP and not TCP or MQTT?** Telemetry links are commonly datagram based. With UDP we handle loss ourselves, so the failures are visible instead of hidden.
- **How do you know the uplink helps?** The pop-up story is checked in simulation before it runs: replanning 7–9 cells ahead with the uplink, versus at the 4–6-cell sensor limit without it. That is a simulation result, and we word it that way.
- **Is the disconnection faked?** Three PIDs, real sockets (Wireshark), a relay process you can kill, and a black box that shows where the estimate was wrong.
- **What if the link and the planner both fail?** The drone holds or returns on its last known map, never enters a cell its map marks as blocked (counted every run), and retries if no safe route exists.
- **Can it use real weather?** The weather layer is one adapter interface. Live Open-Meteo, AWC METAR and a WeeWX station are the Round 2 work.

## Troubleshooting
| Symptom | Fix |
|---|---|
| "Cannot bind UDP …" | Another Janus is running, or the ports are taken: `./run.sh ui --port-base 49800`. `./run.sh doctor` names the busy ports |
| Link never connects | Run `./run.sh doctor`; its last check starts all three processes and waits for a two-way link. Then `./run.sh logs` |
| "Could not verify this story for this drone" | That drone cannot tell that story with that seed (for example, too fast for the shadow). Try another seed or drone |
| Fonts look plain | Google Fonts are not reachable offline; system fonts are used and nothing else changes |
| `--share` link fails at the venue | Run locally on a phone hotspot with `--ui-host 0.0.0.0` and open the laptop's IP from the judges' phones |

## Deck updates needed
Slide 4 (Tech stack) still says *Pygame* and *Python 3.8*. For this demo it is **Python 3.10+, Gradio 6, UDP between three processes**, with the A* core ported from the Pygame planner.

## Round 2 backlog (not in this build)
- Live weather adapters with saved fixtures: Open-Meteo, AWC METAR, a WeeWX station
- Parser for a saved, real volcanic-ash advisory
- Two or three drones at once on one map
- Seeded A/B comparison table over many runs
- A second flight-computer adapter: JSON over UDP first, then MAVLink in a software simulator if time allows
- Two-machine mode (drone on a second laptop or a Pi): flags exist (`--no-spawn-drone`, `python -m janus.drone --relay-host …`) but it is **untested**
