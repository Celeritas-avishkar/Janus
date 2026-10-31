# Janus demonstrator (Round 1)

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

- **Process chain (top):** the three programs with their **PIDs and UDP ports**, plus two link states, because each side only knows its own half of the link: *drone hears GCS* and *GCS hears drone*.
- **Caption bar:** the current tour step, or what the system is doing.
- **Left, Ground station:** drawn **only from packets that crossed the radio**: telemetry, route reports, hazard reports, black-box records.
- **Right, Drone onboard:** the drone's own computer: known hazards (solid), hazards it does not know about yet (dotted), the sensor window, the line of sight to the antenna, and energy. This feed comes over a separate, labelled simulation channel and is **not** the radio link.
- **Packet seismograph:** every datagram's fate as it happens (delivered, lost in the air, dropped, corrupted and caught by CRC, link cut), signal-to-noise traces, link-state bands, and sequence-gap callouts.
- **Controls:** launch, return to launch, resume; inject a pop-up hazard, an unsafe route upload, or a stale weather feed; cut the link, **pull the antenna** (kills the relay process), and loss, latency and corruption sliders.
- **Evidence tab:** the numbers from the current run, plus a downloadable JSON report.
- **How it works tab:** architecture, what is real versus modelled, and judge Q&A.

---

## The stories

Each story is generated from a seed for the drone you pick. It is then **run in the fast simulator before it is offered**, including runs with random radio fading like the live relay. If the story does not actually happen for that drone, you are told so and asked to try another seed.

| Story | What happens | What it proves |
|---|---|---|
| Nominal mission | Plan, upload, ACK, fly | Real UDP upload, telemetry, validation |
| Hazard appears mid-flight | Gust cell (or ash advisory) forms ahead | The uplink lets the drone replan **7–9 cells ahead**, versus its sensor's 4–6 |
| Radio shadow | A tower block blocks line of sight mid-route | Ghost estimate vs black-box truth; failsafe runs with nobody touching anything |
| Bad link + unsafe upload | 30% loss, 12% corruption; operator sends a route through a hazard | CRC catches corruption, retransmits recover, and the drone **rejects** the unsafe route with a reason |
| Detour forces low-energy return | Partial battery; a detour makes the mission unaffordable | Drone turns home on its own at the point of no return |
| Antenna pulled | The relay process is killed mid-flight, then restarted | Failsafe policy, ghost, recovery, black-box reconciliation |

### Hands-free tour (about 3 minutes, loops)
1. Upload over a real UDP link
2. Hazard appears mid-flight
3. Radio shadow: flying blind
4. Antenna pulled (relay process killed and restarted)
5. Bad link and an unsafe upload
6. Aircraft mode: ash advisory
7. Detour forces a low-energy return

Judges can press any control during the tour. The tour keeps running; press **Stop tour** to take over completely.

### A 5-minute manual script
1. **Nominal**, Scout-Q → Generate → Launch. Point at the PIDs, ports and the MISSION tick on the seismograph.
2. During the flight press **Pop-up hazard ahead**. The left screen gets the new picture, and the right screen replans with the old route dashed.
3. Press **Pull antenna**. The relay PID goes grey, both link states go red, and the left screen switches to a ghost with a growing ring while the right screen runs the failsafe. Press **Restart relay process**: the black box draws the true blind track over the ghost, with the error in cells.
4. **Bad link + unsafe upload** → Generate → Launch, then press **Unsafe route upload**. The drone rejects it with a reason. Point at CRC rejects and retransmissions.
5. Switch to **Aircraft**, Sentinel, **Hazard appears mid-flight**. The same software works at 0–40,000 ft.

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

Volcanic ash, buildings, ridges, geofences and TFRs block every drone. LOW turbulence is passable at a cost by default, as the README of `main.py` describes. (`main.py` itself blocks all turbulence; the setting is per drone here.)

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
./run.sh test                  # about 1 minute; the multi-process test runs in real time
```

| Test | What it checks | Result on the build machine |
|---|---|---|
| `test_planner.py` | Equal optimal cost to `main.py`'s own `grid_astar` on 200 random scenarios; ash ceiling rule; per-profile limits; validation codes | 195 equal-cost routes, 4 agreed no-route, 1 skipped (start blocked) |
| `test_protocol.py` | Fragmented message delivered exactly once under 30% loss and 20% corruption; latest-only replacement | pass |
| `test_stories.py` | Every preset completes a nominal mission with zero exposures; uplink detects beyond sensor range; black-box reconciliation; unsafe route rejected; bingo; relay loss | pass |
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
| Ash advisories, TFRs, geofences | Simulated |
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
