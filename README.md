# ANDY 8: RED - Mesh Node Firmware

Firmware for the **ANDY 8: RED** distributed mesh telemetry and node hand-off project
(CS4850 Senior Project, Kennesaw State University).

Each drone carries a small LoRa radio node. Direct-neighbor discovery and
telemetry, multi-hop relaying, route selection, and automatic rerouting are
implemented (tested in PC simulation; hardware validation pending).

## Status

| Step | State |
|------|-------|
| Two-board LoRa ping/pong with link-loss detection | Done (bench tested) |
| Node IDs, HELLO beacons, neighbor table | Done (hardware validation pending) |
| Simulated structured telemetry | Done (hardware validation pending) |
| MAVLink UART4 provider | Done (physical UART/PX4 validation pending) |
| Shared UART/UDP MAVLink decoder with heading and GPS fix | Done |
| PC simulated radio and four-node line-topology tests | Done |
| Live three-vehicle PX4 SIH integration | Adapter built; live PX4 run pending |
| Multi-hop forwarding with duplicate prevention | Done (simulation tested; hardware validation pending) |
| Route selection (fewest hops, then strongest signal) | Done (simulation tested) |
| Relay failure detection and reroute (5 s target) | Done (simulation tested, ~3 s) |

## Hardware

- Node: Seeed XIAO ESP32S3 + Wio-SX1262 kit (915 MHz, US band)
- Library: [RadioLib](https://github.com/jgromes/RadioLib)
- Language: C/C++ (Arduino framework for ESP32-S3)
- Flight controller: MicoAir743v2-AIO-45A (`micoair_h743-v2` PX4 target), ESP32 on UART4/TELEM2
- Motors: EMAX ECO II 2004 3000KV (controlled only by PX4)
- GPS/compass: HGLRC M100-5883 (connected only to the flight controller)

## Repo layout

```
CMakeLists.txt                 Host build and test definition
firmware/
  ping-test/
    groundStationPingTest/   Ground Station sketch (sends PING, expects PONG)
    drone1/                  Drone 1 sketch (answers PONG)
  mesh-node/                 Shared firmware for all four node profiles
tests/
  protocol_tests.cpp         Host tests for packet serialization and validation
  simulation_tests.cpp       Simulated links, discovery, timeout, and recovery
docs/
  hardware-validation.md
simulation/                    PC simulated-radio implementation and SITL adapter
scripts/                       Reproducible simulator/PX4 SIH launch helpers
build/                         Generated locally by CMake; never committed
```

`simulation/third_party/` is also generated locally. The setup script downloads
the MAVLink C headers into that directory, so teammates do not need generated
dependency files stored in Git.

## Run the Phase 2 mesh firmware

The original ping-test sketches are preserved as the known-working reference.
For mesh firmware setup, per-board identity configuration, simulated telemetry
checks, and MAVLink wiring requirements, follow
[`docs/hardware-validation.md`](docs/hardware-validation.md).

The shared firmware uses nonblocking radio receive/transmit operations and
`millis()` scheduling. Each node broadcasts HELLO, maintains a bounded direct-
neighbor table, and marks a neighbor lost after the configured heartbeat timeout.
Drone profiles transmit serialized telemetry to the Ground Station profile.

## Build and run host tests

From the repository root:

```sh
./scripts/setup-simulation.sh
```

This creates all compiler output under `build/` and runs both CTest suites. The
tests cover protocol round trips, validation, direct-link restrictions,
neighbor discovery, heartbeat timeout, and reconnection.

## Run the PC simulation

```sh
./scripts/setup-simulation.sh
./build/andy_mesh_sim --duration-ms 10000
```

Two radio layouts are built in:

- `line` (default): Ground - Drone 1 - Drone 2 - Drone 3. Drone 3's telemetry
  hops through Drone 2 and Drone 1. There is no backup path.
- `diamond`: Drones 1 and 2 both reach the Ground Station; Drone 3 reaches
  both drones but not the Ground Station. Drone 3 relays through Drone 2
  (stronger signal) and reroutes through Drone 1 if Drone 2 is lost.

Reroute demo (Node 2 is disconnected at 5 s and restored at 12 s):

```sh
./build/andy_mesh_sim --topology diamond --failure-demo
```

Telemetry lines appear only when PX4 is running and sending MAVLink.

## Ground Station dashboard

The simulator can write its state to `dashboard/status.json`; `dashboard/index.html`
reads that file once per second and shows the mesh map (links, active routes,
radio state), the latest telemetry per drone, and recent events.

```sh
# Terminal 1: three PX4 drones
./scripts/start-px4-sih.sh ~/PX4-Autopilot

# Terminal 2: mesh simulator writing the status file
./build/andy_mesh_sim --topology diamond --status dashboard/status.json

# Terminal 3: serve the dashboard
cd dashboard && python3 -m http.server 8000
```

Open <http://localhost:8000> in a browser. While the simulator runs, typing
`1`, `2`, or `3` and Enter in terminal 2 turns that drone's radio off or on.

### Distance-based links

With `--topology distance`, two nodes are linked only while they are within
`--range-m` meters of each other, using the positions each drone's PX4 reports.
Signal strength is derived from distance, so the nearer neighbor wins route
ties. The dashboard then draws drones at their real positions (north up) with
the Ground Station's range circle.

```sh
./build/andy_mesh_sim --topology distance --range-m 60 --status dashboard/status.json
```

The Ground Station is placed at Drone 1's first position fix (the launch point)
unless `--gs-lat` and `--gs-lon` are given. The range is a simulation setting,
not a LoRa range estimate.

### Flight scenario

`simulation/fly_scenario.py` takes off the three PX4 drones and flies them
through three formations sized for a 60 m range:

| Formation | Drone 1 | Drone 2 | Drone 3 | What the mesh shows |
|---|---|---|---|---|
| LINE (30 s) | 40 m E | 80 m E | 120 m E | Drone 3 reaches the ground in 3 hops |
| OVERTAKE (30 s) | 40 m E | 145 m E | 95 m E | Drone 2 now relays through Drone 3 |
| DIAMOND (60 s) | 40 m E, 35 m N | 45 m E, 30 m S | 85 m E | Two paths for Drone 3; turning off Drone 2 reroutes it via Drone 1 |

Then all drones land. It uses PX4's ground-control link (UDP 14550), so it runs
alongside `andy_mesh_sim`, which uses 14540-14542.

```sh
# Terminal 4, after PX4, the mesh simulator, and the dashboard are running
python3 simulation/fly_scenario.py
```

Requires `pymavlink` (`pip install --user pymavlink` if it is not already
installed by the PX4 setup script). For three
live PX4 SIH vehicles, follow [`docs/hardware-validation.md`](docs/hardware-validation.md).

## Run the ping test

1. Install Arduino IDE 2.x and the **esp32 by Espressif Systems** board package, then the
   **RadioLib** library.
2. Select **Tools > Board > esp32 > XIAO_ESP32S3** and set **USB CDC On Boot: Enabled**.
3. Upload `drone1` to one board and `groundStationPingTest` to the other.
4. Open the Serial Monitor at **115200 baud** on the Ground Station.

Expected Ground Station output:

```
Ground Station ready.
Ping #1 -> sending PING to Drone 1
LINK OK - Drone 1 answered. Signal strength (RSSI): -34.00 dBm
```

Unplug Drone 1 and after three missed pings it prints `*** NO SIGNAL from Drone 1 ***`.

## Contributing

- Work on a branch, open a pull request, and have a teammate review it before merging to `main`.
- Commit messages: start with what changed, e.g. `Add neighbor table to node sketch`.
- Tag milestones, e.g. `v0.1-ping-test`.
- Commit source, configuration examples, scripts, tests, and documentation.
  `.gitignore` already excludes `build/`, `simulation/third_party/`,
  `NodeBuildConfig.h`, and the local `archive/` folder.
- Before merging to `main`, build from a fresh clone (`./scripts/setup-simulation.sh`)
  and have a teammate review the pull request.
