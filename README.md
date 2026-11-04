# ANDY 8: RED - Mesh Node Firmware

Firmware for the **ANDY 8: RED** distributed mesh telemetry and node hand-off project
(CS4850 Senior Project, Kennesaw State University).

Each drone carries a small LoRa radio node. Direct-neighbor discovery and
telemetry are implemented; packet relaying and automatic rerouting are planned.

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
| Multi-hop forwarding with duplicate prevention | Planned |
| Route selection | Planned |
| Relay failure detection and reroute (5 s target) | Planned |

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

For three live PX4 SIH vehicles and the verified port mapping, follow
[`docs/hardware-validation.md`](docs/hardware-validation.md). The current line
topology intentionally has no direct 0-to-2, 0-to-3, or 1-to-3 links. Since
multi-hop forwarding is not implemented, only directly reachable telemetry is
expected at the ground station.

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
