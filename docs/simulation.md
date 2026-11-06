# Simulation Reference

The PC simulation runs the mesh firmware's C++ classes (packet protocol,
neighbor table, router, MAVLink decoder) for four nodes on one computer. It
does not run Arduino code inside PX4. Drones 1-3 get their telemetry from three
PX4 SIH vehicles over MAVLink; node 0 is the Ground Station.

For the step-by-step demo, see the Quick start in the [README](../README.md).

## Build and tests

```sh
./scripts/setup-simulation.sh
```

Downloads the MAVLink C headers into `simulation/third_party/`, builds
everything under `build/`, and runs the three CTest suites:

| Suite | Covers |
|---|---|
| `protocol` | Packet round trips and validation (magic bytes, version, lengths) |
| `simulated_radio` | Simulated links, neighbor discovery, heartbeat timeout, recovery |
| `routing` | Route selection (fewest hops, then RSSI), reroute when a neighbor is lost, loop prevention, duplicate filtering |

After the first run, rebuild and retest with:

```sh
cmake --build build && (cd build && ctest)
```

## Mesh simulator options

```sh
./build/andy_mesh_sim [options]
```

| Option | Effect |
|---|---|
| `--topology line` | Default. Ground - Drone 1 - Drone 2 - Drone 3; no backup path |
| `--topology diamond` | Drones 1 and 2 reach the Ground Station; Drone 3 reaches both drones but not the Ground Station. Drone 3 relays through Drone 2 (stronger signal) and reroutes through Drone 1 if Drone 2 is lost |
| `--topology distance` | Links follow the drones' PX4 positions (see below) |
| `--range-m N` | Radio range for `distance` (default 60 m) |
| `--gs-lat L --gs-lon L` | Fixed Ground Station position for `distance`; otherwise Drone 1's first position fix |
| `--status FILE` | Writes the network state as JSON for the dashboard |
| `--failure-demo` | Disconnects node 2 at 5 s and restores it at 12 s |
| `--disconnect-node N` | Starts with node N disconnected |
| `--disable-link A B` | Starts with the link between A and B disabled |
| `--loss-percent P` | Random packet loss on every link |
| `--duration-ms N` | Stops after N ms (for scripted runs) |

While it runs, typing `1`, `2`, or `3` and Enter turns that drone's radio off
or back on.

Reroute demo without PX4 (HELLO and routing run; telemetry lines appear only
when PX4 is sending MAVLink):

```sh
./build/andy_mesh_sim --topology diamond --failure-demo
```

In the `line` and `diamond` layouts, link RSSI, SNR, and latency are fixed test
inputs, not an SX1262 propagation model.

## PX4 SIH

```sh
./scripts/start-px4-sih.sh ~/PX4-Autopilot
```

Runs PX4's multi-instance launcher
(`Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_default`; the
standard `make px4_sitl` build includes SIH). Set `PX4_HOME_LAT`,
`PX4_HOME_LON`, and `PX4_HOME_ALT` first to choose the launch location.

| Port | Used by |
|---|---|
| UDP 14540-14542 | PX4 instances 0-2 to mesh nodes 1-3 (`andy_mesh_sim`) |
| UDP 14550 / 18570-18572 | Ground-control link used by `fly_scenario.py` |

PX4 recommends Ubuntu for development. WSL2 works for the simulation; USB
flashing is a separate concern. See
<https://docs.px4.io/main/en/sim_sih/> and
<https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu>.

## Distance-based links

With `--topology distance`, two nodes are linked only while they are within
`--range-m` meters of each other, using the positions each drone's PX4 reports.
Links are re-checked five times per second. Signal strength is derived from
distance, so the nearer neighbor wins route ties. The range is a simulation
setting, not a LoRa range estimate.

```sh
./build/andy_mesh_sim --topology distance --range-m 60 --status dashboard/status.json
```

## Dashboard

`dashboard/index.html` reads `dashboard/status.json` once per second (the
simulator rewrites it twice per second) and shows:

- the mesh map: in `distance` mode, drones at their PX4 positions (north up)
  with the Ground Station's range circle and link distances; in `line` and
  `diamond`, a fixed layout
- radio links that are up (green), each drone's active route (blue), and node
  state (green: route, orange: no route, red: radio off)
- a telemetry table per drone: route, position, altitude, battery, packet
  count, and which relay delivered the last packet
- recent events: route changes, lost and recovered neighbors

```sh
cd dashboard && python3 -m http.server 8000
```

Open <http://localhost:8000/index.html>.

## Flight scenario

`simulation/fly_scenario.py` arms the three PX4 drones, takes off, and flies
three formations sized for a 60 m range, at 15, 20, and 25 m altitude:

| Formation | Drone 1 | Drone 2 | Drone 3 | What the mesh shows |
|---|---|---|---|---|
| LINE (30 s) | 40 m E | 80 m E | 120 m E | Drone 3 reaches the ground in 3 hops |
| OVERTAKE (30 s) | 40 m E | 145 m E | 95 m E | Drone 2 now relays through Drone 3 |
| DIAMOND (60 s) | 40 m E, 35 m N | 45 m E, 30 m S | 85 m E | Two paths for Drone 3; turning off Drone 2 reroutes it via Drone 1 |

Then all drones land where they are. Requires `pymavlink`
(`pip install --user pymavlink`).
