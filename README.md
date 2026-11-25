# ANDY 8: RED - Mesh Node Firmware

Firmware for the **ANDY 8: RED** distributed mesh telemetry and node hand-off project
(CS4850 Senior Project, Kennesaw State University).

Each drone carries a small LoRa radio node. The nodes find each other, relay
telemetry over multiple hops to the Ground Station, and reroute automatically
when a relay drops out.

## Scope

The mesh-node telemetry and hand-off software is complete and tested in
simulation: three PX4 drones fly in software-in-the-loop (SIH), each one's
telemetry is routed by the same C++ classes that run on the ESP32-S3, and a
browser dashboard shows positions, links, routes, and telemetry. Running the
multi-hop mesh on the real radios and flight controllers is the next phase.

## Status

| Step | State |
|------|-------|
| Two-board LoRa ping/pong with link-loss detection | Done (bench tested) |
| Node IDs, HELLO beacons, neighbor table | Done (hardware validation pending) |
| Simulated structured telemetry | Done (hardware validation pending) |
| MAVLink UART4 provider | Done (physical UART/PX4 validation pending) |
| Shared UART/UDP MAVLink decoder with heading and GPS fix | Done |
| PC simulated radio and four-node line-topology tests | Done |
| Live three-vehicle PX4 SIH integration | Done (three SIH drones, distance-based links, flight scenario) |
| Multi-hop forwarding with duplicate prevention | Done (simulation tested; hardware validation pending) |
| Route selection (fewest hops, then strongest signal) | Done (simulation tested) |
| Relay failure detection and reroute (5 s target) | Done (simulation tested, ~3-5 s) |
| Ground Station test dashboard | Done (map, routes, telemetry, events) |
| Mesh LAND command (`LAND <id>` / `LAND ALL`) with flight-controller reply | Done (bench tested on all three drones) |
| Mesh START (mission launch) and KILL (force disarm) commands | Done (simulation tested; hardware validation pending) |

## Known limitations

- **No acknowledgment or resend.** Telemetry sent to a relay that has just
  failed is lost until the relay is declared lost (5 s without a HELLO), so a
  packet or two can be dropped during a reroute.
- **Shared radio channel.** Nodes use SF7 / 125 kHz (about 0.12 s per 45-byte
  telemetry packet) and send HELLOs every 2 s, which keeps four nodes at roughly
  half of one channel's airtime. Nodes do not check that the channel is free
  before sending (no listen-before-talk), so collisions are still possible. The
  PC simulation does not model airtime or collisions.
- **Route information can be up to 2 s old.** Two drones can briefly pick each
  other as next hop while they move (seen once in simulation, for about 1 s);
  the duplicate filter stops any packet from circling.
- **Simulated links.** Links in the simulator switch on or off at a set range;
  they do not fade gradually like a real radio link.
- **Hardware.** Multi-hop routing and the MAVLink UART link to the flight
  controller have not yet run on the boards. Only the two-board ping test has.

## Quick start for teammates

This runs everything built so far on one PC: three PX4 drones in simulation,
the mesh firmware's routing in the mesh simulator, and the test dashboard.
No drone or radio hardware is needed.

### 1. One-time setup

PX4 needs Linux. On Windows, use WSL2 with Ubuntu (in PowerShell as
administrator: `wsl --install -d Ubuntu`, then restart). Everything below runs
in the Ubuntu terminal.

```sh
# PX4 (takes a while the first time)
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd ~/PX4-Autopilot && make px4_sitl
pip install --user pymavlink

# This repository, on the simulation branch
cd ~
git clone https://github.com/Drones-dev/ANDY-8-mesh-firmware.git
cd ANDY-8-mesh-firmware
git switch px4-full-sim
./scripts/setup-simulation.sh
```

`setup-simulation.sh` downloads the MAVLink headers, builds everything, and
runs the tests. It should end with `100% tests passed, 0 tests failed out of 3`.

### 2. Run the demo (four terminals)

Open four Ubuntu terminals and start each one with `cd ~/ANDY-8-mesh-firmware`.

```sh
# Terminal 1: three PX4 drones, launching from the KSU Kennesaw Campus Green
export PX4_HOME_LAT=34.03825 PX4_HOME_LON=-84.58184 PX4_HOME_ALT=328
./scripts/start-px4-sih.sh ~/PX4-Autopilot

# Terminal 2: mesh simulator, 60 m radio range, writing the dashboard file
./build/andy_mesh_sim --topology distance --range-m 60 --status dashboard/status.json

# Terminal 3: dashboard
cd dashboard && python3 -m http.server 8000

# Terminal 4: flight scenario (start after terminals 1-3 are running)
python3 simulation/fly_scenario.py
```

Open <http://localhost:8000/index.html> in a browser (on Windows too; WSL
forwards localhost).

### 3. What to look for

- **LINE:** Drone 3 is out of the Ground Station's range and reaches it in
  3 hops (Drone 3 > Drone 2 > Drone 1 > Ground Station).
- **OVERTAKE:** Drone 2 flies past Drone 3, and its route flips to go through
  Drone 3.
- **DIAMOND:** when terminal 4 prints `Formation DIAMOND: holding 60 s.`,
  click terminal 2, type `2` and press Enter. Drone 2's radio turns off (red on
  the dashboard), and about 3-5 s later Drone 3 reroutes through Drone 1. Type
  `2` and Enter again to turn it back on.
- **LAND:** all three drones land where they are.
- **Mesh commands:** type `LAND 2`, `START 2` or `KILL 2` (or `ALL`) and Enter
  in terminal 2. The Ground Station floods the command through the mesh, Drone
  2 passes it to its PX4, and terminal 2 prints the result, for example
  `[GROUND] Drone 2 LAND ACCEPTED`. `fly_scenario.py` keeps commanding the
  drones, so it can override a command sent while it is running.

Terminal 2 also prints every relay, route change, and lost neighbor.

### 4. Stop

Press Ctrl+C in terminals 2 and 3, then:

```sh
pkill -x px4
```

## Repo layout

```
CMakeLists.txt                 Host build and test definition
firmware/
  mesh-node/                   Mesh firmware shared by all four nodes
                               (radio, packets, HELLO, neighbor table,
                               router, MAVLink decoder)
  ping-test/                   Original two-board radio test
tests/
  protocol_tests.cpp           Packet serialization and validation
  simulation_tests.cpp         Simulated links, discovery, timeout, recovery
  routing_tests.cpp            Route selection, reroute, loop prevention, duplicates
  command_tests.cpp            Command packets, command duplicates, LAND/START/KILL steps
simulation/
  mesh_sim.cpp                 PC mesh simulator (4 nodes, PX4 MAVLink input)
  SimulatedRadio.h/.cpp        Simulated radio links
  fly_scenario.py              Flies the three PX4 drones through the formations
dashboard/
  index.html                   Ground Station test dashboard (reads status.json)
scripts/
  setup-simulation.sh          Downloads MAVLink headers, builds, runs tests
  start-px4-sih.sh             Starts three PX4 SIH drones
docs/
  simulation.md                Simulator options, dashboard, flight scenario
  hardware-validation.md       Hardware, ping test, board setup, wiring, packet format
```

`build/` and `simulation/third_party/` (MAVLink headers) are generated locally
and are not committed.

## Mesh commands (LAND, START, KILL)

On the Ground Station's Serial Monitor (115200 baud, line ending **Newline**):

```
LAND 2      land Drone 2 where it is
START 2     fly Drone 2's uploaded mission (switch to Mission mode, then arm)
KILL 2      force-disarm Drone 2: motors stop immediately, even in the air
LAND ALL    the same for every drone (also START ALL, KILL ALL)
```

1. The Ground Station sends a Command packet to broadcast three times, 700 ms
   apart. Each drone that hears it relays it once (two copies, random 30-250 ms
   delay), up to 5 hops.
2. The target drone sends the command to PX4 as MAVLink `COMMAND_LONG` on the
   same UART as telemetry, up to five times 500 ms apart until PX4 answers with
   `COMMAND_ACK`:
   - LAND: `MAV_CMD_NAV_LAND` at the current position.
   - START: `MAV_CMD_DO_SET_MODE` to Mission, then, only if PX4 accepted that,
     `MAV_CMD_COMPONENT_ARM_DISARM` (arm). PX4 refuses Mission mode without a
     valid uploaded mission and a position fix, so the drone is never armed
     without a mission to fly.
   - KILL: `MAV_CMD_COMPONENT_ARM_DISARM` with the force flag (disarm).
3. The drone routes the result back to the Ground Station, which prints, for
   example, `[GROUND] Drone 2 START ACCEPTED (cmd 41213, via Node 1)`.

| Result | Meaning |
|--------|---------|
| `ACCEPTED` | PX4 carried out the command (START: Mission mode and armed) |
| `DENIED`, `FAILED`, `TEMPORARILY_REJECTED` | PX4 refused (START: no valid mission, no position fix, or a preflight check failed) |
| `NO_AUTOPILOT` | The node has not heard a PX4 heartbeat in the last 3 s (check the TX/RX wires) |
| `NO_RESPONSE` | PX4 was heard but did not answer |

No reply within about 3 s means the command or the reply was lost; type the
command again. Telemetry shows `Mode: AUTO` during a mission and `Mode: LAND`
while landing. All four nodes must run the same firmware.

## Documentation

- [`docs/simulation.md`](docs/simulation.md): every simulator option and radio
  layout, distance-based links, the dashboard, and the flight scenario.
- [`docs/hardware-validation.md`](docs/hardware-validation.md): hardware list,
  the two-board ping test, building a mesh node for each board, flight
  controller wiring, and the packet format.

## Contributing

- Work on a branch, open a pull request, and have a teammate review it before merging to `main`.
- Commit messages: start with what changed, e.g. `Add neighbor table to node sketch`.
- Tag milestones, e.g. `v0.1-ping-test`.
- Commit source, configuration examples, scripts, tests, and documentation.
  `.gitignore` already excludes `build/`, `simulation/third_party/`,
  `NodeBuildConfig.h`, and the local `archive/` folder.
- Before merging to `main`, build from a fresh clone (`./scripts/setup-simulation.sh`).
