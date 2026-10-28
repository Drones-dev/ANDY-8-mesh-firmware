# ANDY 8: RED - Mesh Node Firmware

Firmware for the **ANDY 8: RED** distributed mesh telemetry and node hand-off project
(CS4850 Senior Project, Kennesaw State University).

Each drone carries a small LoRa radio node. The nodes relay telemetry for each other, so if one
node drops out the others reroute around it. 

## Status

| Step | State |
|------|-------|
| Two-board LoRa ping/pong with link-loss detection | Done (bench tested) |
| Node IDs, HELLO beacons, neighbor table | Done (hardware validation pending) |
| Simulated structured telemetry | Done (hardware validation pending) |
| MAVLink UART4 provider | Done (physical UART/PX4 validation pending) |
| Multi-hop forwarding with duplicate prevention | Planned |
| Route selection | Planned |
| Relay failure detection and reroute (5 s target) | Planned |

## Hardware

- Node: Seeed XIAO ESP32S3 + Wio-SX1262 kit (915 MHz, US band)
- Library: [RadioLib](https://github.com/jgromes/RadioLib)
- Language: C/C++ (Arduino framework for ESP32-S3)
- Flight controller (later): MicoAir743v2-AIO-45A, node connects on UART4

## Repo layout

```
firmware/
  ping-test/
    groundStationPingTest/   Ground Station sketch (sends PING, expects PONG)
    drone1/                  Drone 1 sketch (answers PONG)
  mesh-node/                 Shared firmware for all four node profiles
tests/
  protocol_tests.cpp         Host tests for packet serialization and validation
docs/
  hardware-validation.md
```

## Run the Phase 2 mesh firmware

The original ping-test sketches are preserved as the known-working reference.
For mesh firmware setup, per-board identity configuration, simulated telemetry
checks, and MAVLink wiring requirements, follow
[`docs/hardware-validation.md`](docs/hardware-validation.md).

The shared firmware uses nonblocking radio receive/transmit operations and
`millis()` scheduling. Each node broadcasts HELLO, maintains a bounded direct-
neighbor table, and marks a neighbor lost after the configured heartbeat timeout.
Drone profiles transmit serialized telemetry to the Ground Station profile.

## Run the host protocol test

From the repository root:

```sh
c++ -std=c++11 -Wall -Wextra -Werror tests/protocol_tests.cpp firmware/mesh-node/PacketProtocol.cpp -Ifirmware/mesh-node -o /tmp/andy8_protocol_tests
/tmp/andy8_protocol_tests
```

The test checks HELLO and telemetry round trips, byte order, malformed packet
rejection, version checks, identity validation, and telemetry field ranges.

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
