# ANDY 8: RED - Mesh Node Firmware

Firmware for the **ANDY 8: RED** distributed mesh telemetry and node hand-off project
(CS4850 Senior Project, Kennesaw State University).

Each drone carries a small LoRa radio node. The nodes relay telemetry for each other, so if one
node drops out the others reroute around it. 

## Status

| Step | State |
|------|-------|
| Two-board LoRa ping/pong with link-loss detection | Done (bench tested) |
| Node IDs, HELLO beacons, neighbor table | Next |
| Telemetry packets (fake data, then MAVLink from the flight controller) | Planned |
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
```

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
