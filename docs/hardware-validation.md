# Hardware Validation

How to bring the mesh firmware up on the real boards. Multi-hop routing has been
tested in simulation (see [simulation.md](simulation.md)); this document covers
the hardware steps that come next.

## Hardware

- Node: Seeed XIAO ESP32S3 + Wio-SX1262 kit (915 MHz, US band)
- Library: [RadioLib](https://github.com/jgromes/RadioLib)
- Language: C/C++ (Arduino framework for ESP32-S3)
- Radio settings (mesh firmware): 915 MHz, 125 kHz bandwidth, SF7, coding rate
  4/7, 12 dBm, preamble 8, 1.8 V TCXO. The ping-test sketches use SF9; both of
  their boards match each other, so the ping test still works as a reference.
- Flight controller: MicoAir743v2-AIO-45A (`micoair_h743-v2` PX4 target), ESP32 on UART4/TELEM2
- Motors: EMAX ECO II 2004 3000KV (controlled only by PX4)
- GPS/compass: HGLRC M100-5883 (connected only to the flight controller)

## Ping test (run first)

The original sketches under `firmware/ping-test/` are the known-working radio
reference. Run them first whenever wiring or radio behavior is in doubt.

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

## Build one mesh node

1. Install Arduino IDE 2.x, **esp32 by Espressif Systems**, and RadioLib.
2. Open `firmware/mesh-node/mesh-node.ino`.
3. Copy `NodeBuildConfig.example.h` to `NodeBuildConfig.h` in the same folder.
   The copied file is intentionally ignored by Git.
4. Set `ANDY_NODE_ID` to `0`, `1`, `2`, or `3`. The centralized defaults map
   those IDs to the Ground Station, Gateway, Relay, and Remote Drone roles.
5. Leave `ANDY_TELEMETRY_SOURCE` at `0` for initial simulated tests.
6. Select **XIAO_ESP32S3**, enable **USB CDC On Boot**, upload, and open Serial
   Monitor at 115200 baud.

Each board must have a unique ID. Build and upload the same sketch separately for
each board, changing only its local `NodeBuildConfig.h`.

The firmware uses nonblocking radio receive/transmit and `millis()` scheduling.
Each node broadcasts a HELLO every 2 seconds carrying its hop count to the Ground
Station and its next hop, keeps a table of direct neighbors, and marks a
neighbor lost after 5 seconds without a HELLO. Each drone sends telemetry to
its best neighbor (fewest hops, then strongest RSSI, never a neighbor that
relays through it), and relays other drones' telemetry the same way until it
reaches the Ground Station.

## Phase A acceptance checks

- Every active board logs a HELLO about every 2 seconds.
- A receiver adds only nodes whose HELLO it directly receives.
- The neighbor table shows role, RSSI, SNR, status, and HELLO count.
- Removing power from a neighbor changes it to `LOST` after about 5 seconds (two missed HELLOs plus margin).
- Restoring power changes it back to `ACTIVE`.
- Each drone sends simulated telemetry about every two seconds.
- The Ground Station reports the correct originating ID and does not mix values
  or sequence numbers between drones.
- The original PING/PONG sketches still produce the expected output above.

## Phase B acceptance checks (multi-hop, pending)

Place the boards so the Ground Station hears only Drone 1, Drone 1 hears
Drone 2, and Drone 2 hears Drone 3 (distance, walls, or reduced transmit power).

- Each drone reports a route: Drone 1 via Ground Station (1 hop), Drone 2 via
  Drone 1 (2 hops), Drone 3 via Drone 2 (3 hops).
- The Ground Station receives Drone 3's telemetry, relayed by Drone 1.
- With a second path available, removing power from the relay in use reroutes
  the drone within about 5 seconds, and telemetry resumes.
- Check range at SF7: the mesh uses a faster setting than the SF9 ping test,
  so it reaches less far. Confirm each link before relying on it.
- A relayed packet arrives once (no duplicates at the Ground Station).

## MAVLink UART4 wiring — verify before enabling

No ESP32 GPIO numbers are assumed by the firmware. Before selecting MAVLink,
confirm all of the following against the MicoAir743v2-AIO-45A documentation and
the actual harness:

| Flight controller UART4 | ESP32-S3 | Note |
|---|---|---|
| UART4 TX | Configured RX GPIO | Cross TX to RX |
| UART4 RX | Configured TX GPIO | Cross RX to TX; optional if receive-only |
| GND | GND | A common ground is required |

Do not connect a 5 V UART signal to the ESP32-S3. Confirm that UART4 uses 3.3 V
logic. Power the XIAO and flight controller according to their own specifications;
do not infer power compatibility from UART voltage compatibility.

## PX4 UART4 setup

PX4 parameters and port labels vary by board and PX4 release. Identify the
parameter group that corresponds to the physical UART4 connector, configure it
for MAVLink, and record:

- MAVLink instance and mode
- MAVLink 2 or MAVLink 1 framing
- baud rate
- message stream rates
- the exact PX4 release and parameter names used

Set `ANDY_MAVLINK_RX_PIN`, `ANDY_MAVLINK_TX_PIN`, and `ANDY_MAVLINK_BAUD` in the
local `NodeBuildConfig.h`, then set `ANDY_TELEMETRY_SOURCE` to `1`. The build
intentionally fails if any wiring or baud setting is omitted. Install generated
MAVLink C headers compatible with the common dialect.

The provider parses `HEARTBEAT`, `GLOBAL_POSITION_INT`, `GPS_RAW_INT`,
`SYS_STATUS`, `BATTERY_STATUS`, and `VFR_HUD`. Each telemetry field has its own
freshness timer. Stale or missing real values are marked unavailable and are
never replaced with simulated values.

## Packet format

All multi-byte integers use network byte order (big endian). No raw C++ structs
are transmitted.

The 13-byte common header contains two magic bytes, protocol version, packet
type, source ID, destination ID, 16-bit sequence number, 32-bit uptime, and
8-bit payload length. HELLO is 9 bytes (including the sender's hop count to the
Ground Station and its next hop); telemetry is 32 bytes (including heading in
centidegrees and GPS fix status), so a telemetry packet is 45 bytes. RadioLib's
LoRa CRC protects each over-the-air frame.

## Simulation

The PC simulator, PX4 SIH setup, and dashboard are described in
[simulation.md](simulation.md).
