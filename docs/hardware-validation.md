# Hardware Validation

The original sketches under `firmware/ping-test/` remain the known-working radio
reference. Run them first whenever wiring or radio behavior is in doubt.

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

## Phase A acceptance checks

- Every active board logs a HELLO about once per second.
- A receiver adds only nodes whose HELLO it directly receives.
- The neighbor table shows role, RSSI, SNR, status, and HELLO count.
- Removing power from a neighbor changes it to `LOST` after about five seconds.
- Restoring power changes it back to `ACTIVE`.
- Each drone sends simulated telemetry about every two seconds.
- The Ground Station reports the correct originating ID and does not mix values
  or sequence numbers between drones.
- The original PING/PONG sketches still produce the README's expected output.

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

## Synthetic MAVLink parser check

With MAVLink mode configured, define `ANDY_MAVLINK_SELF_TEST 1`. At startup the
firmware generates HEARTBEAT, GLOBAL_POSITION_INT, and VFR_HUD frames, converts
them to raw bytes, feeds those bytes through the same parser used by UART4, and
prints `[MAVLINK SELF-TEST] PASS` or halts on failure.

Disable the self-test for flight testing. The synthetic check validates parsing
and field mapping, but it does not validate the physical UART4 pins, electrical
levels, PX4 stream configuration, or behavior in flight. Those items remain
hardware validation work.

## Packet format

All multi-byte integers use network byte order (big endian). No raw C++ structs
are transmitted.

The 13-byte common header contains two magic bytes, protocol version, packet
type, source ID, destination ID, 16-bit sequence number, 32-bit uptime, and
8-bit payload length. HELLO is 7 bytes; telemetry is 29 bytes. RadioLib's LoRa
CRC protects each over-the-air frame.

