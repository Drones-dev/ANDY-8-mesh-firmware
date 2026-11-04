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
- Removing power from a neighbor changes it to `LOST` after about three seconds (three missed HELLOs).
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

## Packet format

All multi-byte integers use network byte order (big endian). No raw C++ structs
are transmitted.

The 13-byte common header contains two magic bytes, protocol version, packet
type, source ID, destination ID, 16-bit sequence number, 32-bit uptime, and
8-bit payload length. HELLO is 7 bytes; telemetry is 32 bytes (including heading
in centidegrees and GPS fix status). RadioLib's LoRa
CRC protects each over-the-air frame.

## PX4 SIH and mesh simulation

The PC executable is ordinary C++ software; it does not try to run Arduino code
inside PX4. It reuses the packet serializer, telemetry format, direct-neighbor
table, heartbeat timeout, and MAVLink decoder used by hardware mode.

```sh
./scripts/setup-simulation.sh
./scripts/start-px4-sih.sh /path/to/PX4-Autopilot
# in another terminal
./build/andy_mesh_sim
```

PX4's official SIH command is `make px4_sitl_sih sihsim_quadx`; its official
multi-instance launcher command is
`./Tools/simulation/sitl_multiple_run.sh 3 sihsim_quadx px4_sitl_sih`.
Instances 0-2 send offboard MAVLink to UDP ports 14540-14542, which map to mesh
nodes 1-3. See <https://docs.px4.io/main/en/sim_sih/>.

The default radio topology is `0 <-> 1 <-> 2 <-> 3`, with different RSSI, SNR,
and latency values on each direct link. These values are test inputs, not an
SX1262 propagation model. `SimulatedRadioNetwork` also supports per-link enable,
loss percentage, endpoint disconnect, and restoration. The automated test
verifies discovery, restricted reachability, timeout, and recovery.

Useful controls are `--loss-percent 10`, `--disable-link 1 2`,
`--disconnect-node 2`, and `--failure-demo` (disconnect node 2 at five seconds,
restore it at twelve seconds). `--duration-ms` makes scripted runs terminate.

Important limitation: multi-hop forwarding is not implemented. Consequently,
node 1 telemetry can reach ground node 0 in the line topology, while nodes 2 and
3 cannot yet relay telemetry through intermediate nodes. HELLO packets remain
strictly direct-neighbor observations. Do not interpret successful neighbor
discovery as end-to-end routing.

PX4 recommends Ubuntu for development; SIH is the lightest headless option.
WSL2 can build/run PX4 simulation, but host networking/firewall configuration
may be needed and USB flashing is a separate concern. Official setup:
<https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu>.

