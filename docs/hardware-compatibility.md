# Hardware Compatibility Report

This report separates facts verified in manufacturer/PX4 documentation from
items that still require inspection or bench testing. It is not a flight-readiness
approval.

## MicoAir743v2-AIO-45A

MicoAir documents this exact AIO as an STM32H743VIH6 flight controller with a
four-channel 45 A AM32 ESC, seven UARTs, external I2C, no onboard magnetometer,
3-6S input, and 5 V/2 A and 12 V/2 A BEC outputs. Its manual maps UART4 TX/RX
to PA0/PA1 and PX4 `/dev/ttyS3` (TELEM2). MicoAir states that this AIO uses the
MicoAir743v2 core and PX4 target `micoair_h743-v2`. PX4 1.16 release notes and
the official source tree contain that target. Verify the printed board revision
and install the correct bootloader before flashing; the target's existence is
not evidence that this particular aircraft has been bench- or flight-tested.

- Board/manual: <https://micoair.com/en/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-45a-manual>
- PX4 1.16 board addition: <https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/releases/1.16.md>

Use PX4/QGroundControl, not the ESP32, for motor outputs. MicoAir recommends
DShot300 for the integrated ESC. In QGroundControl choose the actual frame
geometry (normally Generic Quadrotor X only if the physical frame is X), assign
outputs with actuator identification, verify direction with props removed, and
then configure the AM32 ESC. Do not copy PID, propeller, cell-count, thrust, or
current-limit values from this repository.

For TELEM2, PX4's documented example is `MAV_1_CONFIG=TELEM 2`,
`MAV_1_MODE=Onboard`, `MAV_1_RATE=0`, `MAV_1_FORWARD=Disabled`, and
`SER_TEL2_BAUD=921600`. The ESP32 `ANDY_MAVLINK_BAUD` must match the chosen
PX4 value. Begin with receive-only ESP32 wiring if practical:

| MicoAir signal | ESP32 signal | Requirement |
|---|---|---|
| UART4 TX (PA0, TELEM2) | chosen RX GPIO | TX crosses to RX |
| UART4 RX (PA1, TELEM2) | chosen TX GPIO | needed for two-way serial |
| GND | GND | common electrical reference |

Confirm connector labels and 3.3 V UART logic on the actual harness. Never use
the TELEM connector's 5 V pin as a UART signal, and do not power one board from
another until the complete power design has been checked.

- PX4 MAVLink peripherals: <https://docs.px4.io/main/en/peripherals/mavlink_peripherals>
- PX4 companion wiring: <https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/companion_computer/pixhawk_companion.md>
- PX4 DShot: <https://docs.px4.io/main/en/peripherals/dshot>

## EMAX ECO II 2004 3000KV

EMAX lists a 3000KV version with a 2004 stator and 12x12 mm mounting pattern.
The official page does not provide a complete propeller/battery/current table
for this exact version, so electrical and thrust compatibility with the frame,
propeller, battery, and 45 A ESC remains a propulsion bench-test item. Motor
direction and output assignment belong to PX4.

- Manufacturer listing: <https://shop.emax-usa.com/collections/20xx/products/eco-ii-2004-brushless-motor-choose-kv>
- PX4 actuator setup: <https://docs.px4.io/main/en/config/actuators>

## HGLRC M100-5883

HGLRC documents an M10 GNSS receiver, 72 channels, 10 Hz output, 115200 baud,
3.3-5 V input, ceramic antenna, and GPS/QZSS, SBAS, GLONASS, BeiDou, and Galileo
reception. The product page does not identify the exact compass IC or publish a
definitive connector pinout/logic-level table. Match the labels on the received
module and cable before powering it.

Connect GPS TX/RX to the flight controller GPS UART (MicoAir maps GPS1 to UART3,
`/dev/ttyS2`) and compass SDA/SCL to the FC external I2C bus (PB9/PB8), plus the
verified supply and ground. PX4 must configure and calibrate the GPS and external
magnetometer. The ESP32 obtains position, heading, and fix status only from
PX4 MAVLink; it does not wire to this module.

- Manufacturer listing: <https://www.hglrc.com/products/m100-5883-gps>

## Remaining physical validation

- Verify the exact board revision, bootloader, harness pin order, UART voltage,
  and ESP32 GPIO choices with a meter and the physical products.
- Build and flash PX4 `micoair_h743-v2_default`; verify sensors, SD logging,
  voltage/current scaling, GPS, compass orientation/calibration, and TELEM2.
- Bench-test propulsion with props removed first, then use the team's approved
  guarded test procedure. No PID or power-system values are supplied here.
- Compile/upload the ESP32 firmware with the actual board package, RadioLib,
  MAVLink headers, and local `NodeBuildConfig.h`; the current workspace has no
  Arduino CLI, so that hardware build has not been executed here.
