#pragma once

// Copy this file to NodeBuildConfig.h in this directory. That local file is
// ignored by Git so each board can keep its own identity without modifying the
// shared firmware.
//
// Why: every board runs the same networking program, but each physical device
// needs a different address. A local ignored file makes it less likely that one
// person's board identity will accidentally be committed for everybody else.

// Initial deployment identities:
//   0 = Ground Station
//   1 = Drone 1 / Gateway
//   2 = Drone 2 / Primary Relay
//   3 = Drone 3 / Remote Drone
#define ANDY_NODE_ID 1

// Optional overrides. If omitted, role and name come from the ID profile above.
// The role is written as its one-byte NodeRole value:
//   0 = Ground Station, 1 = Gateway, 2 = Relay, 3 = Drone.
// #define ANDY_NODE_ROLE 1
// #define ANDY_DEVICE_NAME "Drone 1 Gateway"

// 0 = simulated telemetry, 1 = MAVLink telemetry.
// Start with simulation so the packet and radio path can be tested without a
// flight controller. Select MAVLink only after UART wiring and PX4 settings are
// known.
#define ANDY_TELEMETRY_SOURCE 0

// MAVLink settings are required when ANDY_TELEMETRY_SOURCE is 1. Set the ESP32
// pins that are physically wired to the flight controller; there are no pin
// defaults because the wiring must be verified first.
// #define ANDY_MAVLINK_RX_PIN 4
// #define ANDY_MAVLINK_TX_PIN 5
// #define ANDY_MAVLINK_BAUD 57600

// Optional: inject generated HEARTBEAT, GLOBAL_POSITION_INT, and VFR_HUD
// frames through the real parser at startup and report PASS/FAIL.
// This checks software parsing only; it cannot prove that physical UART wiring
// or PX4 configuration is correct.
// #define ANDY_MAVLINK_SELF_TEST 1
