#pragma once

#include <stdint.h>

#include "MeshTypes.h"

/*
  Central configuration for one mesh node

  Every timing value, radio setting, and default identity is collected here so
  the rest of the firmware can use named settings instead of hardcoded numbers.
  This makes it clear which values are safe to configure and ensures every
  module agrees about the selected node.
*/

// If a student copied NodeBuildConfig.example.h to NodeBuildConfig.h, include
// those local choices. __has_include lets the shared firmware still compile when
// the optional per-board file does not exist.
#if __has_include("NodeBuildConfig.h")
#include "NodeBuildConfig.h"
#endif

#ifndef ANDY_NODE_ID
// With no local file, build the safe receive-oriented Ground Station profile.
#define ANDY_NODE_ID 0
#endif

// Named choices are clearer than using unexplained 0 and 1 values elsewhere.
#define ANDY_TELEMETRY_SIMULATED 0
#define ANDY_TELEMETRY_MAVLINK 1

#ifndef ANDY_TELEMETRY_SOURCE
#define ANDY_TELEMETRY_SOURCE ANDY_TELEMETRY_SIMULATED
#endif

#ifndef ANDY_HELLO_INTERVAL_MS
// One HELLO per second gives quick discovery without using a blocking delay.
#define ANDY_HELLO_INTERVAL_MS 1000UL
#endif

#ifndef ANDY_NEIGHBOR_TIMEOUT_MS
// Five seconds allows several one-second HELLOs to be missed before declaring a
// link lost. This reduces false alarms from an occasional corrupted LoRa frame.
#define ANDY_NEIGHBOR_TIMEOUT_MS 5000UL
#endif

#ifndef ANDY_TELEMETRY_INTERVAL_MS
// Flight data is sent every two seconds during Phase 2. HELLO traffic is more
// frequent because link discovery and failure detection need faster updates.
#define ANDY_TELEMETRY_INTERVAL_MS 2000UL
#endif

#ifndef ANDY_NEIGHBOR_TABLE_INTERVAL_MS
// Print the full table every ten seconds. Individual HELLOs still update it
// immediately; this interval only limits repeated diagnostic output.
#define ANDY_NEIGHBOR_TABLE_INTERVAL_MS 10000UL
#endif

#ifndef ANDY_MAVLINK_FRESHNESS_MS
// A MAVLink field older than three seconds is marked unavailable instead of
// silently presenting old flight information as current.
#define ANDY_MAVLINK_FRESHNESS_MS 3000UL
#endif

#ifndef ANDY_MAVLINK_SELF_TEST
#define ANDY_MAVLINK_SELF_TEST 0
#endif

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
// There are deliberately no guessed UART pin or baud defaults. A wrong guess
// can prevent communication or connect the wrong hardware signals, so a MAVLink
// build must state values that were checked against the actual aircraft.
#ifndef ANDY_MAVLINK_RX_PIN
#error "Define ANDY_MAVLINK_RX_PIN after verifying the ESP32-to-UART4 wiring."
#endif
#ifndef ANDY_MAVLINK_TX_PIN
#error "Define ANDY_MAVLINK_TX_PIN after verifying the ESP32-to-UART4 wiring."
#endif
#ifndef ANDY_MAVLINK_BAUD
#error "Define ANDY_MAVLINK_BAUD to match the verified PX4 UART4 setting."
#endif
#endif

namespace AndyMesh {

// TelemetrySource selects which object supplies the common TelemetryData. The
// radio and packet code do not need to know whether data is simulated or real.
enum class TelemetrySource : uint8_t {
  Simulated = ANDY_TELEMETRY_SIMULATED,
  Mavlink = ANDY_TELEMETRY_MAVLINK,
};

// All SX1262 settings used by RadioManager. Units are part of each name so, for
// example, 915.0 cannot be confused with a value in hertz.
struct RadioConfig {
  float frequencyMhz;
  float bandwidthKhz;
  uint8_t spreadingFactor;
  uint8_t codingRateDenominator;
  int8_t transmitPowerDbm;
  uint16_t preambleLength;
  float tcxoVoltage;
  int16_t nssPin;
  int16_t dio1Pin;
  int16_t resetPin;
  int16_t busyPin;
  int16_t receiveEnablePin;
};

// One NodeConfig instance is created at startup and passed to each subsystem.
// firmwareMajor/Minor describe this program, while protocolVersion describes
// the over-the-air packet layout; those versions may change independently.
struct NodeConfig {
  NodeId nodeId;
  NodeRole role;
  const char* deviceName;
  uint8_t firmwareMajor;
  uint8_t firmwareMinor;
  uint8_t protocolVersion;
  NodeId groundStationId;
  uint32_t helloIntervalMs;
  uint32_t neighborTimeoutMs;
  uint32_t telemetryIntervalMs;
  uint32_t neighborTableIntervalMs;
  uint32_t mavlinkFreshnessMs;
  TelemetrySource telemetrySource;
  RadioConfig radio;
};

// These two functions contain the initial four-node deployment profiles in one
// place. Code elsewhere uses config.role and config.deviceName rather than
// assuming that ID 2 must always be a relay.
inline NodeRole defaultRoleForNode(NodeId id) {
  switch (id) {
    case 0:
      return NodeRole::GroundStation;
    case 1:
      return NodeRole::Gateway;
    case 2:
      return NodeRole::Relay;
    case 3:
      return NodeRole::Drone;
    default:
      return NodeRole::Drone;
  }
}

inline const char* defaultNameForNode(NodeId id) {
  switch (id) {
    case 0:
      return "Ground Station";
    case 1:
      return "Drone 1 Gateway";
    case 2:
      return "Drone 2 Relay";
    case 3:
      return "Drone 3 Remote";
    default:
      return "Mesh Node";
  }
}

inline NodeConfig loadNodeConfig() {
  // static_assert catches an invalid ID while compiling, before firmware can be
  // uploaded to a board with an unusable network address.
  static_assert(ANDY_NODE_ID >= 0 &&
                    ANDY_NODE_ID <= kHighestSupportedNodeId,
                "ANDY_NODE_ID is outside the supported range.");

  constexpr NodeId nodeId = static_cast<NodeId>(ANDY_NODE_ID);

#ifdef ANDY_NODE_ROLE
  // A local build may override the initial role/profile mapping when the mesh
  // topology changes in a future experiment.
  const NodeRole role = static_cast<NodeRole>(ANDY_NODE_ROLE);
#else
  const NodeRole role = defaultRoleForNode(nodeId);
#endif

#ifdef ANDY_DEVICE_NAME
  const char* deviceName = ANDY_DEVICE_NAME;
#else
  const char* deviceName = defaultNameForNode(nodeId);
#endif

  return NodeConfig{
      nodeId,
      role,
      deviceName,
      0,
      2,
      kProtocolVersion,
      kGroundStationId,
      ANDY_HELLO_INTERVAL_MS,
      ANDY_NEIGHBOR_TIMEOUT_MS,
      ANDY_TELEMETRY_INTERVAL_MS,
      ANDY_NEIGHBOR_TABLE_INTERVAL_MS,
      ANDY_MAVLINK_FRESHNESS_MS,
      static_cast<TelemetrySource>(ANDY_TELEMETRY_SOURCE),
      // These values and pins exactly preserve the bench-tested PING/PONG setup.
      //
      // 915.0 MHz: US-band test frequency used by the existing hardware.
      // 125.0 kHz: channel bandwidth; narrower channels trade speed for range.
      // SF9: spreading factor; a middle setting for range and airtime.
      // Coding rate 4/7: adds three error-correction bits per four data bits.
      // 14 dBm: radio transmit power used during the successful bench test.
      // Preamble 8: standard eight-symbol receiver synchronization pattern.
      // 1.8 V: powers the SX1262 temperature-compensated crystal oscillator.
      // Pins 41/39/42/40: NSS, DIO1 interrupt, RESET, and BUSY.
      // Pin 38: RXEN controls the Wio board's receive-side antenna switch.
      RadioConfig{915.0F, 125.0F, 9, 7, 14, 8, 1.8F, 41, 39, 42, 40,
                  38}};
}

// Runtime validation complements compile-time checks. It catches invalid enum
// overrides, empty names, or zero timing intervals before the radio starts.
inline bool isValidNodeConfig(const NodeConfig& config) {
  return isValidNodeId(config.nodeId) && isValidNodeRole(config.role) &&
         config.deviceName != nullptr && config.deviceName[0] != '\0' &&
         config.protocolVersion == kProtocolVersion &&
         config.helloIntervalMs > 0 && config.neighborTimeoutMs > 0 &&
         config.telemetryIntervalMs > 0 &&
         (config.telemetrySource == TelemetrySource::Simulated ||
          config.telemetrySource == TelemetrySource::Mavlink);
}

}  // namespace AndyMesh
