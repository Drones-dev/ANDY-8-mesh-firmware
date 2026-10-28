#pragma once

#include <stdint.h>

/*
  Shared mesh terms and small data types

  This file gives the rest of the firmware one consistent vocabulary. For
  example, every module uses the same definition of a node ID, packet type, and
  flight mode. Keeping these definitions in one place prevents two parts of the
  program from assigning different meanings to the same number.

  The enum classes below are stored as uint8_t values (one byte). Small values
  matter on LoRa because every extra byte increases the time that a packet must
  stay on the air.
*/
namespace AndyMesh {

// A NodeId is the small numeric address used to identify one radio node.
using NodeId = uint8_t;

// Node 0 is the initial Ground Station profile. It is defined here instead of
// scattering the number 0 throughout the program.
constexpr NodeId kGroundStationId = 0;

// 255 (all bits set in one byte) means "every node." HELLO beacons use this
// destination because discovery must work before a sender knows its neighbors.
constexpr NodeId kBroadcastNodeId = 0xFF;

// IDs 0-31 leave room for future nodes while still rejecting obviously damaged
// packet headers. The current deployment uses only IDs 0, 1, 2, and 3.
constexpr NodeId kHighestSupportedNodeId = 31;

// A receiver rejects packets with a different protocol version. This prevents
// it from interpreting a future, incompatible packet layout incorrectly.
constexpr uint8_t kProtocolVersion = 1;

// A role describes what a node normally does. It is separate from the node ID,
// so networking code does not have to assume that a particular ID always has a
// particular job.
enum class NodeRole : uint8_t {
  GroundStation = 0,
  Gateway = 1,
  Relay = 2,
  Drone = 3,
};

// PacketType tells the receiver how to decode the bytes after the common packet
// header. Phase 2 supports discovery (HELLO) and flight data (Telemetry).
enum class PacketType : uint8_t {
  Hello = 1,
  Telemetry = 2,
};

// ACTIVE means HELLO beacons are arriving recently. LOST means the heartbeat
// timeout expired, but the table keeps the entry for diagnostics and recovery.
enum class NeighborStatus : uint8_t {
  Active = 0,
  Lost = 1,
};

// These are the common flight modes sent over LoRa. A telemetry provider maps
// its source-specific values (for example MAVLink flags) into this small list.
enum class FlightMode : uint8_t {
  Unknown = 0,
  Manual = 1,
  Stabilized = 2,
  Auto = 3,
  ReturnToLaunch = 4,
  Land = 5,
};

// SystemHealth gives the Ground Station a compact summary of the aircraft's
// condition without tying the radio packet format directly to MAVLink.
enum class SystemHealth : uint8_t {
  Unknown = 0,
  Ok = 1,
  Warning = 2,
  Error = 3,
};

// Each validity flag uses one bit. A set bit means that field is fresh and may
// be displayed. This is especially important for MAVLink: messages arrive at
// different times, so GPS may be current while battery data is stale.
enum TelemetryValidity : uint16_t {
  TelemetryValidPosition = 1U << 0,
  TelemetryValidAltitude = 1U << 1,
  TelemetryValidBattery = 1U << 2,
  TelemetryValidGroundSpeed = 1U << 3,
  TelemetryValidFlightMode = 1U << 4,
  TelemetryValidArmedState = 1U << 5,
  TelemetryValidSystemHealth = 1U << 6,
};

// The mask contains every validity bit understood by protocol version 1. It is
// used both for simulated data (all fields valid) and for rejecting unknown bits
// in received packets.
constexpr uint16_t kKnownTelemetryValidityMask =
    TelemetryValidPosition | TelemetryValidAltitude | TelemetryValidBattery |
    TelemetryValidGroundSpeed | TelemetryValidFlightMode |
    TelemetryValidArmedState | TelemetryValidSystemHealth;

inline bool isValidNodeId(NodeId id) {
  return id <= kHighestSupportedNodeId;
}

// A source must be one real node, but a destination may also be the broadcast
// address. Keeping these checks separate makes that rule explicit.
inline bool isValidDestination(NodeId id) {
  return isValidNodeId(id) || id == kBroadcastNodeId;
}

// The conversion helpers below turn compact numeric enum values into readable
// Serial Monitor text. They do not allocate Arduino String objects, which helps
// avoid memory fragmentation on a small embedded system.
inline bool isValidNodeRole(NodeRole role) {
  return static_cast<uint8_t>(role) <=
         static_cast<uint8_t>(NodeRole::Drone);
}

inline const char* nodeRoleName(NodeRole role) {
  switch (role) {
    case NodeRole::GroundStation:
      return "GROUND";
    case NodeRole::Gateway:
      return "GATEWAY";
    case NodeRole::Relay:
      return "RELAY";
    case NodeRole::Drone:
      return "DRONE";
  }
  return "UNKNOWN";
}

inline const char* flightModeName(FlightMode mode) {
  switch (mode) {
    case FlightMode::Manual:
      return "MANUAL";
    case FlightMode::Stabilized:
      return "STABILIZED";
    case FlightMode::Auto:
      return "AUTO";
    case FlightMode::ReturnToLaunch:
      return "RTL";
    case FlightMode::Land:
      return "LAND";
    case FlightMode::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

inline const char* systemHealthName(SystemHealth health) {
  switch (health) {
    case SystemHealth::Ok:
      return "OK";
    case SystemHealth::Warning:
      return "WARNING";
    case SystemHealth::Error:
      return "ERROR";
    case SystemHealth::Unknown:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

}  // namespace AndyMesh
