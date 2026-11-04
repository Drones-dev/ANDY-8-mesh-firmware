#pragma once

#include <stddef.h>
#include <stdint.h>

#include "MeshTypes.h"

/*
  ANDY 8: RED over-the-air packet format

  Serialization means converting useful C++ values into a known sequence of
  bytes for transmission. Deserialization performs the reverse operation after
  a packet arrives. The firmware writes every field explicitly instead of
  transmitting a raw C++ struct, because compilers may add invisible padding or
  store multi-byte numbers in a different byte order.
*/
namespace AndyMesh {

// The first two bytes are a recognizable signature. A receiver can quickly
// reject unrelated LoRa traffic before interpreting the remaining bytes.
constexpr uint8_t kPacketMagic0 = 0xA8;
constexpr uint8_t kPacketMagic1 = 0x52;

// The common header is always 13 bytes. HELLO and telemetry then add a fixed
// payload. A 64-byte maximum keeps memory bounded and leaves plenty of room
// below the SX1262 LoRa packet limit.
constexpr size_t kPacketHeaderSize = 13;
constexpr size_t kMaxPayloadSize = 64;
constexpr size_t kMaxPacketSize = kPacketHeaderSize + kMaxPayloadSize;
// Includes the two routing bytes (hops to ground, next hop).
constexpr size_t kHelloPayloadSize = 9;
constexpr size_t kTelemetryPayloadSize = 32;

// Every packet begins with this information. Source and destination identify
// who sent the packet and who should process it. The sequence number helps track
// packet order, uptime shows when the sender created it, and payloadLength makes
// length validation possible before any payload is decoded.
struct PacketHeader {
  uint8_t protocolVersion;
  PacketType packetType;
  NodeId sourceNodeId;
  NodeId destinationNodeId;
  uint16_t sequenceNumber;
  uint32_t uptimeMs;
  uint8_t payloadLength;
};

// A HELLO payload is a small heartbeat and introduction. Nearby receivers learn
// the sender's identity and role, then refresh that neighbor's last-seen time.
struct HelloPayload {
  NodeId nodeId;
  NodeRole role;
  uint8_t protocolVersion;
  uint32_t uptimeMs;
  // Routing fields used by Router:
  //   hopsToGround - sender's hop count to the Ground Station (0 = Ground
  //                  Station, 255 = no route).
  //   nextHop      - neighbor the sender currently relays through (255 = none).
  uint8_t hopsToGround;
  NodeId nextHop;
};

// TelemetryData is the common format used by both simulated and MAVLink input.
//
// GPS coordinates use degrees multiplied by 10,000,000 (E7). Altitude uses
// centimeters, voltage uses millivolts, and speed uses centimeters per second.
// Scaled integers avoid sending larger floating-point values and produce the
// same byte representation on every compiler.
struct TelemetryData {
  NodeId originatingNodeId;
  uint16_t sequenceNumber;
  uint32_t timestampMs;
  int32_t latitudeE7;
  int32_t longitudeE7;
  int32_t altitudeCm;
  uint8_t batteryPercent;
  uint16_t batteryMillivolts;
  uint16_t groundSpeedCms;
  uint16_t headingCentidegrees;  // 0-35999 means 0.00-359.99 degrees.
  GpsFixStatus gpsFixStatus;
  FlightMode flightMode;
  bool armed;
  SystemHealth systemHealth;
  uint16_t validityFlags;
};

// decodePacket points payload at bytes inside the caller's received buffer. It
// does not allocate or copy another 64-byte array, which saves ESP32 memory.
struct DecodedPacket {
  PacketHeader header;
  const uint8_t* payload;
};

// Protocol functions return a specific error instead of only true/false. Clear
// reasons make malformed-packet diagnostics and host tests easier to understand.
enum class ProtocolError : uint8_t {
  None = 0,
  NullArgument,
  BufferTooSmall,
  BadMagic,
  UnsupportedVersion,
  UnknownPacketType,
  InvalidSource,
  InvalidDestination,
  PayloadTooLarge,
  LengthMismatch,
  InvalidPayload,
  InvalidField,
};

// Converts an error code into short Serial Monitor text.
const char* protocolErrorName(ProtocolError error);

// Builds or validates the common packet envelope. outputLength is set to the
// exact number of bytes that RadioManager should transmit.
ProtocolError encodePacket(const PacketHeader& header,
                           const uint8_t* payload,
                           uint8_t* output,
                           size_t outputCapacity,
                           size_t& outputLength);

ProtocolError decodePacket(const uint8_t* packet,
                           size_t packetLength,
                           DecodedPacket& decoded);

// HELLO and telemetry payloads have separate functions so packet formatting is
// independent of radio transmission. Host tests can verify these functions
// without an SX1262 connected.
ProtocolError encodeHelloPayload(const HelloPayload& hello,
                                 uint8_t* output,
                                 size_t outputCapacity);

ProtocolError decodeHelloPayload(const uint8_t* payload,
                                 size_t payloadLength,
                                 HelloPayload& hello);

ProtocolError encodeTelemetryPayload(const TelemetryData& telemetry,
                                     uint8_t* output,
                                     size_t outputCapacity);

ProtocolError decodeTelemetryPayload(const uint8_t* payload,
                                     size_t payloadLength,
                                     TelemetryData& telemetry);

bool isValidTelemetry(const TelemetryData& telemetry);

}  // namespace AndyMesh
