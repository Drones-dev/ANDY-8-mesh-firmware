#include "PacketProtocol.h"

namespace AndyMesh {
namespace {

// "Network byte order" stores the most significant byte first (big endian).
// These helpers make that ordering explicit for every multi-byte integer.
void writeU16(uint8_t* output, uint16_t value) {
  output[0] = static_cast<uint8_t>(value >> 8);
  output[1] = static_cast<uint8_t>(value);
}

void writeU32(uint8_t* output, uint32_t value) {
  output[0] = static_cast<uint8_t>(value >> 24);
  output[1] = static_cast<uint8_t>(value >> 16);
  output[2] = static_cast<uint8_t>(value >> 8);
  output[3] = static_cast<uint8_t>(value);
}

uint16_t readU16(const uint8_t* input) {
  return static_cast<uint16_t>((static_cast<uint16_t>(input[0]) << 8) |
                               static_cast<uint16_t>(input[1]));
}

uint32_t readU32(const uint8_t* input) {
  return (static_cast<uint32_t>(input[0]) << 24) |
         (static_cast<uint32_t>(input[1]) << 16) |
         (static_cast<uint32_t>(input[2]) << 8) |
         static_cast<uint32_t>(input[3]);
}

// Only packet types understood by this protocol version are accepted. Without
// this check, damaged type bytes could be sent to the wrong decoder.
bool isKnownPacketType(PacketType type) {
  return type == PacketType::Hello || type == PacketType::Telemetry;
}

}  // namespace

const char* protocolErrorName(ProtocolError error) {
  switch (error) {
    case ProtocolError::None:
      return "none";
    case ProtocolError::NullArgument:
      return "null argument";
    case ProtocolError::BufferTooSmall:
      return "buffer too small";
    case ProtocolError::BadMagic:
      return "bad magic";
    case ProtocolError::UnsupportedVersion:
      return "unsupported version";
    case ProtocolError::UnknownPacketType:
      return "unknown packet type";
    case ProtocolError::InvalidSource:
      return "invalid source";
    case ProtocolError::InvalidDestination:
      return "invalid destination";
    case ProtocolError::PayloadTooLarge:
      return "payload too large";
    case ProtocolError::LengthMismatch:
      return "length mismatch";
    case ProtocolError::InvalidPayload:
      return "invalid payload";
    case ProtocolError::InvalidField:
      return "invalid field";
  }
  return "unknown error";
}

ProtocolError encodePacket(const PacketHeader& header,
                           const uint8_t* payload,
                           uint8_t* output,
                           size_t outputCapacity,
                           size_t& outputLength) {
  // A failed encoder must never leave the caller believing bytes are ready to
  // transmit, so begin with a reported length of zero.
  outputLength = 0;

  // A payload pointer may be null only when its stated length is zero.
  if (output == nullptr || (header.payloadLength > 0 && payload == nullptr)) {
    return ProtocolError::NullArgument;
  }
  if (header.protocolVersion != kProtocolVersion) {
    return ProtocolError::UnsupportedVersion;
  }
  if (!isKnownPacketType(header.packetType)) {
    return ProtocolError::UnknownPacketType;
  }
  if (!isValidNodeId(header.sourceNodeId)) {
    return ProtocolError::InvalidSource;
  }
  if (!isValidDestination(header.destinationNodeId)) {
    return ProtocolError::InvalidDestination;
  }
  if (header.payloadLength > kMaxPayloadSize) {
    return ProtocolError::PayloadTooLarge;
  }

  const size_t requiredLength = kPacketHeaderSize + header.payloadLength;
  if (outputCapacity < requiredLength) {
    return ProtocolError::BufferTooSmall;
  }

  // Header byte layout. These exact offsets are the protocol and must match the
  // offsets used by decodePacket below.
  output[0] = kPacketMagic0;
  output[1] = kPacketMagic1;
  output[2] = header.protocolVersion;
  output[3] = static_cast<uint8_t>(header.packetType);
  output[4] = header.sourceNodeId;
  output[5] = header.destinationNodeId;
  writeU16(output + 6, header.sequenceNumber);
  writeU32(output + 8, header.uptimeMs);
  output[12] = header.payloadLength;

  // Copy only the validated payload length; no dynamic allocation is needed.
  for (size_t i = 0; i < header.payloadLength; ++i) {
    output[kPacketHeaderSize + i] = payload[i];
  }
  outputLength = requiredLength;
  return ProtocolError::None;
}

ProtocolError decodePacket(const uint8_t* packet,
                           size_t packetLength,
                           DecodedPacket& decoded) {
  // Validate the outer envelope before allowing another module to inspect any
  // payload fields. This protects against truncated or unrelated radio data.
  if (packet == nullptr) {
    return ProtocolError::NullArgument;
  }
  if (packetLength < kPacketHeaderSize) {
    return ProtocolError::BufferTooSmall;
  }
  if (packet[0] != kPacketMagic0 || packet[1] != kPacketMagic1) {
    return ProtocolError::BadMagic;
  }

  // Read the same fixed byte offsets written by encodePacket.
  decoded.header.protocolVersion = packet[2];
  decoded.header.packetType = static_cast<PacketType>(packet[3]);
  decoded.header.sourceNodeId = packet[4];
  decoded.header.destinationNodeId = packet[5];
  decoded.header.sequenceNumber = readU16(packet + 6);
  decoded.header.uptimeMs = readU32(packet + 8);
  decoded.header.payloadLength = packet[12];
  decoded.payload = packet + kPacketHeaderSize;

  if (decoded.header.protocolVersion != kProtocolVersion) {
    return ProtocolError::UnsupportedVersion;
  }
  if (!isKnownPacketType(decoded.header.packetType)) {
    return ProtocolError::UnknownPacketType;
  }
  if (!isValidNodeId(decoded.header.sourceNodeId)) {
    return ProtocolError::InvalidSource;
  }
  if (!isValidDestination(decoded.header.destinationNodeId)) {
    return ProtocolError::InvalidDestination;
  }
  if (decoded.header.payloadLength > kMaxPayloadSize) {
    return ProtocolError::PayloadTooLarge;
  }
  // Exact equality rejects both truncated data and unexplained trailing bytes.
  if (packetLength != kPacketHeaderSize + decoded.header.payloadLength) {
    return ProtocolError::LengthMismatch;
  }
  return ProtocolError::None;
}

ProtocolError encodeHelloPayload(const HelloPayload& hello,
                                 uint8_t* output,
                                 size_t outputCapacity) {
  // HELLO layout (9 bytes): ID (1), role (1), version (1), uptime (4),
  // hops to ground (1), next hop (1).
  if (output == nullptr) {
    return ProtocolError::NullArgument;
  }
  if (outputCapacity < kHelloPayloadSize) {
    return ProtocolError::BufferTooSmall;
  }
  if (!isValidNodeId(hello.nodeId) || !isValidNodeRole(hello.role) ||
      hello.protocolVersion != kProtocolVersion) {
    return ProtocolError::InvalidField;
  }
  output[0] = hello.nodeId;
  output[1] = static_cast<uint8_t>(hello.role);
  output[2] = hello.protocolVersion;
  writeU32(output + 3, hello.uptimeMs);
  output[7] = hello.hopsToGround;
  output[8] = hello.nextHop;
  return ProtocolError::None;
}

ProtocolError decodeHelloPayload(const uint8_t* payload,
                                 size_t payloadLength,
                                 HelloPayload& hello) {
  if (payload == nullptr) {
    return ProtocolError::NullArgument;
  }
  if (payloadLength != kHelloPayloadSize) {
    return ProtocolError::LengthMismatch;
  }
  // Explicit field reads avoid compiler-dependent struct padding.
  hello.nodeId = payload[0];
  hello.role = static_cast<NodeRole>(payload[1]);
  hello.protocolVersion = payload[2];
  hello.uptimeMs = readU32(payload + 3);
  hello.hopsToGround = payload[7];
  hello.nextHop = payload[8];
  if (!isValidNodeId(hello.nodeId) || !isValidNodeRole(hello.role) ||
      hello.protocolVersion != kProtocolVersion) {
    return ProtocolError::InvalidField;
  }
  return ProtocolError::None;
}

bool isValidTelemetry(const TelemetryData& telemetry) {
  // 0xFF is the standard local "unknown" battery percentage marker. Otherwise
  // a percentage must be in the human-readable 0-100 range.
  const bool batteryValid = telemetry.batteryPercent <= 100 ||
                            telemetry.batteryPercent == 0xFF;
  // Latitude and longitude ranges are checked in their E7 scaled units.
  return isValidNodeId(telemetry.originatingNodeId) &&
         telemetry.latitudeE7 >= -900000000 &&
         telemetry.latitudeE7 <= 900000000 &&
         telemetry.longitudeE7 >= -1800000000 &&
         telemetry.longitudeE7 <= 1800000000 && batteryValid &&
         telemetry.headingCentidegrees < 36000 &&
         static_cast<uint8_t>(telemetry.gpsFixStatus) <=
             static_cast<uint8_t>(GpsFixStatus::Ppp) &&
         static_cast<uint8_t>(telemetry.flightMode) <=
             static_cast<uint8_t>(FlightMode::Land) &&
         static_cast<uint8_t>(telemetry.systemHealth) <=
             static_cast<uint8_t>(SystemHealth::Error) &&
         (telemetry.validityFlags & ~kKnownTelemetryValidityMask) == 0;
}

ProtocolError encodeTelemetryPayload(const TelemetryData& telemetry,
                                     uint8_t* output,
                                     size_t outputCapacity) {
  if (output == nullptr) {
    return ProtocolError::NullArgument;
  }
  if (outputCapacity < kTelemetryPayloadSize) {
    return ProtocolError::BufferTooSmall;
  }
  if (!isValidTelemetry(telemetry)) {
    return ProtocolError::InvalidField;
  }

  // Telemetry byte layout (32 bytes total). Heading is stored in hundredths of
  // a degree, and GPS fix uses the compact status defined in MeshTypes.h:
  //   origin ID       1 byte
  //   sequence        2 bytes
  //   timestamp       4 bytes
  //   latitude E7     4 bytes
  //   longitude E7    4 bytes
  //   altitude cm     4 bytes
  //   battery %       1 byte
  //   battery mV      2 bytes
  //   speed cm/s      2 bytes
  //   mode/armed/health 1 byte each
  //   validity flags  2 bytes
  output[0] = telemetry.originatingNodeId;
  writeU16(output + 1, telemetry.sequenceNumber);
  writeU32(output + 3, telemetry.timestampMs);
  writeU32(output + 7, static_cast<uint32_t>(telemetry.latitudeE7));
  writeU32(output + 11, static_cast<uint32_t>(telemetry.longitudeE7));
  writeU32(output + 15, static_cast<uint32_t>(telemetry.altitudeCm));
  output[19] = telemetry.batteryPercent;
  writeU16(output + 20, telemetry.batteryMillivolts);
  writeU16(output + 22, telemetry.groundSpeedCms);
  writeU16(output + 24, telemetry.headingCentidegrees);
  output[26] = static_cast<uint8_t>(telemetry.gpsFixStatus);
  output[27] = static_cast<uint8_t>(telemetry.flightMode);
  output[28] = telemetry.armed ? 1 : 0;
  output[29] = static_cast<uint8_t>(telemetry.systemHealth);
  writeU16(output + 30, telemetry.validityFlags);
  return ProtocolError::None;
}

ProtocolError decodeTelemetryPayload(const uint8_t* payload,
                                     size_t payloadLength,
                                     TelemetryData& telemetry) {
  if (payload == nullptr) {
    return ProtocolError::NullArgument;
  }
  if (payloadLength != kTelemetryPayloadSize) {
    return ProtocolError::LengthMismatch;
  }

  // Reconstruct signed coordinates and altitude from the same four raw bytes
  // used by the encoder. static_cast preserves their two's-complement bit pattern.
  telemetry.originatingNodeId = payload[0];
  telemetry.sequenceNumber = readU16(payload + 1);
  telemetry.timestampMs = readU32(payload + 3);
  telemetry.latitudeE7 = static_cast<int32_t>(readU32(payload + 7));
  telemetry.longitudeE7 = static_cast<int32_t>(readU32(payload + 11));
  telemetry.altitudeCm = static_cast<int32_t>(readU32(payload + 15));
  telemetry.batteryPercent = payload[19];
  telemetry.batteryMillivolts = readU16(payload + 20);
  telemetry.groundSpeedCms = readU16(payload + 22);
  telemetry.headingCentidegrees = readU16(payload + 24);
  telemetry.gpsFixStatus = static_cast<GpsFixStatus>(payload[26]);
  telemetry.flightMode = static_cast<FlightMode>(payload[27]);
  telemetry.armed = payload[28] != 0;
  telemetry.systemHealth = static_cast<SystemHealth>(payload[29]);
  telemetry.validityFlags = readU16(payload + 30);

  // The armed byte is deliberately restricted to 0 or 1. Values such as 42
  // most likely indicate packet corruption or a protocol mismatch.
  if (payload[28] > 1 || !isValidTelemetry(telemetry)) {
    return ProtocolError::InvalidField;
  }
  return ProtocolError::None;
}

}  // namespace AndyMesh
