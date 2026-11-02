#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "PacketProtocol.h"

using namespace AndyMesh;

namespace {

void testHelloRoundTrip() {
  const HelloPayload original{2, NodeRole::Relay, kProtocolVersion, 0x12345678};
  uint8_t payload[kHelloPayloadSize];
  assert(encodeHelloPayload(original, payload, sizeof(payload)) ==
         ProtocolError::None);

  const PacketHeader header{kProtocolVersion, PacketType::Hello, 2,
                            kBroadcastNodeId, 0x3456, 0x12345678,
                            static_cast<uint8_t>(sizeof(payload))};
  uint8_t packet[kMaxPacketSize];
  size_t packetLength = 0;
  assert(encodePacket(header, payload, packet, sizeof(packet), packetLength) ==
         ProtocolError::None);
  assert(packetLength == kPacketHeaderSize + kHelloPayloadSize);
  assert(packet[6] == 0x34 && packet[7] == 0x56);
  assert(packet[8] == 0x12 && packet[9] == 0x34 && packet[10] == 0x56 &&
         packet[11] == 0x78);

  DecodedPacket decoded;
  assert(decodePacket(packet, packetLength, decoded) == ProtocolError::None);
  assert(decoded.header.sourceNodeId == 2);
  assert(decoded.header.destinationNodeId == kBroadcastNodeId);
  assert(decoded.header.sequenceNumber == 0x3456);

  HelloPayload result;
  assert(decodeHelloPayload(decoded.payload, decoded.header.payloadLength,
                            result) == ProtocolError::None);
  assert(result.nodeId == original.nodeId);
  assert(result.role == original.role);
  assert(result.uptimeMs == original.uptimeMs);
}

void testTelemetryRoundTrip() {
  const TelemetryData original{3,
                               104,
                               250000,
                               340231000,
                               -845812000,
                               4520,
                               87,
                               15250,
                               340,
                               12345,
                               GpsFixStatus::Fix3D,
                               FlightMode::Auto,
                               true,
                               SystemHealth::Ok,
                               kKnownTelemetryValidityMask};
  uint8_t payload[kTelemetryPayloadSize];
  assert(encodeTelemetryPayload(original, payload, sizeof(payload)) ==
         ProtocolError::None);

  TelemetryData result;
  assert(decodeTelemetryPayload(payload, sizeof(payload), result) ==
         ProtocolError::None);
  assert(result.originatingNodeId == original.originatingNodeId);
  assert(result.sequenceNumber == original.sequenceNumber);
  assert(result.timestampMs == original.timestampMs);
  assert(result.latitudeE7 == original.latitudeE7);
  assert(result.longitudeE7 == original.longitudeE7);
  assert(result.altitudeCm == original.altitudeCm);
  assert(result.batteryPercent == original.batteryPercent);
  assert(result.batteryMillivolts == original.batteryMillivolts);
  assert(result.groundSpeedCms == original.groundSpeedCms);
  assert(result.headingCentidegrees == original.headingCentidegrees);
  assert(result.gpsFixStatus == original.gpsFixStatus);
  assert(result.flightMode == original.flightMode);
  assert(result.armed == original.armed);
  assert(result.systemHealth == original.systemHealth);
  assert(result.validityFlags == original.validityFlags);
}

void testMalformedPacketsAreRejected() {
  const uint8_t payload[kHelloPayloadSize] = {1, 1, kProtocolVersion,
                                              0, 0, 0, 1};
  const PacketHeader header{kProtocolVersion, PacketType::Hello, 1,
                            kBroadcastNodeId, 1, 1,
                            static_cast<uint8_t>(sizeof(payload))};
  uint8_t packet[kMaxPacketSize];
  size_t packetLength = 0;
  assert(encodePacket(header, payload, packet, sizeof(packet), packetLength) ==
         ProtocolError::None);

  DecodedPacket decoded;
  packet[0] ^= 0x01;
  assert(decodePacket(packet, packetLength, decoded) ==
         ProtocolError::BadMagic);
  packet[0] ^= 0x01;

  packet[2] = kProtocolVersion + 1;
  assert(decodePacket(packet, packetLength, decoded) ==
         ProtocolError::UnsupportedVersion);
  packet[2] = kProtocolVersion;

  assert(decodePacket(packet, packetLength - 1, decoded) ==
         ProtocolError::LengthMismatch);

  packet[4] = kBroadcastNodeId;
  assert(decodePacket(packet, packetLength, decoded) ==
         ProtocolError::InvalidSource);
}

void testInvalidTelemetryIsRejected() {
  TelemetryData telemetry{1,
                          1,
                          1,
                          910000000,
                          0,
                          0,
                          101,
                          0,
                          0,
                          0,
                          GpsFixStatus::NoGps,
                          FlightMode::Unknown,
                          false,
                          SystemHealth::Unknown,
                          0};
  uint8_t payload[kTelemetryPayloadSize];
  assert(encodeTelemetryPayload(telemetry, payload, sizeof(payload)) ==
         ProtocolError::InvalidField);
}

}  // namespace

int main() {
  testHelloRoundTrip();
  testTelemetryRoundTrip();
  testMalformedPacketsAreRejected();
  testInvalidTelemetryIsRejected();
  puts("protocol_tests: PASS");
  return 0;
}

