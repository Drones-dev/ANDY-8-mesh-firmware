// Mesh command tests: Command/CommandAck serialization and validation,
// command duplicate filtering, and flight-controller retry behavior.

#include <assert.h>
#include <stdio.h>

#include "CommandExecutor.h"
#include "PacketProtocol.h"
#include "Router.h"

using namespace AndyMesh;

namespace {

void testCommandRoundTrip() {
  const CommandPayload original{0, 2, MeshCommand::Land, 0xBEEF, 5};
  uint8_t payload[kCommandPayloadSize];
  assert(encodeCommandPayload(original, payload, sizeof(payload)) ==
         ProtocolError::None);
  assert(payload[3] == 0xBE && payload[4] == 0xEF);

  const PacketHeader header{kProtocolVersion, PacketType::Command, 0,
                            kBroadcastNodeId, 7, 1000,
                            static_cast<uint8_t>(sizeof(payload))};
  uint8_t packet[kMaxPacketSize];
  size_t length = 0;
  assert(encodePacket(header, payload, packet, sizeof(packet), length) ==
         ProtocolError::None);
  DecodedPacket decoded;
  assert(decodePacket(packet, length, decoded) == ProtocolError::None);
  assert(decoded.header.packetType == PacketType::Command);

  CommandPayload result;
  assert(decodeCommandPayload(decoded.payload, decoded.header.payloadLength,
                              result) == ProtocolError::None);
  assert(result.originNodeId == 0);
  assert(result.targetNodeId == 2);
  assert(result.command == MeshCommand::Land);
  assert(result.commandSequence == 0xBEEF);
  assert(result.hopsRemaining == 5);
}

void testBroadcastTargetAllowed() {
  const CommandPayload all{0, kBroadcastNodeId, MeshCommand::Land, 1, 5};
  uint8_t payload[kCommandPayloadSize];
  assert(encodeCommandPayload(all, payload, sizeof(payload)) ==
         ProtocolError::None);
  CommandPayload result;
  assert(decodeCommandPayload(payload, sizeof(payload), result) ==
         ProtocolError::None);
  assert(result.targetNodeId == kBroadcastNodeId);
}

void testInvalidCommandsRejected() {
  uint8_t payload[kCommandPayloadSize];
  const CommandPayload unknown{0, 2, static_cast<MeshCommand>(9), 1, 5};
  assert(encodeCommandPayload(unknown, payload, sizeof(payload)) ==
         ProtocolError::InvalidField);
  const CommandPayload noHops{0, 2, MeshCommand::Land, 1, 0};
  assert(encodeCommandPayload(noHops, payload, sizeof(payload)) ==
         ProtocolError::InvalidField);
  const CommandPayload badTarget{0, 200, MeshCommand::Land, 1, 5};
  assert(encodeCommandPayload(badTarget, payload, sizeof(payload)) ==
         ProtocolError::InvalidField);

  const CommandPayload good{0, 2, MeshCommand::Land, 1, 5};
  assert(encodeCommandPayload(good, payload, sizeof(payload)) ==
         ProtocolError::None);
  CommandPayload result;
  assert(decodeCommandPayload(payload, sizeof(payload) - 1, result) ==
         ProtocolError::LengthMismatch);
  payload[2] = 0x7F;
  assert(decodeCommandPayload(payload, sizeof(payload), result) ==
         ProtocolError::InvalidField);
}

void testCommandAckRoundTrip() {
  const CommandAckPayload original{3, 0x1234, MeshCommand::Land,
                                   kCommandResultAccepted};
  uint8_t payload[kCommandAckPayloadSize];
  assert(encodeCommandAckPayload(original, payload, sizeof(payload)) ==
         ProtocolError::None);
  CommandAckPayload result;
  assert(decodeCommandAckPayload(payload, sizeof(payload), result) ==
         ProtocolError::None);
  assert(result.droneNodeId == 3);
  assert(result.commandSequence == 0x1234);
  assert(result.command == MeshCommand::Land);
  assert(result.result == kCommandResultAccepted);
  assert(decodeCommandAckPayload(payload, 4, result) ==
         ProtocolError::LengthMismatch);
}

void testCommandDuplicateFilter() {
  Router router;
  router.begin(2, false);
  assert(!router.commandAlreadySeen(PacketType::Command, 0, 40));
  assert(router.commandAlreadySeen(PacketType::Command, 0, 40));
  // Same numbers under the other packet type are a different message.
  assert(!router.commandAlreadySeen(PacketType::CommandAck, 0, 40));
  assert(!router.commandAlreadySeen(PacketType::Command, 0, 41));
  // Telemetry traffic does not evict remembered commands.
  for (uint16_t i = 0; i < 100; ++i) router.alreadySeen(1, i, i * 2000U);
  assert(router.commandAlreadySeen(PacketType::Command, 0, 40));
}

void testExecutorAcceptedOnFirstTry() {
  CommandExecutor executor;
  executor.begin(2);
  executor.start(MeshCommand::Land, 99, 1000);
  uint8_t attempt = 0xFF;
  assert(executor.attemptDue(1000, attempt) && attempt == 0);
  assert(!executor.attemptDue(1100, attempt));
  executor.finish(kCommandResultAccepted);
  assert(!executor.active());
  CommandAckPayload report;
  assert(executor.takeReport(report));
  assert(report.droneNodeId == 2 && report.commandSequence == 99);
  assert(report.result == kCommandResultAccepted);
  assert(!executor.takeReport(report));
}

void testExecutorRetriesThenGivesUp() {
  CommandExecutor executor;
  executor.begin(1);
  executor.start(MeshCommand::Land, 5, 0);
  uint8_t attempt = 0;
  uint32_t now = 0;
  for (uint8_t i = 0; i < CommandExecutor::kMaxAttempts; ++i) {
    assert(executor.attemptDue(now, attempt) && attempt == i);
    now += CommandExecutor::kRetryIntervalMs;
  }
  assert(!executor.attemptDue(now, attempt));
  CommandAckPayload report;
  assert(executor.takeReport(report));
  assert(report.result == kCommandResultNoResponse);
}

void testNewerCommandReplacesOld() {
  CommandExecutor executor;
  executor.begin(3);
  executor.start(MeshCommand::Land, 10, 0);
  uint8_t attempt = 0;
  assert(executor.attemptDue(0, attempt));
  executor.start(MeshCommand::Land, 11, 100);
  assert(executor.attemptDue(100, attempt) && attempt == 0);
  executor.finish(kCommandResultAccepted);
  CommandAckPayload report;
  assert(executor.takeReport(report) && report.commandSequence == 11);
}

}  // namespace

int main() {
  testCommandRoundTrip();
  testBroadcastTargetAllowed();
  testInvalidCommandsRejected();
  testCommandAckRoundTrip();
  testCommandDuplicateFilter();
  testExecutorAcceptedOnFirstTry();
  testExecutorRetriesThenGivesUp();
  testNewerCommandReplacesOld();
  puts("command_tests: PASS");
  return 0;
}
