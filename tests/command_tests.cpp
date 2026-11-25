// Mesh command tests: Command/CommandAck serialization and validation,
// command duplicate filtering, LAND/START/KILL step sequences, and
// flight-controller retry behavior.

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

void testCommandNamesAndValidation() {
  uint8_t payload[kCommandPayloadSize];
  for (uint8_t c = 1; c <= 3; ++c) {
    const CommandPayload command{0, 1, static_cast<MeshCommand>(c), 1, 5};
    assert(encodeCommandPayload(command, payload, sizeof(payload)) ==
           ProtocolError::None);
  }
  const CommandPayload unknown{0, 1, static_cast<MeshCommand>(4), 1, 5};
  assert(encodeCommandPayload(unknown, payload, sizeof(payload)) ==
         ProtocolError::InvalidField);
}

void testStepTables() {
  const FlightControllerStep* steps = nullptr;
  assert(CommandExecutor::stepsFor(MeshCommand::Land, steps) == 1);
  assert(steps[0].command == kMavCmdNavLand);

  // START switches to Mission mode before arming.
  assert(CommandExecutor::stepsFor(MeshCommand::Start, steps) == 2);
  assert(steps[0].command == kMavCmdDoSetMode);
  assert(steps[0].params[0] == 1 && steps[0].params[1] == 4 &&
         steps[0].params[2] == 4);
  assert(steps[1].command == kMavCmdComponentArmDisarm);
  assert(steps[1].params[0] == 1);

  // KILL is a forced disarm.
  assert(CommandExecutor::stepsFor(MeshCommand::Kill, steps) == 1);
  assert(steps[0].command == kMavCmdComponentArmDisarm);
  assert(steps[0].params[0] == 0 && steps[0].params[1] == kForceDisarmMagic);
}

void testLandAcceptedOnFirstTry() {
  CommandExecutor executor;
  executor.begin(2);
  executor.start(MeshCommand::Land, 99, 1000);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0xFF;
  assert(executor.attemptDue(1000, step, attempt) && attempt == 0);
  assert(step->command == kMavCmdNavLand);
  assert(!executor.attemptDue(1100, step, attempt));
  executor.onFlightControllerAck(kMavCmdNavLand, kCommandResultAccepted, 1150);
  assert(!executor.active());
  CommandAckPayload report;
  assert(executor.takeReport(report));
  assert(report.droneNodeId == 2 && report.commandSequence == 99);
  assert(report.command == MeshCommand::Land);
  assert(report.result == kCommandResultAccepted);
  assert(!executor.takeReport(report));
}

void testStartRunsBothSteps() {
  CommandExecutor executor;
  executor.begin(1);
  executor.start(MeshCommand::Start, 7, 0);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0;
  assert(executor.attemptDue(0, step, attempt));
  assert(step->command == kMavCmdDoSetMode);
  // An answer for a different command does not advance the sequence.
  executor.onFlightControllerAck(kMavCmdComponentArmDisarm,
                                 kCommandResultAccepted, 10);
  assert(executor.active());
  executor.onFlightControllerAck(kMavCmdDoSetMode, kCommandResultAccepted, 20);
  assert(executor.active());
  assert(executor.attemptDue(20, step, attempt) && attempt == 0);
  assert(step->command == kMavCmdComponentArmDisarm);
  executor.onFlightControllerAck(kMavCmdComponentArmDisarm,
                                 kCommandResultAccepted, 30);
  CommandAckPayload report;
  assert(executor.takeReport(report));
  assert(report.command == MeshCommand::Start);
  assert(report.result == kCommandResultAccepted);
}

void testStartRefusedModeNeverArms() {
  CommandExecutor executor;
  executor.begin(1);
  executor.start(MeshCommand::Start, 8, 0);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0;
  assert(executor.attemptDue(0, step, attempt));
  executor.onFlightControllerAck(kMavCmdDoSetMode, 2 /* DENIED */, 10);
  assert(!executor.active());
  assert(!executor.attemptDue(1000, step, attempt));
  CommandAckPayload report;
  assert(executor.takeReport(report) && report.result == 2);
}

void testInProgressKeepsWaiting() {
  CommandExecutor executor;
  executor.begin(1);
  executor.start(MeshCommand::Land, 9, 0);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0;
  assert(executor.attemptDue(0, step, attempt));
  executor.onFlightControllerAck(kMavCmdNavLand, kMavResultInProgress, 10);
  assert(executor.active());
}

void testExecutorRetriesThenGivesUp() {
  CommandExecutor executor;
  executor.begin(1);
  executor.start(MeshCommand::Kill, 5, 0);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0;
  uint32_t now = 0;
  for (uint8_t i = 0; i < CommandExecutor::kMaxAttempts; ++i) {
    assert(executor.attemptDue(now, step, attempt) && attempt == i);
    now += CommandExecutor::kRetryIntervalMs;
  }
  assert(!executor.attemptDue(now, step, attempt));
  CommandAckPayload report;
  assert(executor.takeReport(report));
  assert(report.result == kCommandResultNoResponse);
}

void testNewerCommandReplacesOld() {
  CommandExecutor executor;
  executor.begin(3);
  executor.start(MeshCommand::Start, 10, 0);
  const FlightControllerStep* step = nullptr;
  uint8_t attempt = 0;
  assert(executor.attemptDue(0, step, attempt));
  executor.start(MeshCommand::Land, 11, 100);
  assert(executor.attemptDue(100, step, attempt) && attempt == 0);
  assert(step->command == kMavCmdNavLand);
  executor.onFlightControllerAck(kMavCmdNavLand, kCommandResultAccepted, 150);
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
  testCommandNamesAndValidation();
  testStepTables();
  testLandAcceptedOnFirstTry();
  testStartRunsBothSteps();
  testStartRefusedModeNeverArms();
  testInProgressKeepsWaiting();
  testExecutorRetriesThenGivesUp();
  testNewerCommandReplacesOld();
  puts("command_tests: PASS");
  return 0;
}
