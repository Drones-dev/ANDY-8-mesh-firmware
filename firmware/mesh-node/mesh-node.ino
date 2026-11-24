/*
  ANDY 8: RED - Phase 2 mesh node firmware

  Board:   Seeed XIAO ESP32S3 + Wio-SX1262
  Radio:   915 MHz, 125 kHz, SF7, CR 4/7, 12 dBm
  Library: RadioLib

  Copy NodeBuildConfig.example.h to NodeBuildConfig.h and select this board's
  identity before uploading. The same core firmware runs on every node.

  Main-loop responsibilities:
    1. Read completed LoRa packets without blocking.
    2. Broadcast periodic HELLO discovery/heartbeat packets.
    3. Update direct neighbors and detect heartbeat timeouts.
    4. Generate simulated telemetry or ingest MAVLink UART data.
    5. Select the next hop toward the Ground Station (Router).
    6. Send telemetry to the next hop and relay telemetry addressed to this
       node from other drones.
    7. Ground Station: read "LAND <id>" / "LAND ALL" typed in the Serial
       Monitor (line ending: Newline) and flood the command to the drones.
    8. Drones: relay flooded commands, pass LAND to the flight controller over
       MAVLink, and route the flight controller's answer back to the ground.
*/

#include <Arduino.h>  // millis(), delay(), Serial, and basic Arduino types.
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// Each project header owns one focused responsibility. This keeps the main
// sketch readable and prevents radio, protocol, and telemetry details from being
// mixed into one very large loop.
#include "CommandExecutor.h"
#include "HelloManager.h"
#include "MavlinkTelemetryProvider.h"
#include "NeighborTable.h"
#include "NodeConfig.h"
#include "PacketProtocol.h"
#include "RadioManager.h"
#include "Router.h"
#include "SimulatedTelemetryProvider.h"
#include "TelemetryManager.h"

// These types belong to the AndyMesh namespace. The using statement keeps the
// sketch readable (NodeConfig instead of AndyMesh::NodeConfig on every line).
using namespace AndyMesh;

// loadNodeConfig reads the centralized defaults plus this board's optional local
// NodeBuildConfig.h. Every subsystem below uses this same identity and timing.
NodeConfig nodeConfig = loadNodeConfig();

// One object controls each long-lived mesh subsystem. Global lifetime is useful
// on a microcontroller because these objects exist for the entire power cycle
// and do not repeatedly allocate memory inside loop().
RadioManager radioManager;
HelloManager helloManager;
NeighborTable neighborTable;
Router router;  // Next-hop selection and duplicate filtering.

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
// ESP32 hardware UART number 1 receives bytes from the flight controller. The
// actual GPIO pins and baud rate are still required in NodeBuildConfig.h.
HardwareSerial flightControllerSerial(1);
MavlinkTelemetryProvider telemetryProvider(flightControllerSerial);
#else
// Simulation is the default Phase A provider and requires no flight controller.
SimulatedTelemetryProvider telemetryProvider;
#endif

// TelemetryManager talks only to the common provider interface, so all packet
// scheduling below is identical for simulated and real flight data.
TelemetryManager telemetryManager(telemetryProvider);

// packetSequenceNumber counts every outgoing protocol packet (both HELLO and
// telemetry). nextNeighborTablePrintMs limits full-table Serial output to the
// configured interval without delaying other work.
uint16_t packetSequenceNumber = 0;
uint32_t nextNeighborTablePrintMs = 0;

// Drone side: carries one mesh command through to the flight controller.
CommandExecutor commandExecutor;

// Ground Station side. The command sequence starts at a random value so drones
// that still remember commands from before a Ground Station reboot do not
// discard new commands as duplicates.
uint16_t groundCommandSequence = 0;
char consoleLine[32];
size_t consoleLength = 0;

// The Ground Station sends each command this many times; drones run it once.
const uint8_t kCommandRepeats = 3;
const uint32_t kCommandRepeatIntervalMs = 700;
// Relayed commands wait a short random time so two relays that heard the same
// copy do not transmit over each other.
const uint32_t kRelayJitterMinMs = 30;
const uint32_t kRelayJitterMaxMs = 250;
// Every hop sends relayed commands and replies twice. Each node forwards only
// the first copy it hears, so without this one lost frame would end the flood.
const uint8_t kHopRepeats = 2;
const uint32_t kHopRepeatIntervalMs = 400;
// A queued reply with no route to ground is dropped after this long.
const uint32_t kOutboundExpiryMs = 10000;

// Command and CommandAck frames wait here until the radio is free, so a busy
// radio delays them instead of dropping them.
struct OutboundFrame {
  bool used;
  PacketType type;
  bool viaNextHop;  // Destination is chosen at send time from the router.
  NodeId destination;
  uint8_t payload[kMaxPayloadSize];
  uint8_t payloadLength;
  uint32_t notBeforeMs;
  uint32_t expiresMs;
};
const size_t kOutboundSlots = 10;
OutboundFrame outbound[kOutboundSlots];

// millis() is a 32-bit counter and eventually wraps to zero. Testing a signed
// subtraction keeps deadline checks correct across that wraparound.
bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}

void stopWithMessage(const char* message) {
  // Configuration/radio startup failures make networking impossible. Print the
  // reason once, then remain stopped instead of running partially initialized
  // code that could produce misleading results.
  Serial.println(message);
  while (true) {
    delay(1000);
  }
}

bool transmitPayload(PacketType type,
                     NodeId destination,
                     const uint8_t* payload,
                     uint8_t payloadLength,
                     uint32_t nowMs) {
  // Allocate the maximum packet buffer on this short-lived function's stack.
  // encodePacket reports the actual smaller length to transmit.
  uint8_t packet[kMaxPacketSize];
  size_t packetLength = 0;

  // The common header wraps either payload type. ++packetSequenceNumber gives
  // each outgoing frame a new order number; uptime uses this node's millis().
  const PacketHeader header{kProtocolVersion,
                            type,
                            nodeConfig.nodeId,
                            destination,
                            ++packetSequenceNumber,
                            nowMs,
                            payloadLength};
  const ProtocolError error = encodePacket(
      header, payload, packet, sizeof(packet), packetLength);
  if (error != ProtocolError::None) {
    Serial.print("[PROTOCOL] Could not encode packet: ");
    Serial.println(protocolErrorName(error));
    return false;
  }
  // RadioManager starts asynchronous transmission and returns immediately.
  return radioManager.send(packet, packetLength);
}

bool queueFrame(PacketType type, bool viaNextHop, NodeId destination,
                const uint8_t* payload, uint8_t payloadLength,
                uint32_t notBeforeMs) {
  for (size_t i = 0; i < kOutboundSlots; ++i) {
    OutboundFrame& frame = outbound[i];
    if (frame.used) continue;
    frame.used = true;
    frame.type = type;
    frame.viaNextHop = viaNextHop;
    frame.destination = destination;
    memcpy(frame.payload, payload, payloadLength);
    frame.payloadLength = payloadLength;
    frame.notBeforeMs = notBeforeMs;
    frame.expiresMs = notBeforeMs + kOutboundExpiryMs;
    return true;
  }
  Serial.println("[MESH] Outbound queue full; command frame dropped");
  return false;
}

void queueHopRepeats(PacketType type, bool viaNextHop, NodeId destination,
                     const uint8_t* payload, uint8_t payloadLength,
                     uint32_t firstMs) {
  for (uint8_t i = 0; i < kHopRepeats; ++i) {
    queueFrame(type, viaNextHop, destination, payload, payloadLength,
               firstMs + i * kHopRepeatIntervalMs);
  }
}

// Sends at most one due frame. Returns true when the radio was used.
bool serviceOutbound(uint32_t nowMs) {
  for (size_t i = 0; i < kOutboundSlots; ++i) {
    OutboundFrame& frame = outbound[i];
    if (!frame.used || !timeReached(nowMs, frame.notBeforeMs)) continue;
    if (frame.viaNextHop && !router.hasRoute()) {
      if (timeReached(nowMs, frame.expiresMs)) {
        frame.used = false;
        Serial.println("[MESH] Command reply dropped - no route to ground");
      }
      continue;
    }
    const NodeId destination =
        frame.viaNextHop ? router.nextHop() : frame.destination;
    frame.used = false;
    transmitPayload(frame.type, destination, frame.payload,
                    frame.payloadLength, nowMs);
    return true;
  }
  return false;
}

bool sendHello(uint32_t nowMs) {
  // HELLO is first encoded as its seven-byte payload, then wrapped in the common
  // header with destination 255 (broadcast) so every nearby node may receive it.
  HelloPayload hello = helloManager.makePayload(nowMs);
  // Route advertised to neighbors for their own route selection.
  hello.hopsToGround = router.hopsToGround();
  hello.nextHop = router.nextHop();
  uint8_t payload[kHelloPayloadSize];
  const ProtocolError error =
      encodeHelloPayload(hello, payload, sizeof(payload));
  if (error != ProtocolError::None) {
    Serial.print("[PROTOCOL] Could not encode HELLO: ");
    Serial.println(protocolErrorName(error));
    return false;
  }
  if (!transmitPayload(PacketType::Hello, kBroadcastNodeId, payload,
                       sizeof(payload), nowMs)) {
    return false;
  }
  helloManager.markSent(nowMs);
  Serial.print("[Node ");
  Serial.print(nodeConfig.nodeId);
  Serial.println("] HELLO transmitted");
  return true;
}

bool sendTelemetry(uint32_t nowMs) {
  // prepare asks the selected provider for a fresh common data sample and adds
  // the telemetry sequence number. It can return false when real MAVLink has no
  // fresh fields, in which case no misleading packet is transmitted.
  TelemetryData telemetry;
  if (!telemetryManager.prepare(nowMs, telemetry)) {
    return false;
  }

  uint8_t payload[kTelemetryPayloadSize];
  const ProtocolError error =
      encodeTelemetryPayload(telemetry, payload, sizeof(payload));
  if (error != ProtocolError::None) {
    Serial.print("[PROTOCOL] Could not encode telemetry: ");
    Serial.println(protocolErrorName(error));
    return false;
  }
  // No route: sample is skipped until the next interval.
  if (!router.hasRoute()) {
    Serial.println("[MESH] No route to Ground Station - telemetry not sent");
    telemetryManager.markSent(nowMs);
    return false;
  }

  // Own packet recorded so an echoed copy is not relayed.
  router.alreadySeen(telemetry.originatingNodeId, telemetry.sequenceNumber,
                     telemetry.timestampMs);

  // Destination is the next hop: the Ground Station when in direct range,
  // otherwise a relay drone.
  if (!transmitPayload(PacketType::Telemetry, router.nextHop(),
                       payload, sizeof(payload), nowMs)) {
    return false;
  }
  telemetryManager.markSent(nowMs);
  Serial.print("[Node ");
  Serial.print(nodeConfig.nodeId);
  Serial.print("] TELEMETRY transmitted to Node ");
  Serial.print(router.nextHop());
  Serial.print(", sequence ");
  Serial.println(telemetry.sequenceNumber);
  return true;
}

void printTelemetry(const TelemetryData& telemetry) {
  // Every scaled integer is converted to a student-friendly engineering unit
  // only for display. The compact value itself remains unchanged on the radio.
  Serial.println("[GROUND] TELEMETRY RECEIVED");
  Serial.print("Drone ID: ");
  Serial.println(telemetry.originatingNodeId);

  Serial.print("Latitude: ");
  // Check validity before displaying each group. A MAVLink packet may contain
  // current GPS but stale battery data, and "N/A" is safer than an old value.
  if ((telemetry.validityFlags & TelemetryValidPosition) != 0) {
    // Divide E7 degrees by ten million to recover ordinary decimal degrees.
    Serial.println(static_cast<double>(telemetry.latitudeE7) / 10000000.0, 7);
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Longitude: ");
  if ((telemetry.validityFlags & TelemetryValidPosition) != 0) {
    Serial.println(static_cast<double>(telemetry.longitudeE7) / 10000000.0,
                   7);
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Altitude: ");
  if ((telemetry.validityFlags & TelemetryValidAltitude) != 0) {
    // 100 centimeters = 1 meter.
    Serial.print(static_cast<float>(telemetry.altitudeCm) / 100.0F, 2);
    Serial.println(" m");
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Battery: ");
  if ((telemetry.validityFlags & TelemetryValidBattery) != 0) {
    Serial.print(telemetry.batteryPercent);
    Serial.print("%, ");
    // 1000 millivolts = 1 volt.
    Serial.print(static_cast<float>(telemetry.batteryMillivolts) / 1000.0F, 2);
    Serial.println(" V");
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Speed: ");
  if ((telemetry.validityFlags & TelemetryValidGroundSpeed) != 0) {
    // 100 centimeters per second = 1 meter per second.
    Serial.print(static_cast<float>(telemetry.groundSpeedCms) / 100.0F, 2);
    Serial.println(" m/s");
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Heading: ");
  if ((telemetry.validityFlags & TelemetryValidHeading) != 0) {
    Serial.print(static_cast<float>(telemetry.headingCentidegrees) / 100.0F, 2);
    Serial.println(" degrees");
  } else Serial.println("N/A (stale or unavailable)");
  Serial.print("GPS fix: ");
  Serial.println((telemetry.validityFlags & TelemetryValidGpsFix) != 0
                     ? gpsFixName(telemetry.gpsFixStatus)
                     : "N/A (stale or unavailable)");
  Serial.print("Mode: ");
  Serial.println((telemetry.validityFlags & TelemetryValidFlightMode) != 0
                     ? flightModeName(telemetry.flightMode)
                     : "N/A (stale or unavailable)");
  Serial.print("Armed: ");
  if ((telemetry.validityFlags & TelemetryValidArmedState) != 0) {
    Serial.println(telemetry.armed ? "YES" : "NO");
  } else {
    Serial.println("N/A (stale or unavailable)");
  }
  Serial.print("Health: ");
  Serial.println((telemetry.validityFlags & TelemetryValidSystemHealth) != 0
                     ? systemHealthName(telemetry.systemHealth)
                     : "N/A (stale or unavailable)");
  Serial.print("Sequence: ");
  Serial.println(telemetry.sequenceNumber);
}

void handleHello(const DecodedPacket& packet,
                 const TransportPacket& radioPacket,
                 uint32_t nowMs) {
  // The outer packet is already valid. Now validate the HELLO-specific seven
  // bytes and require its repeated node ID to match the header source ID. The
  // duplicate identity check makes corrupted or inconsistent packets harmless.
  HelloPayload hello;
  const ProtocolError error = decodeHelloPayload(
      packet.payload, packet.header.payloadLength, hello);
  if (error != ProtocolError::None ||
      hello.nodeId != packet.header.sourceNodeId) {
    Serial.println("[PROTOCOL] Invalid HELLO ignored");
    return;
  }
  // Only an actually received HELLO reaches this call, so RSSI/SNR describe a
  // direct neighbor rather than a node learned through another relay.
  const NeighborEvent event = neighborTable.observeHello(
      hello, nowMs, radioPacket.rssiDbm, radioPacket.snrDb);
  Serial.print("[Node "); Serial.print(nodeConfig.nodeId);
  Serial.print("] HELLO from Node "); Serial.print(hello.nodeId);
  Serial.print(" RSSI "); Serial.print(radioPacket.rssiDbm, 1);
  Serial.print(" dBm, SNR "); Serial.print(radioPacket.snrDb, 1); Serial.println(" dB");
  if (event == NeighborEvent::Discovered) Serial.println("[MESH] New direct neighbor discovered");
  if (event == NeighborEvent::Recovered) Serial.println("[MESH] Neighbor is ACTIVE again");
  if (event == NeighborEvent::TableFull) Serial.println("[MESH] Neighbor table full");
}

void printNeighborTable() {
  Serial.println("ID | Role | RSSI | SNR | Status | HELLOs");
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    const NeighborEntry* entry = neighborTable.entryAt(i);
    if (entry == nullptr) continue;
    Serial.print(entry->nodeId); Serial.print(" | "); Serial.print(nodeRoleName(entry->role));
    Serial.print(" | "); Serial.print(entry->rssiDbm, 1); Serial.print(" | ");
    Serial.print(entry->snrDb, 1); Serial.print(" | ");
    Serial.print(entry->status == NeighborStatus::Active ? "ACTIVE" : "LOST");
    Serial.print(" | "); Serial.println(entry->helloPacketsReceived);
  }
}

void handleTelemetry(const DecodedPacket& packet, uint32_t nowMs) {
  // Telemetry addressed to this node: the Ground Station displays it, any
  // other node relays it to its own next hop.
  TelemetryData telemetry;
  const ProtocolError error = decodeTelemetryPayload(
      packet.payload, packet.header.payloadLength, telemetry);
  if (error != ProtocolError::None) {
    Serial.println("[PROTOCOL] Invalid telemetry ignored");
    return;
  }
  // The header source is the last relay; the originating drone is
  // telemetry.originatingNodeId.
  if (router.alreadySeen(telemetry.originatingNodeId, telemetry.sequenceNumber,
                         telemetry.timestampMs)) {
    return;
  }

  if (nodeConfig.role == NodeRole::GroundStation) {
    // Last-hop neighbor is shown to confirm the path taken.
    Serial.print("[GROUND] Received via Node ");
    Serial.println(packet.header.sourceNodeId);
    printTelemetry(telemetry);
    return;
  }

  // Relay: payload forwarded unchanged.
  if (!router.hasRoute()) {
    Serial.print("[MESH] Dropped Drone ");
    Serial.print(telemetry.originatingNodeId);
    Serial.println(" telemetry - no route");
    return;
  }
  Serial.print("[MESH] RELAY Drone ");
  Serial.print(telemetry.originatingNodeId);
  Serial.print(" telemetry -> Node ");
  Serial.println(router.nextHop());
  // Dropped if the radio is busy; the origin sends a new sample every 2 s.
  transmitPayload(PacketType::Telemetry, router.nextHop(), packet.payload,
                  packet.header.payloadLength, nowMs);
}

void printCommandTarget(NodeId target) {
  if (target == kBroadcastNodeId) {
    Serial.print("ALL drones");
  } else {
    Serial.print("Drone ");
    Serial.print(target);
  }
}

void handleCommand(const DecodedPacket& packet, uint32_t nowMs) {
  CommandPayload command;
  if (decodeCommandPayload(packet.payload, packet.header.payloadLength,
                           command) != ProtocolError::None) {
    Serial.println("[PROTOCOL] Invalid command ignored");
    return;
  }
  // The Ground Station hears its own command echoed back by relays.
  if (nodeConfig.role == NodeRole::GroundStation) return;
  if (router.commandAlreadySeen(PacketType::Command, command.originNodeId,
                                command.commandSequence)) {
    return;
  }

  if (command.targetNodeId == nodeConfig.nodeId ||
      command.targetNodeId == kBroadcastNodeId) {
    Serial.print("[COMMAND] ");
    Serial.print(meshCommandName(command.command));
    Serial.print(" received from Node ");
    Serial.print(command.originNodeId);
    Serial.print(" via Node ");
    Serial.print(packet.header.sourceNodeId);
    Serial.print(" (cmd ");
    Serial.print(command.commandSequence);
    Serial.println(")");
    commandExecutor.start(command.command, command.commandSequence, nowMs);
  }

  // A command for one specific drone stops at that drone; everything else
  // keeps flooding until its hop budget runs out.
  if (command.targetNodeId == nodeConfig.nodeId || command.hopsRemaining <= 1) {
    return;
  }
  command.hopsRemaining--;
  uint8_t payload[kCommandPayloadSize];
  if (encodeCommandPayload(command, payload, sizeof(payload)) !=
      ProtocolError::None) {
    return;
  }
  const uint32_t jitter =
      random(kRelayJitterMinMs, kRelayJitterMaxMs + 1);
  queueHopRepeats(PacketType::Command, false, kBroadcastNodeId, payload,
                  sizeof(payload), nowMs + jitter);
  Serial.print("[MESH] RELAY ");
  Serial.print(meshCommandName(command.command));
  Serial.print(" for ");
  printCommandTarget(command.targetNodeId);
  Serial.println();
}

void handleCommandAck(const DecodedPacket& packet, uint32_t nowMs) {
  CommandAckPayload ack;
  if (decodeCommandAckPayload(packet.payload, packet.header.payloadLength,
                              ack) != ProtocolError::None) {
    Serial.println("[PROTOCOL] Invalid command reply ignored");
    return;
  }
  if (router.commandAlreadySeen(PacketType::CommandAck, ack.droneNodeId,
                                ack.commandSequence)) {
    return;
  }
  if (nodeConfig.role == NodeRole::GroundStation) {
    Serial.print("[GROUND] Drone ");
    Serial.print(ack.droneNodeId);
    Serial.print(' ');
    Serial.print(meshCommandName(ack.command));
    Serial.print(' ');
    Serial.print(commandResultName(ack.result));
    Serial.print(" (cmd ");
    Serial.print(ack.commandSequence);
    Serial.print(", via Node ");
    Serial.print(packet.header.sourceNodeId);
    Serial.println(")");
    return;
  }
  Serial.print("[MESH] RELAY Drone ");
  Serial.print(ack.droneNodeId);
  Serial.println(" command reply");
  queueHopRepeats(PacketType::CommandAck, true, 0, packet.payload,
                  packet.header.payloadLength, nowMs);
}

void serviceCommandExecutor(uint32_t nowMs) {
  uint8_t result = 0;
  if (commandExecutor.active() && telemetryProvider.takeLandAck(result)) {
    commandExecutor.finish(result);
  }
  uint8_t attempt = 0;
  if (commandExecutor.attemptDue(nowMs, attempt)) {
    if (telemetryProvider.sendLandCommand(attempt, nowMs)) {
      Serial.print("[COMMAND] LAND sent to flight controller (try ");
      Serial.print(attempt + 1);
      Serial.println(")");
    } else {
      commandExecutor.finish(kCommandResultNoAutopilot);
    }
  }
  CommandAckPayload report;
  if (!commandExecutor.takeReport(report)) return;
  Serial.print("[COMMAND] ");
  Serial.print(meshCommandName(report.command));
  Serial.print(" result: ");
  Serial.println(commandResultName(report.result));
  // The executing drone marks its own reply as seen so a relay echoing it
  // back is not forwarded a second time.
  router.commandAlreadySeen(PacketType::CommandAck, report.droneNodeId,
                            report.commandSequence);
  uint8_t payload[kCommandAckPayloadSize];
  if (encodeCommandAckPayload(report, payload, sizeof(payload)) ==
      ProtocolError::None) {
    queueHopRepeats(PacketType::CommandAck, true, 0, payload, sizeof(payload),
                    nowMs);
  }
}

void printConsoleHelp() {
  Serial.println("[GROUND] Commands: LAND <drone id>   LAND ALL");
}

void sendGroundCommand(MeshCommand type, NodeId target, uint32_t nowMs) {
  const CommandPayload command{nodeConfig.nodeId, target, type,
                               ++groundCommandSequence, kMaxHops};
  uint8_t payload[kCommandPayloadSize];
  if (encodeCommandPayload(command, payload, sizeof(payload)) !=
      ProtocolError::None) {
    Serial.println("[GROUND] Could not encode command");
    return;
  }
  for (uint8_t i = 0; i < kCommandRepeats; ++i) {
    queueFrame(PacketType::Command, false, kBroadcastNodeId, payload,
               sizeof(payload), nowMs + i * kCommandRepeatIntervalMs);
  }
  Serial.print("[GROUND] ");
  Serial.print(meshCommandName(type));
  Serial.print(" -> ");
  printCommandTarget(target);
  Serial.print(" (cmd ");
  Serial.print(command.commandSequence);
  Serial.println(")");
}

void runConsoleLine(char* line, uint32_t nowMs) {
  char* word = strtok(line, " \t");
  if (word == nullptr) return;
  if (strcmp(word, "LAND") != 0) {
    printConsoleHelp();
    return;
  }
  char* targetText = strtok(nullptr, " \t");
  if (targetText == nullptr || strtok(nullptr, " \t") != nullptr) {
    printConsoleHelp();
    return;
  }
  if (strcmp(targetText, "ALL") == 0) {
    sendGroundCommand(MeshCommand::Land, kBroadcastNodeId, nowMs);
    return;
  }
  char* end = nullptr;
  const unsigned long id = strtoul(targetText, &end, 10);
  if (*end != '\0' || id == 0 || id > kHighestSupportedNodeId) {
    Serial.println("[GROUND] Drone ID must be 1-31 or ALL");
    return;
  }
  sendGroundCommand(MeshCommand::Land, static_cast<NodeId>(id), nowMs);
}

void serviceGroundConsole(uint32_t nowMs) {
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\n' || ch == '\r') {
      if (consoleLength > 0) {
        consoleLine[consoleLength] = '\0';
        runConsoleLine(consoleLine, nowMs);
      }
      consoleLength = 0;
    } else if (consoleLength < sizeof(consoleLine) - 1) {
      consoleLine[consoleLength++] =
          static_cast<char>(toupper(static_cast<unsigned char>(ch)));
    }
  }
}

void printRoute() {
  // Prints the current route, e.g. "[MESH] ROUTE -> via Node 2 (2 hops to ground)".
  if (router.hasRoute()) {
    Serial.print("[MESH] ROUTE -> via Node ");
    Serial.print(router.nextHop());
    Serial.print(" (");
    Serial.print(router.hopsToGround());
    Serial.println(" hops to ground)");
  } else {
    Serial.println("[MESH] ROUTE -> NONE (no path to ground)");
  }
}

void handleReceivedPacket(const TransportPacket& radioPacket,
                          uint32_t nowMs) {
  // First validate magic bytes, version, IDs, packet type, and exact length.
  // Payload-specific code runs only after this common safety gate succeeds.
  DecodedPacket packet;
  const ProtocolError error =
      decodePacket(radioPacket.data, radioPacket.length, packet);
  if (error != ProtocolError::None) {
    Serial.print("[PROTOCOL] Invalid packet ignored: ");
    Serial.println(protocolErrorName(error));
    return;
  }
  if (packet.header.sourceNodeId == nodeConfig.nodeId) {
    // Ignore an echoed copy of this node's own transmission.
    return;
  }
  if (packet.header.destinationNodeId != kBroadcastNodeId &&
      packet.header.destinationNodeId != nodeConfig.nodeId) {
    // LoRa radios may physically hear traffic addressed to another node. This
    // filter ensures only broadcast or locally addressed packets are processed.
    return;
  }

  switch (packet.header.packetType) {
    case PacketType::Hello:
      handleHello(packet, radioPacket, nowMs);
      break;
    case PacketType::Telemetry:
      handleTelemetry(packet, nowMs);
      break;
    case PacketType::Command:
      handleCommand(packet, nowMs);
      break;
    case PacketType::CommandAck:
      handleCommandAck(packet, nowMs);
      break;
  }
}

void setup() {
  // 115200 bits per second is fast enough for readable diagnostics and matches
  // the original PING/PONG Serial Monitor setting.
  Serial.begin(115200);
  // Preserve the tested native-USB startup pause so early diagnostics are not
  // lost when the board disconnects and reconnects after reset.
  delay(3000);

  if (!isValidNodeConfig(nodeConfig)) {
    stopWithMessage("Invalid node configuration; firmware halted.");
  }

  // Print the active identity at every boot. This is an important check before
  // deploying multiple boards: duplicate node IDs would make packets ambiguous.
  Serial.println();
  Serial.println("ANDY 8: RED - Phase 2 mesh firmware");
  Serial.print("Node ID: ");
  Serial.println(nodeConfig.nodeId);
  Serial.print("Role: ");
  Serial.println(nodeRoleName(nodeConfig.role));
  Serial.print("Device: ");
  Serial.println(nodeConfig.deviceName);
  Serial.print("Firmware: ");
  Serial.print(nodeConfig.firmwareMajor);
  Serial.print('.');
  Serial.println(nodeConfig.firmwareMinor);
  Serial.print("Protocol: ");
  Serial.println(nodeConfig.protocolVersion);

  // All managers receive one consistent startup time and the same NodeConfig.
  const uint32_t nowMs = millis();
  neighborTable.begin(nodeConfig.nodeId, nodeConfig.neighborTimeoutMs);
  router.begin(nodeConfig.nodeId,
               nodeConfig.role == NodeRole::GroundStation);
  helloManager.begin(nodeConfig, nowMs);
  commandExecutor.begin(nodeConfig.nodeId);
  randomSeed(esp_random());
  groundCommandSequence = static_cast<uint16_t>(esp_random());
  if (!telemetryManager.begin(nodeConfig, Serial, nowMs)) {
    stopWithMessage("Telemetry provider failed to initialize.");
  }
  if (!radioManager.begin(nodeConfig.radio, Serial)) {
    stopWithMessage("Radio failed to initialize; firmware halted.");
  }

  // Schedule diagnostic output with millis(); no delay is added to loop().
  nextNeighborTablePrintMs = nowMs + nodeConfig.neighborTableIntervalMs;
  Serial.println("Mesh node ready.");
  if (nodeConfig.role == NodeRole::GroundStation) {
    printConsoleHelp();
  }
}

void loop() {
  // Capture millis() once so every task in this loop iteration uses the same
  // time reference.
  const uint32_t nowMs = millis();

  // MAVLink update consumes at most 256 UART bytes; simulation update is empty.
  // Neither path waits for the next telemetry transmit deadline.
  telemetryManager.update(nowMs);

  if (nodeConfig.role == NodeRole::GroundStation) {
    serviceGroundConsole(nowMs);
  } else {
    serviceCommandExecutor(nowMs);
  }

  // poll returns true only when a complete LoRa packet was read successfully.
  TransportPacket radioPacket;
  if (radioManager.poll(radioPacket)) {
    handleReceivedPacket(radioPacket, nowMs);
  }

  NodeId lostNodes[kMaxNeighbors];
  const size_t lostCount = neighborTable.updateTimeouts(nowMs, lostNodes, kMaxNeighbors);
  for (size_t i = 0; i < lostCount && i < kMaxNeighbors; ++i) {
    Serial.print("[MESH] WARNING: Node "); Serial.print(lostNodes[i]);
    Serial.println(" heartbeat timeout; marked LOST");
  }

  // Route re-evaluated after timeouts so a LOST next hop is replaced.
  if (router.update(neighborTable)) {
    printRoute();
  }

  // Full table output is diagnostic only. count avoids printing an empty table,
  // and the timer prevents Serial output from overwhelming normal loop work.
  if (neighborTable.count() > 0 &&
      timeReached(nowMs, nextNeighborTablePrintMs)) {
    printNeighborTable();
    nextNeighborTablePrintMs = nowMs + nodeConfig.neighborTableIntervalMs;
  }

  if (!radioManager.readyToSend()) {
    // Receiving and timeout work above still ran even when the radio is busy.
    return;
  }
  // Commands and their replies go ahead of HELLO and telemetry.
  if (serviceOutbound(nowMs)) {
    return;
  }
  if (helloManager.due(nowMs)) {
    // HELLO has priority because discovery and heartbeat monitoring depend on a
    // regular one-second beacon. Telemetry remains due and is tried after TX.
    sendHello(nowMs);
    return;
  }
  if (telemetryManager.due(nowMs)) {
    // Only aircraft profiles report due. The Ground Station remains receive-only.
    sendTelemetry(nowMs);
  }
}
