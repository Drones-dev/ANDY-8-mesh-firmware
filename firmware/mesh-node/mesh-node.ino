/*
  ANDY 8: RED - Phase 2 mesh node firmware

  Board:   Seeed XIAO ESP32S3 + Wio-SX1262
  Radio:   915 MHz, 125 kHz, SF9, CR 4/7, 14 dBm
  Library: RadioLib

  Copy NodeBuildConfig.example.h to NodeBuildConfig.h and select this board's
  identity before uploading. The same core firmware runs on every node.

  Main-loop responsibilities:
    1. Read completed LoRa packets without blocking.
    2. Broadcast periodic HELLO discovery/heartbeat packets.
    3. Update direct neighbors and detect heartbeat timeouts.
    4. Generate simulated telemetry or ingest MAVLink UART data.
    5. Send telemetry from drone profiles to the Ground Station profile.
*/

#include <Arduino.h>  // millis(), delay(), Serial, and basic Arduino types.

// Each project header owns one focused responsibility. This keeps the main
// sketch readable and prevents radio, protocol, and telemetry details from being
// mixed into one very large loop.
#include "HelloManager.h"
#include "MavlinkSyntheticTest.h"
#include "MavlinkTelemetryProvider.h"
#include "NeighborTable.h"
#include "NodeConfig.h"
#include "PacketProtocol.h"
#include "RadioManager.h"
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

bool sendHello(uint32_t nowMs) {
  // HELLO is first encoded as its seven-byte payload, then wrapped in the common
  // header with destination 255 (broadcast) so every nearby node may receive it.
  const HelloPayload hello = helloManager.makePayload(nowMs);
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
  // Telemetry is addressed to the configured Ground Station, unlike broadcast
  // HELLO discovery traffic.
  if (!transmitPayload(PacketType::Telemetry, nodeConfig.groundStationId,
                       payload, sizeof(payload), nowMs)) {
    return false;
  }
  telemetryManager.markSent(nowMs);
  Serial.print("[Node ");
  Serial.print(nodeConfig.nodeId);
  Serial.print("] TELEMETRY transmitted, sequence ");
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

void handleTelemetry(const DecodedPacket& packet) {
  // As with HELLO, the origin inside the payload must agree with the common
  // header source. This prevents one drone's data from being labeled as another.
  TelemetryData telemetry;
  const ProtocolError error = decodeTelemetryPayload(
      packet.payload, packet.header.payloadLength, telemetry);
  if (error != ProtocolError::None ||
      telemetry.originatingNodeId != packet.header.sourceNodeId) {
    Serial.println("[PROTOCOL] Invalid telemetry ignored");
    return;
  }
  if (nodeConfig.role == NodeRole::GroundStation) {
    // Drone profiles currently consume no addressed telemetry. The Ground
    // Station is the Phase 2 display endpoint.
    printTelemetry(telemetry);
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
      handleTelemetry(packet);
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
  helloManager.begin(nodeConfig, nowMs);
  if (!telemetryManager.begin(nodeConfig, Serial, nowMs)) {
    stopWithMessage("Telemetry provider failed to initialize.");
  }
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK && \
    ANDY_MAVLINK_SELF_TEST
  // This optional startup check sends generated frames through the parser before
  // normal radio operation. It is normally disabled after software validation.
  if (!runMavlinkSyntheticTest(telemetryProvider, nowMs, Serial)) {
    stopWithMessage("MAVLink synthetic parser self-test failed.");
  }
#endif
  if (!radioManager.begin(nodeConfig.radio, Serial)) {
    stopWithMessage("Radio failed to initialize; firmware halted.");
  }

  // Schedule diagnostic output with millis(); no delay is added to loop().
  nextNeighborTablePrintMs = nowMs + nodeConfig.neighborTableIntervalMs;
  Serial.println("Mesh node ready.");
}

void loop() {
  // Capture millis() once so every task in this loop iteration uses the same
  // time reference.
  const uint32_t nowMs = millis();

  // MAVLink update consumes at most 256 UART bytes; simulation update is empty.
  // Neither path waits for the next telemetry transmit deadline.
  telemetryManager.update(nowMs);

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
