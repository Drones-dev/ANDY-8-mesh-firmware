#pragma once
#include <stddef.h>
#include <stdint.h>
#include "PacketProtocol.h"
namespace AndyMesh {
// One decoder is shared by physical UART and SITL UDP. Transport wrappers only
// supply bytes, preventing two subtly different MAVLink implementations.
class MavlinkTelemetryDecoder {
 public:
  MavlinkTelemetryDecoder();
  void begin(NodeId nodeId, uint32_t freshnessMs);
  void ingestByte(uint8_t byte, uint32_t nowMs);
  bool read(uint32_t nowMs, TelemetryData& telemetry);

  // True while the flight controller's own HEARTBEAT is fresh.
  bool autopilotPresent(uint32_t nowMs) const;
  // Builds a MAVLink COMMAND_LONG addressed to the flight controller.
  // Returns the frame length, or 0 when no flight controller has been heard.
  size_t packCommandLong(uint16_t command, const float params[7],
                         uint8_t confirmation, uint32_t nowMs, uint8_t* output,
                         size_t outputCapacity);
  // Reports, once, the flight controller's latest COMMAND_ACK addressed to
  // this node.
  bool takeCommandAck(uint16_t& command, uint8_t& result);

 private:
  TelemetryData latest_;
  uint32_t freshnessMs_, positionMs_, altitudeMs_, batteryMs_, speedMs_;
  uint32_t heartbeatMs_, healthMs_, headingMs_, gpsFixMs_, globalPositionMs_;
  uint8_t targetSystem_ = 0, targetComponent_ = 0;
  bool ackPending_ = false;
  uint16_t ackCommand_ = 0;
  uint8_t ackResult_ = 0;
};
}  // namespace AndyMesh
