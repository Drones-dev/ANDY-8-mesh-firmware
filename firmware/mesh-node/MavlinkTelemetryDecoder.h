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
  // Builds a MAVLink COMMAND_LONG MAV_CMD_NAV_LAND (land at the current
  // position) addressed to the flight controller. Returns the frame length,
  // or 0 when no flight controller has been heard.
  size_t packLandCommand(uint8_t confirmation, uint32_t nowMs, uint8_t* output,
                         size_t outputCapacity);
  // Reports the flight controller's COMMAND_ACK for NAV_LAND once.
  bool takeLandAck(uint8_t& result);

 private:
  TelemetryData latest_;
  uint32_t freshnessMs_, positionMs_, altitudeMs_, batteryMs_, speedMs_;
  uint32_t heartbeatMs_, healthMs_, headingMs_, gpsFixMs_, globalPositionMs_;
  uint8_t targetSystem_ = 0, targetComponent_ = 0;
  bool landAckPending_ = false;
  uint8_t landAckResult_ = 0;
};
}  // namespace AndyMesh
