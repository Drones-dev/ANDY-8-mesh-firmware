#pragma once
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

 private:
  TelemetryData latest_;
  uint32_t freshnessMs_, positionMs_, altitudeMs_, batteryMs_, speedMs_;
  uint32_t heartbeatMs_, healthMs_, headingMs_, gpsFixMs_, globalPositionMs_;
};
}  // namespace AndyMesh
