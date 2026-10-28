#pragma once

#include "TelemetryProvider.h"

/*
  Telemetry scheduling and sequence numbering

  The provider creates values; this manager decides when an aircraft should send
  them and assigns an increasing sequence number. Separating those jobs keeps
  UART/simulation details out of network timing code.
*/
namespace AndyMesh {

class TelemetryManager {
 public:
  explicit TelemetryManager(TelemetryProvider& provider);

  bool begin(const NodeConfig& config, Print& log, uint32_t nowMs);
  void update(uint32_t nowMs);
  bool due(uint32_t nowMs) const;
  bool prepare(uint32_t nowMs, TelemetryData& telemetry);
  void markSent(uint32_t nowMs);

 private:
  TelemetryProvider& provider_;  // Selected simulation or MAVLink source.
  bool transmitEnabled_;         // False for the receive-only Ground Station.
  uint32_t intervalMs_;          // Configured delay between telemetry packets.
  uint32_t nextSendMs_;          // millis() deadline for the next attempt.
  uint16_t sequenceNumber_;      // Increases for each prepared telemetry sample.
};

}  // namespace AndyMesh
