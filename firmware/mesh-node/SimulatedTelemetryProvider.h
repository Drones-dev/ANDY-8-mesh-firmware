#pragma once

#include "TelemetryProvider.h"

/*
  Predictable practice flight data

  Simulation exercises serialization, LoRa transmission, and Ground Station
  display before a flight controller is connected. It is deliberately separate
  from MAVLink so fake values can be completely disabled for real flights.
*/
namespace AndyMesh {

class SimulatedTelemetryProvider : public TelemetryProvider {
 public:
  SimulatedTelemetryProvider();

  bool begin(const NodeConfig& config, Print& log) override;
  void update(uint32_t nowMs) override;
  bool read(uint32_t nowMs, TelemetryData& telemetry) override;
  const char* name() const override;

 private:
  NodeId nodeId_;          // Keeps simultaneous drones' data distinguishable.
  uint32_t startedAtMs_;   // Starting point for repeatable time-based changes.
  bool initialized_;      // False on the receive-only Ground Station profile.
};

}  // namespace AndyMesh
