#pragma once

#include <Arduino.h>

#include "NodeConfig.h"
#include "PacketProtocol.h"

/*
  Common telemetry-source interface

  An interface describes what a telemetry source must do without specifying how
  it gets its data. SimulatedTelemetryProvider calculates practice values, while
  MavlinkTelemetryProvider reads a flight-controller UART. TelemetryManager can
  therefore use either source without changing LoRa packet code.
*/
namespace AndyMesh {

class TelemetryProvider {
 public:
  // A virtual destructor is required when different provider types are used
  // through the same base interface.
  virtual ~TelemetryProvider() = default;

  // begin performs source-specific setup. update handles small pieces of ongoing
  // work without blocking. read copies the newest available common data.
  virtual bool begin(const NodeConfig& config, Print& log) = 0;
  virtual void update(uint32_t nowMs) = 0;
  virtual bool read(uint32_t nowMs, TelemetryData& telemetry) = 0;
  virtual const char* name() const = 0;
};

}  // namespace AndyMesh
