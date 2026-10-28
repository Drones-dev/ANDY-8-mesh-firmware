#pragma once

#include <HardwareSerial.h>

#include "TelemetryProvider.h"

/*
  PX4 MAVLink telemetry source

  MAVLink is a compact message protocol commonly used between flight
  controllers and companion devices. UART is the asynchronous serial electrical
  connection that carries those message bytes. This provider reads PX4 data from
  the ESP32 hardware serial port and maps several MAVLink message types into the
  project's small, source-independent TelemetryData format.
*/
namespace AndyMesh {

class MavlinkTelemetryProvider : public TelemetryProvider {
 public:
  explicit MavlinkTelemetryProvider(HardwareSerial& serial);

  bool begin(const NodeConfig& config, Print& log) override;
  void update(uint32_t nowMs) override;
  bool read(uint32_t nowMs, TelemetryData& telemetry) override;
  const char* name() const override;

  // Exposed so recorded or synthetic MAVLink byte streams can exercise the
  // same parser without a connected flight controller.
  void ingestByte(uint8_t byte, uint32_t nowMs);

 private:
  HardwareSerial& serial_;       // ESP32 UART connected to flight-controller UART4.
  NodeId nodeId_;                // Aircraft identity placed in outgoing telemetry.
  uint32_t freshnessMs_;         // Maximum acceptable age for each field group.
  bool initialized_;             // True only after MAVLink UART setup succeeds.

  // latest_ holds the newest value seen for every common telemetry field. Each
  // group also has its own update time because MAVLink messages arrive
  // independently; a recent HEARTBEAT does not make an old GPS fix fresh.
  TelemetryData latest_;
  uint32_t positionUpdatedMs_;
  uint32_t altitudeUpdatedMs_;
  uint32_t batteryUpdatedMs_;
  uint32_t speedUpdatedMs_;
  uint32_t heartbeatUpdatedMs_;
  uint32_t healthUpdatedMs_;
};

}  // namespace AndyMesh
