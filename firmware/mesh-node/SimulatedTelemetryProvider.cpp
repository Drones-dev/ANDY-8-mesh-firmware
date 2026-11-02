#include "SimulatedTelemetryProvider.h"

namespace AndyMesh {

SimulatedTelemetryProvider::SimulatedTelemetryProvider()
    : nodeId_(0), startedAtMs_(0), initialized_(false) {}

bool SimulatedTelemetryProvider::begin(const NodeConfig& config, Print& log) {
  nodeId_ = config.nodeId;
  startedAtMs_ = millis();
  // Only aircraft profiles generate telemetry. The Ground Station still uses
  // the same firmware, but it receives and displays packets instead of sending
  // fake flight data about itself.
  initialized_ = config.role != NodeRole::GroundStation;
  if (initialized_) {
    log.println("[TELEMETRY] Simulated provider enabled");
  } else {
    log.println("[TELEMETRY] Ground station receive-only mode");
  }
  return true;
}

void SimulatedTelemetryProvider::update(uint32_t nowMs) { (void)nowMs; }

// Simulation is calculated when read is requested, so update has no UART or
// sensor work to perform. The cast above documents that nowMs is intentionally
// unused and keeps compiler warnings clean.

bool SimulatedTelemetryProvider::read(uint32_t nowMs,
                                      TelemetryData& telemetry) {
  if (!initialized_) {
    return false;
  }

  // Integer time patterns make the values change slowly and repeatably:
  // - movement moves GPS by small E7 steps over a 200-second cycle;
  // - altitude climbs for 30 seconds, then descends for 30 seconds;
  // - battery loses one percent per minute, stopping after a 75% decrease.
  const uint32_t elapsedSeconds = (nowMs - startedAtMs_) / 1000U;
  const int32_t movement = static_cast<int32_t>(elapsedSeconds % 200U) - 100;
  const uint32_t altitudePhase = elapsedSeconds % 60U;
  const int32_t altitudeOffset =
      altitudePhase <= 30U ? static_cast<int32_t>(altitudePhase) * 8
                           : static_cast<int32_t>(60U - altitudePhase) * 8;
  const uint8_t batteryDrop =
      static_cast<uint8_t>((elapsedSeconds / 60U) > 75U
                               ? 75U
                               : (elapsedSeconds / 60U));

  // Different node-ID offsets prevent Drone 1, 2, and 3 from appearing at the
  // exact same coordinates. The Ground Station can therefore demonstrate that
  // simultaneous telemetry values are not being mixed between aircraft.
  telemetry.originatingNodeId = nodeId_;
  telemetry.sequenceNumber = 0;  // Assigned by TelemetryManager.
  telemetry.timestampMs = nowMs;
  // 340231000 means 34.0231000 degrees. Longitude uses the same E7 scaling.
  telemetry.latitudeE7 =
      340231000 + static_cast<int32_t>(nodeId_) * 2500 + movement;
  telemetry.longitudeE7 =
      -845812000 - static_cast<int32_t>(nodeId_) * 2500 + movement;
  // Altitude is centimeters, battery voltage is millivolts, and ground speed is
  // centimeters per second. The Ground Station converts them for display.
  telemetry.altitudeCm =
      3500 + static_cast<int32_t>(nodeId_) * 500 + altitudeOffset;
  telemetry.batteryPercent = static_cast<uint8_t>(95U - batteryDrop);
  telemetry.batteryMillivolts =
      static_cast<uint16_t>(16800U - static_cast<uint16_t>(batteryDrop) * 65U);
  telemetry.groundSpeedCms =
      static_cast<uint16_t>(250U + (elapsedSeconds % 30U) * 8U);
  telemetry.headingCentidegrees =
      static_cast<uint16_t>((elapsedSeconds * 150U) % 36000U);
  telemetry.gpsFixStatus = GpsFixStatus::Fix3D;
  // Alternate AUTO and STABILIZED every 30 seconds so mode decoding is visible
  // during a bench test instead of remaining a single constant value forever.
  telemetry.flightMode =
      ((elapsedSeconds / 30U) % 2U) == 0U ? FlightMode::Auto
                                          : FlightMode::Stabilized;
  telemetry.armed = true;
  telemetry.systemHealth = SystemHealth::Ok;
  // Every simulated field is freshly calculated, so all validity bits are set.
  telemetry.validityFlags = kKnownTelemetryValidityMask;
  return true;
}

const char* SimulatedTelemetryProvider::name() const { return "SIMULATED"; }

}  // namespace AndyMesh
