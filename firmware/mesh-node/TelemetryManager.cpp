#include "TelemetryManager.h"

namespace AndyMesh {

namespace {
// Signed interpretation of an unsigned difference is the standard embedded
// timer pattern and remains safe when millis() wraps around.
bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}
}  // namespace

TelemetryManager::TelemetryManager(TelemetryProvider& provider)
    : provider_(provider),
      transmitEnabled_(false),
      intervalMs_(2000),
      nextSendMs_(0),
      sequenceNumber_(0) {}

bool TelemetryManager::begin(const NodeConfig& config,
                             Print& log,
                             uint32_t nowMs) {
  transmitEnabled_ = config.role != NodeRole::GroundStation;
  intervalMs_ = config.telemetryIntervalMs;
  // Each ID adds 125 ms to the first send. This reduces the chance that several
  // drones powered together will transmit their first telemetry simultaneously.
  nextSendMs_ = nowMs + intervalMs_ + static_cast<uint32_t>(config.nodeId) * 125U;
  sequenceNumber_ = 0;
  return provider_.begin(config, log);
}

void TelemetryManager::update(uint32_t nowMs) { provider_.update(nowMs); }

// update is called on every loop even when a packet is not due. That is required
// for MAVLink because UART bytes must be consumed promptly to avoid overflow.

bool TelemetryManager::due(uint32_t nowMs) const {
  return transmitEnabled_ && timeReached(nowMs, nextSendMs_);
}

bool TelemetryManager::prepare(uint32_t nowMs, TelemetryData& telemetry) {
  if (!transmitEnabled_ || !provider_.read(nowMs, telemetry)) {
    return false;
  }
  // The provider supplies measurements, but network sequence and send timestamp
  // belong here so simulated and MAVLink packets follow identical rules.
  telemetry.sequenceNumber = ++sequenceNumber_;
  telemetry.timestampMs = nowMs;
  return isValidTelemetry(telemetry);
}

void TelemetryManager::markSent(uint32_t nowMs) {
  // Only a packet accepted by RadioManager advances the deadline. If the radio
  // was busy, due remains true and the main loop can try again shortly.
  nextSendMs_ = nowMs + intervalMs_;
}

}  // namespace AndyMesh
