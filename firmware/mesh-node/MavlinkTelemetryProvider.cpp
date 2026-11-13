#include "MavlinkTelemetryProvider.h"
namespace AndyMesh {
MavlinkTelemetryProvider::MavlinkTelemetryProvider(HardwareSerial& serial)
    : serial_(serial), initialized_(false) {}
bool MavlinkTelemetryProvider::begin(const NodeConfig& config, Print& log) {
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
  decoder_.begin(config.nodeId, config.mavlinkFreshnessMs);
  serial_.begin(ANDY_MAVLINK_BAUD, SERIAL_8N1, ANDY_MAVLINK_RX_PIN,
                ANDY_MAVLINK_TX_PIN);
  initialized_ = true;
  log.println("[TELEMETRY] MAVLink UART provider enabled");
  return true;
#else
  (void)config;
  (void)log;
  return false;
#endif
}
// Limit each loop pass to 256 bytes so heavy serial traffic cannot starve LoRa
// receive processing, HELLO deadlines, or neighbor heartbeat checks.
void MavlinkTelemetryProvider::update(uint32_t now) {
  if (!initialized_) return;
  uint16_t n = 0;
  while (serial_.available() > 0 && n++ < 256)
    decoder_.ingestByte(static_cast<uint8_t>(serial_.read()), now);
}
void MavlinkTelemetryProvider::ingestByte(uint8_t byte, uint32_t now) {
  decoder_.ingestByte(byte, now);
}
bool MavlinkTelemetryProvider::read(uint32_t now, TelemetryData& out) {
  return initialized_ && decoder_.read(now, out);
}
const char* MavlinkTelemetryProvider::name() const { return "MAVLINK_UART"; }
}  // namespace AndyMesh
