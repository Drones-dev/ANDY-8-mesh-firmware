#include "MavlinkSyntheticTest.h"

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK && \
    ANDY_MAVLINK_SELF_TEST
#if __has_include(<mavlink/common/mavlink.h>)
#include <mavlink/common/mavlink.h>
#elif __has_include(<common/mavlink.h>)
#include <common/mavlink.h>
#elif __has_include(<mavlink.h>)
#include <mavlink.h>
#endif
#endif

namespace AndyMesh {

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK && \
    ANDY_MAVLINK_SELF_TEST
namespace {

// Convert a structured MAVLink message back into its transmitted byte frame.
// Feeding one byte at a time accurately imitates how UART delivers the data.
void feedMessage(MavlinkTelemetryProvider& provider,
                 const mavlink_message_t& message,
                 uint32_t nowMs) {
  uint8_t frame[MAVLINK_MAX_PACKET_LEN];
  const uint16_t length = mavlink_msg_to_send_buffer(frame, &message);
  for (uint16_t i = 0; i < length; ++i) {
    provider.ingestByte(frame[i], nowMs);
  }
}

}  // namespace
#endif

bool runMavlinkSyntheticTest(MavlinkTelemetryProvider& provider,
                             uint32_t nowMs,
                             Print& log) {
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK && \
    ANDY_MAVLINK_SELF_TEST
  // Create an armed AUTO-mode quadrotor HEARTBEAT from PX4 system 1.
  mavlink_message_t message;
  mavlink_msg_heartbeat_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &message, MAV_TYPE_QUADROTOR,
      MAV_AUTOPILOT_PX4,
      MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED, 0,
      MAV_STATE_ACTIVE);
  feedMessage(provider, message, nowMs);

  // Add a known Georgia-area position, 45.2 m altitude, and horizontal velocity.
  mavlink_msg_global_position_int_pack(
      1, MAV_COMP_ID_AUTOPILOT1, &message, nowMs, 340231000, -845812000,
      45200, 45200, 100, 200, 0, 9000);
  feedMessage(provider, message, nowMs);

  // Add a known 3.4 m/s display speed. The provider should scale this to 340
  // centimeters per second in the common telemetry format.
  mavlink_msg_vfr_hud_pack(1, MAV_COMP_ID_AUTOPILOT1, &message, 4.0F, 3.4F,
                           90, 45, 45.2F, 0.0F);
  feedMessage(provider, message, nowMs);

  TelemetryData telemetry;
  // PASS requires every field group supplied above plus the exact expected
  // identity-independent values. Battery/health are not required because this
  // short fixture intentionally does not generate those message types.
  const uint16_t required = TelemetryValidPosition | TelemetryValidAltitude |
                            TelemetryValidGroundSpeed |
                            TelemetryValidFlightMode |
                            TelemetryValidArmedState;
  const bool passed = provider.read(nowMs, telemetry) &&
                      (telemetry.validityFlags & required) == required &&
                      telemetry.latitudeE7 == 340231000 &&
                      telemetry.longitudeE7 == -845812000 &&
                      telemetry.groundSpeedCms == 340 && telemetry.armed &&
                      telemetry.headingCentidegrees == 9000 &&
                      telemetry.flightMode == FlightMode::Auto;
  log.print("[MAVLINK SELF-TEST] ");
  log.println(passed ? "PASS" : "FAIL");
  return passed;
#else
  // Keeping a compiled disabled branch makes accidental calls visible in the
  // Serial Monitor without requiring MAVLink headers in simulation builds.
  (void)provider;
  (void)nowMs;
  log.println("[MAVLINK SELF-TEST] Disabled");
  return false;
#endif
}

}  // namespace AndyMesh
