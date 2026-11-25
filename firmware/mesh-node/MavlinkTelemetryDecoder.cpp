#include "MavlinkTelemetryDecoder.h"
#include "NodeConfig.h"
#include <math.h>
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
#if __has_include(<mavlink/common/mavlink.h>)
#include <mavlink/common/mavlink.h>
#elif __has_include(<common/mavlink.h>)
#include <common/mavlink.h>
#elif __has_include(<mavlink.h>)
#include <mavlink.h>
#else
#error "Generated MAVLink common-dialect headers are required."
#endif
#endif
namespace AndyMesh {
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
namespace {
bool fresh(uint32_t n, uint32_t u, uint32_t f) {
  return u != 0 && static_cast<uint32_t>(n - u) <= f;
}
uint16_t speed(int16_t x, int16_t y) {
  float v = sqrtf(static_cast<float>(x) * x + static_cast<float>(y) * y);
  return v >= 65535 ? 65535 : static_cast<uint16_t>(v + .5F);
}
FlightMode mode(uint8_t m) {
  if (m & MAV_MODE_FLAG_AUTO_ENABLED) return FlightMode::Auto;
  if (m & MAV_MODE_FLAG_STABILIZE_ENABLED) return FlightMode::Stabilized;
  if (m & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) return FlightMode::Manual;
  return FlightMode::Unknown;
}
// PX4 packs its mode into custom_mode: main mode in bits 16-23, auto sub-mode
// in bits 24-31 (px4_custom_mode.h).
FlightMode px4Mode(uint32_t customMode) {
  const uint8_t mainMode = (customMode >> 16) & 0xFF;
  const uint8_t subMode = (customMode >> 24) & 0xFF;
  switch (mainMode) {
    case 1:  // MANUAL
    case 5:  // ACRO
      return FlightMode::Manual;
    case 2:  // ALTCTL
    case 3:  // POSCTL
    case 7:  // STABILIZED
      return FlightMode::Stabilized;
    case 4:  // AUTO
      if (subMode == 5) return FlightMode::ReturnToLaunch;
      if (subMode == 6 || subMode == 9) return FlightMode::Land;  // LAND, PRECLAND
      return FlightMode::Auto;
    case 6:  // OFFBOARD
      return FlightMode::Auto;
    default:
      return FlightMode::Unknown;
  }
}
// Identity this node uses when commanding the flight controller; distinct
// from QGroundControl's default system ID 255.
constexpr uint8_t kMeshSystemId = 245;
constexpr uint8_t kMeshComponentId = MAV_COMP_ID_ONBOARD_COMPUTER;
}  // namespace
MavlinkTelemetryDecoder::MavlinkTelemetryDecoder()
    : latest_{0,
              0,
              0,
              0,
              0,
              0,
              0xFF,
              0,
              0,
              0,
              GpsFixStatus::NoGps,
              FlightMode::Unknown,
              false,
              SystemHealth::Unknown,
              0},
      freshnessMs_(3000),
      positionMs_(0),
      altitudeMs_(0),
      batteryMs_(0),
      speedMs_(0),
      heartbeatMs_(0),
      healthMs_(0),
      headingMs_(0),
      gpsFixMs_(0),
      globalPositionMs_(0) {}
// begin assigns the mesh identity once. It also stores the maximum age allowed
// for each measurement; an old GPS value must not look like a current fix.
void MavlinkTelemetryDecoder::begin(NodeId id, uint32_t f) {
  latest_.originatingNodeId = id;
  freshnessMs_ = f;
}
void MavlinkTelemetryDecoder::ingestByte(uint8_t b, uint32_t now) {
  // Generated MAVLink parsers keep one byte-assembly state per channel. Using
  // the mesh node ID as the channel prevents three SITL UDP streams from
  // corrupting one another when this process updates nodes in turn.
  mavlink_message_t msg;
  mavlink_status_t status;
  const mavlink_channel_t channel = static_cast<mavlink_channel_t>(
      latest_.originatingNodeId % MAVLINK_COMM_NUM_BUFFERS);
  if (!mavlink_parse_char(channel, b, &msg, &status)) return;
  switch (msg.msgid) {
    // HEARTBEAT does not contain measurements. It tells us whether PX4 is armed
    // and supplies broad mode flags that fit the project's compact FlightMode.
    // Only the autopilot's heartbeat counts; a GCS or peripheral heartbeat
    // forwarded on the same link must not change armed state or command target.
    case MAVLINK_MSG_ID_HEARTBEAT: {
      mavlink_heartbeat_t m;
      mavlink_msg_heartbeat_decode(&msg, &m);
      if (m.autopilot == MAV_AUTOPILOT_INVALID || m.type == MAV_TYPE_GCS) break;
      latest_.armed = (m.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      latest_.flightMode =
          (m.autopilot == MAV_AUTOPILOT_PX4 &&
           (m.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0)
              ? px4Mode(m.custom_mode)
              : mode(m.base_mode);
      targetSystem_ = msg.sysid;
      targetComponent_ = msg.compid;
      heartbeatMs_ = now;
      break;
    }
    // Answers addressed to QGroundControl on another link are not ours.
    case MAVLINK_MSG_ID_COMMAND_ACK: {
      mavlink_command_ack_t m;
      mavlink_msg_command_ack_decode(&msg, &m);
      if (msg.sysid == targetSystem_ &&
          (m.target_system == kMeshSystemId || m.target_system == 0)) {
        ackCommand_ = m.command;
        ackResult_ = m.result;
        ackPending_ = true;
      }
      break;
    }
    // GLOBAL_POSITION_INT is preferred because it includes E7 coordinates,
    // relative altitude, velocity, and heading in precise integer units.
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      mavlink_global_position_int_t m;
      mavlink_msg_global_position_int_decode(&msg, &m);
      latest_.latitudeE7 = m.lat;
      latest_.longitudeE7 = m.lon;
      latest_.altitudeCm = m.relative_alt / 10;
      latest_.groundSpeedCms = speed(m.vx, m.vy);
      positionMs_ = altitudeMs_ = speedMs_ = globalPositionMs_ = now;
      if (m.hdg != UINT16_MAX && m.hdg < 36000) {
        latest_.headingCentidegrees = m.hdg;
        headingMs_ = now;
      }
      break;
    }
    // GPS_RAW_INT adds fix quality. Below 3D, status is reported but the
    // received coordinates do not become valid. Course over ground is a heading
    // fallback.
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      mavlink_gps_raw_int_t m;
      mavlink_msg_gps_raw_int_decode(&msg, &m);
      latest_.gpsFixStatus =
          static_cast<GpsFixStatus>(m.fix_type <= 8 ? m.fix_type : 0);
      gpsFixMs_ = now;
      // Its altitude is above sea level, so it is only used when
      // GLOBAL_POSITION_INT (height above home) is not arriving.
      if (m.fix_type >= 3 && !fresh(now, globalPositionMs_, freshnessMs_)) {
        latest_.latitudeE7 = m.lat;
        latest_.longitudeE7 = m.lon;
        latest_.altitudeCm = m.alt / 10;
        positionMs_ = altitudeMs_ = now;
      }
      if (m.vel != UINT16_MAX) {
        latest_.groundSpeedCms = m.vel;
        speedMs_ = now;
      }
      if (m.cog != UINT16_MAX && m.cog < 36000) {
        latest_.headingCentidegrees = m.cog;
        headingMs_ = now;
      }
      break;
    }
    // SYS_STATUS contains battery and sensor-health masks. All enabled sensor
    // bits must also be healthy before the compact result can be marked OK.
    case MAVLINK_MSG_ID_SYS_STATUS: {
      mavlink_sys_status_t m;
      mavlink_msg_sys_status_decode(&msg, &m);
      if (m.voltage_battery != UINT16_MAX)
        latest_.batteryMillivolts = m.voltage_battery;
      if (m.battery_remaining >= 0)
        latest_.batteryPercent = static_cast<uint8_t>(m.battery_remaining);
      batteryMs_ = now;
      uint32_t e = m.onboard_control_sensors_enabled;
      latest_.systemHealth = e == 0
                                 ? SystemHealth::Unknown
                                 : ((m.onboard_control_sensors_health & e) == e
                                        ? SystemHealth::Ok
                                        : SystemHealth::Warning);
      healthMs_ = now;
      break;
    }
    // BATTERY_STATUS lists cell voltages; UINT16_MAX marks an unknown/end cell.
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      mavlink_battery_status_t m;
      mavlink_msg_battery_status_decode(&msg, &m);
      if (m.voltages[0] != UINT16_MAX) {
        uint32_t sum = 0;
        for (size_t i = 0; i < 10 && m.voltages[i] != UINT16_MAX; ++i)
          sum += m.voltages[i];
        latest_.batteryMillivolts =
            sum > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(sum);
      }
      if (m.battery_remaining >= 0)
        latest_.batteryPercent = static_cast<uint8_t>(m.battery_remaining);
      batteryMs_ = now;
      break;
    }
    // VFR_HUD is a fallback when more precise position messages are not fresh.
    case MAVLINK_MSG_ID_VFR_HUD: {
      mavlink_vfr_hud_t m;
      mavlink_msg_vfr_hud_decode(&msg, &m);
      float cms = m.groundspeed * 100;
      latest_.groundSpeedCms =
          cms <= 0 ? 0
                   : (cms >= 65535 ? 65535 : static_cast<uint16_t>(cms + .5F));
      speedMs_ = now;
      if (!fresh(now, altitudeMs_, freshnessMs_)) {
        latest_.altitudeCm = static_cast<int32_t>(m.alt * 100);
        altitudeMs_ = now;
      }
      if (m.heading >= 0 && m.heading < 360) {
        latest_.headingCentidegrees = static_cast<uint16_t>(m.heading) * 100;
        headingMs_ = now;
      }
      break;
    }
    default:
      break;
  }
}
// read rebuilds validity at the moment a mesh packet is prepared. Measurements
// age independently because MAVLink messages arrive at different rates.
bool MavlinkTelemetryDecoder::read(uint32_t now, TelemetryData& out) {
  latest_.timestampMs = now;
  latest_.validityFlags = 0;
  if (fresh(now, positionMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidPosition;
  if (fresh(now, altitudeMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidAltitude;
  if (fresh(now, batteryMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidBattery;
  if (fresh(now, speedMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidGroundSpeed;
  if (fresh(now, heartbeatMs_, freshnessMs_))
    latest_.validityFlags |=
        TelemetryValidFlightMode | TelemetryValidArmedState;
  if (fresh(now, healthMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidSystemHealth;
  if (fresh(now, headingMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidHeading;
  if (fresh(now, gpsFixMs_, freshnessMs_))
    latest_.validityFlags |= TelemetryValidGpsFix;
  if (latest_.validityFlags == 0) return false;
  out = latest_;
  return true;
}
bool MavlinkTelemetryDecoder::autopilotPresent(uint32_t now) const {
  return fresh(now, heartbeatMs_, freshnessMs_);
}
size_t MavlinkTelemetryDecoder::packCommandLong(uint16_t command,
                                                const float p[7],
                                                uint8_t confirmation,
                                                uint32_t now, uint8_t* out,
                                                size_t capacity) {
  if (out == nullptr || p == nullptr || capacity < MAVLINK_MAX_PACKET_LEN ||
      !autopilotPresent(now))
    return 0;
  const mavlink_channel_t channel = static_cast<mavlink_channel_t>(
      latest_.originatingNodeId % MAVLINK_COMM_NUM_BUFFERS);
  mavlink_message_t msg;
  mavlink_msg_command_long_pack_chan(kMeshSystemId, kMeshComponentId, channel,
                                     &msg, targetSystem_, targetComponent_,
                                     command, confirmation, p[0], p[1], p[2],
                                     p[3], p[4], p[5], p[6]);
  return mavlink_msg_to_send_buffer(out, &msg);
}
bool MavlinkTelemetryDecoder::takeCommandAck(uint16_t& command,
                                             uint8_t& result) {
  if (!ackPending_) return false;
  ackPending_ = false;
  command = ackCommand_;
  result = ackResult_;
  return true;
}
#else
MavlinkTelemetryDecoder::MavlinkTelemetryDecoder()
    : latest_{},
      freshnessMs_(0),
      positionMs_(0),
      altitudeMs_(0),
      batteryMs_(0),
      speedMs_(0),
      heartbeatMs_(0),
      healthMs_(0),
      headingMs_(0),
      gpsFixMs_(0),
      globalPositionMs_(0) {}
void MavlinkTelemetryDecoder::begin(NodeId, uint32_t) {}
void MavlinkTelemetryDecoder::ingestByte(uint8_t, uint32_t) {}
bool MavlinkTelemetryDecoder::read(uint32_t, TelemetryData&) { return false; }
bool MavlinkTelemetryDecoder::autopilotPresent(uint32_t) const { return false; }
size_t MavlinkTelemetryDecoder::packCommandLong(uint16_t, const float[7],
                                                uint8_t, uint32_t, uint8_t*,
                                                size_t) {
  return 0;
}
bool MavlinkTelemetryDecoder::takeCommandAck(uint16_t&, uint8_t&) {
  return false;
}
#endif
}  // namespace AndyMesh
