#include "MavlinkTelemetryProvider.h"

#include <math.h>

// MAVLink C headers are generated from message definitions called a dialect.
// The common dialect contains the standard messages used here. These alternate
// include paths support the layouts used by common Arduino and generated-header
// installations.
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
#if __has_include(<mavlink/common/mavlink.h>)
#include <mavlink/common/mavlink.h>
#elif __has_include(<common/mavlink.h>)
#include <common/mavlink.h>
#elif __has_include(<mavlink.h>)
#include <mavlink.h>
#else
#error "MAVLink mode requires generated MAVLink C headers."
#endif
#endif

namespace AndyMesh {

namespace {

// A field is fresh only if at least one matching message was received and its
// age is within the configured limit. Unsigned subtraction handles millis()
// wraparound correctly.
bool isFresh(uint32_t nowMs, uint32_t updatedMs, uint32_t freshnessMs) {
  return updatedMs != 0 &&
         static_cast<uint32_t>(nowMs - updatedMs) <= freshnessMs;
}

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
// MAVLink VFR_HUD speed uses meters per second as a float. The mesh protocol
// stores centimeters per second as uint16_t, so clamp values to the range that
// two bytes can represent (0 to 65,535 cm/s, or 655.35 m/s).
uint16_t metersPerSecondToCms(float metersPerSecond) {
  if (metersPerSecond <= 0.0F) {
    return 0;
  }
  if (metersPerSecond >= 655.35F) {
    return 65535;
  }
  return static_cast<uint16_t>(metersPerSecond * 100.0F + 0.5F);
}

uint16_t vectorSpeedCms(int16_t xCms, int16_t yCms) {
  // GLOBAL_POSITION_INT supplies north/east velocity components in cm/s. The
  // Pythagorean formula sqrt(x^2 + y^2) converts them to horizontal ground speed.
  const float x = static_cast<float>(xCms);
  const float y = static_cast<float>(yCms);
  const float speed = sqrtf(x * x + y * y);
  return speed >= 65535.0F ? 65535 : static_cast<uint16_t>(speed + 0.5F);
}

int32_t metersToCm(float meters) {
  // VFR_HUD altitude is floating-point meters. Clamp before scaling so an
  // extreme/corrupt value cannot overflow the signed 32-bit centimeter field.
  if (meters >= 21474836.0F) {
    return INT32_MAX;
  }
  if (meters <= -21474836.0F) {
    return INT32_MIN;
  }
  return static_cast<int32_t>(meters * 100.0F);
}

FlightMode flightModeFromHeartbeat(uint8_t baseMode) {
  // HEARTBEAT base_mode is a bit field: several independent mode flags can be
  // set in one byte. Prefer AUTO, then STABILIZED, then MANUAL for the compact
  // common mode. PX4-specific detailed modes can be added later without changing
  // radio transmission code.
  if ((baseMode & MAV_MODE_FLAG_AUTO_ENABLED) != 0) {
    return FlightMode::Auto;
  }
  if ((baseMode & MAV_MODE_FLAG_STABILIZE_ENABLED) != 0) {
    return FlightMode::Stabilized;
  }
  if ((baseMode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) != 0) {
    return FlightMode::Manual;
  }
  return FlightMode::Unknown;
}
#endif

}  // namespace

MavlinkTelemetryProvider::MavlinkTelemetryProvider(HardwareSerial& serial)
    : serial_(serial),
      nodeId_(0),
      freshnessMs_(3000),
      initialized_(false),
      latest_{0,
              0,
              0,
              0,
              0,
              0,
              0xFF,
              0,
              0,
              FlightMode::Unknown,
              false,
              SystemHealth::Unknown,
              0},
      // A timestamp of zero means this field group has never been received.
      positionUpdatedMs_(0),
      altitudeUpdatedMs_(0),
      batteryUpdatedMs_(0),
      speedUpdatedMs_(0),
      heartbeatUpdatedMs_(0),
      healthUpdatedMs_(0) {}

bool MavlinkTelemetryProvider::begin(const NodeConfig& config, Print& log) {
  nodeId_ = config.nodeId;
  freshnessMs_ = config.mavlinkFreshnessMs;
  latest_.originatingNodeId = nodeId_;

#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
  // SERIAL_8N1 means 8 data bits, no parity bit, and 1 stop bit. RX/TX pins and
  // baud must be supplied by the verified per-board configuration; the firmware
  // deliberately does not guess UART4 wiring.
  serial_.begin(ANDY_MAVLINK_BAUD, SERIAL_8N1, ANDY_MAVLINK_RX_PIN,
                ANDY_MAVLINK_TX_PIN);
  initialized_ = true;
  log.println("[TELEMETRY] MAVLink provider enabled");
  return true;
#else
  (void)serial_;
  log.println("[TELEMETRY] MAVLink provider was not selected at build time");
  return false;
#endif
}

void MavlinkTelemetryProvider::update(uint32_t nowMs) {
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
  if (!initialized_) {
    return;
  }
  // Bound work per loop so a busy UART cannot starve radio processing.
  // 256 bytes is enough to make steady progress through several MAVLink frames,
  // while still returning control for LoRa receive and heartbeat timers.
  uint16_t bytesHandled = 0;
  while (serial_.available() > 0 && bytesHandled < 256) {
    ingestByte(static_cast<uint8_t>(serial_.read()), nowMs);
    ++bytesHandled;
  }
#else
  (void)nowMs;
#endif
}

void MavlinkTelemetryProvider::ingestByte(uint8_t byte, uint32_t nowMs) {
#if ANDY_TELEMETRY_SOURCE == ANDY_TELEMETRY_MAVLINK
  // MAVLink is a byte-stream protocol: packet boundaries are not supplied by
  // UART. mavlink_parse_char consumes one byte at a time, checks framing and
  // checksum, and returns true only when one complete valid message is ready.
  mavlink_message_t message;
  mavlink_status_t status;
  if (!mavlink_parse_char(MAVLINK_COMM_0, byte, &message, &status)) {
    return;
  }

  switch (message.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      // HEARTBEAT is the regular "autopilot is alive" message. It carries basic
      // mode and armed state, not GPS or battery measurements.
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(&message, &heartbeat);
      latest_.armed =
          (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
      latest_.flightMode = flightModeFromHeartbeat(heartbeat.base_mode);
      heartbeatUpdatedMs_ = nowMs;
      break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
      // GLOBAL_POSITION_INT uses integer units designed for telemetry:
      // latitude/longitude are E7 degrees, relative_alt is millimeters, and
      // vx/vy are north/east centimeters per second.
      mavlink_global_position_int_t position;
      mavlink_msg_global_position_int_decode(&message, &position);
      latest_.latitudeE7 = position.lat;
      latest_.longitudeE7 = position.lon;
      latest_.altitudeCm = position.relative_alt / 10;
      latest_.groundSpeedCms = vectorSpeedCms(position.vx, position.vy);
      positionUpdatedMs_ = nowMs;
      altitudeUpdatedMs_ = nowMs;
      speedUpdatedMs_ = nowMs;
      break;
    }

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
      // GPS_RAW_INT is a fallback position source. fix_type 3 or greater means a
      // 3D fix is available; weaker/no fixes are not treated as valid position.
      mavlink_gps_raw_int_t gps;
      mavlink_msg_gps_raw_int_decode(&message, &gps);
      if (gps.fix_type >= 3) {
        latest_.latitudeE7 = gps.lat;
        latest_.longitudeE7 = gps.lon;
        latest_.altitudeCm = static_cast<int32_t>(gps.alt / 10);
        positionUpdatedMs_ = nowMs;
        altitudeUpdatedMs_ = nowMs;
      }
      if (gps.vel != UINT16_MAX) {
        // MAVLink uses UINT16_MAX (65,535) as the "unknown" marker for GPS speed.
        latest_.groundSpeedCms = gps.vel;
        speedUpdatedMs_ = nowMs;
      }
      break;
    }

    case MAVLINK_MSG_ID_SYS_STATUS: {
      // SYS_STATUS provides overall sensor health and a battery summary. PX4
      // uses special unknown values, so each field is checked before copying.
      mavlink_sys_status_t systemStatus;
      mavlink_msg_sys_status_decode(&message, &systemStatus);
      if (systemStatus.voltage_battery != UINT16_MAX) {
        latest_.batteryMillivolts = systemStatus.voltage_battery;
      }
      if (systemStatus.battery_remaining >= 0) {
        latest_.batteryPercent =
            static_cast<uint8_t>(systemStatus.battery_remaining);
      }
      batteryUpdatedMs_ = nowMs;

      // A sensor is healthy only when every enabled sensor bit is also present
      // in the health mask. No enabled bits means there is not enough information
      // to claim OK, so the common state remains UNKNOWN.
      const uint32_t enabled = systemStatus.onboard_control_sensors_enabled;
      const uint32_t healthy = systemStatus.onboard_control_sensors_health;
      latest_.systemHealth =
          enabled == 0
              ? SystemHealth::Unknown
              : ((healthy & enabled) == enabled ? SystemHealth::Ok
                                                : SystemHealth::Warning);
      healthUpdatedMs_ = nowMs;
      break;
    }

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
      // BATTERY_STATUS may contain up to ten cell voltages. UINT16_MAX marks the
      // end/unknown cells. Summing known cells produces the pack voltage in mV.
      mavlink_battery_status_t battery;
      mavlink_msg_battery_status_decode(&message, &battery);
      if (battery.voltages[0] != UINT16_MAX) {
        uint32_t totalMillivolts = 0;
        for (size_t i = 0; i < 10 && battery.voltages[i] != UINT16_MAX; ++i) {
          totalMillivolts += battery.voltages[i];
        }
        latest_.batteryMillivolts =
            totalMillivolts > UINT16_MAX ? UINT16_MAX
                                         : static_cast<uint16_t>(totalMillivolts);
      }
      if (battery.battery_remaining >= 0) {
        latest_.batteryPercent =
            static_cast<uint8_t>(battery.battery_remaining);
      }
      batteryUpdatedMs_ = nowMs;
      break;
    }

    case MAVLINK_MSG_ID_VFR_HUD: {
      // VFR_HUD gives pilot-display values such as ground speed and altitude.
      // Its altitude reference is usually above mean sea level, while
      // GLOBAL_POSITION_INT relative_alt is relative to home/takeoff.
      mavlink_vfr_hud_t hud;
      mavlink_msg_vfr_hud_decode(&message, &hud);
      latest_.groundSpeedCms = metersPerSecondToCms(hud.groundspeed);
      speedUpdatedMs_ = nowMs;
      // GLOBAL_POSITION_INT relative altitude is preferred. VFR_HUD altitude
      // (typically AMSL) is only a fallback when no fresh relative value exists.
      if (!isFresh(nowMs, altitudeUpdatedMs_, freshnessMs_)) {
        latest_.altitudeCm = metersToCm(hud.alt);
        altitudeUpdatedMs_ = nowMs;
      }
      break;
    }

    default:
      // MAVLink contains many messages that Phase 2 does not need. Ignoring them
      // is normal and keeps the telemetry packet small and focused.
      break;
  }
#else
  (void)byte;
  (void)nowMs;
#endif
}

bool MavlinkTelemetryProvider::read(uint32_t nowMs,
                                    TelemetryData& telemetry) {
  if (!initialized_) {
    return false;
  }

  // Rebuild validity flags at read time; freshness changes as time passes even
  // when no new UART byte arrives.
  latest_.timestampMs = nowMs;
  latest_.validityFlags = 0;
  if (isFresh(nowMs, positionUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |= TelemetryValidPosition;
  }
  if (isFresh(nowMs, altitudeUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |= TelemetryValidAltitude;
  }
  if (isFresh(nowMs, batteryUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |= TelemetryValidBattery;
  }
  if (isFresh(nowMs, speedUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |= TelemetryValidGroundSpeed;
  }
  if (isFresh(nowMs, heartbeatUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |=
        TelemetryValidFlightMode | TelemetryValidArmedState;
  }
  if (isFresh(nowMs, healthUpdatedMs_, freshnessMs_)) {
    latest_.validityFlags |= TelemetryValidSystemHealth;
  }

  // Do not replace missing real values with simulation. A zero validity mask
  // means no fresh MAVLink fields are available, so nothing is transmitted.
  // This avoids showing believable-looking fake data during a real wiring or
  // flight-controller failure.
  if (latest_.validityFlags == 0) {
    return false;
  }
  telemetry = latest_;
  return true;
}

const char* MavlinkTelemetryProvider::name() const { return "MAVLINK"; }

}  // namespace AndyMesh
