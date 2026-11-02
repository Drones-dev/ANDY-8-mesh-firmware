#include "MavlinkTelemetryDecoder.h"
#include "NodeConfig.h"
#include <math.h>
namespace AndyMesh {
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
namespace {
bool fresh(uint32_t n,uint32_t u,uint32_t f){return u!=0&&static_cast<uint32_t>(n-u)<=f;}
uint16_t speed(int16_t x,int16_t y){float v=sqrtf(static_cast<float>(x)*x+static_cast<float>(y)*y);return v>=65535?65535:static_cast<uint16_t>(v+.5F);}
FlightMode mode(uint8_t m){if(m&MAV_MODE_FLAG_AUTO_ENABLED)return FlightMode::Auto;if(m&MAV_MODE_FLAG_STABILIZE_ENABLED)return FlightMode::Stabilized;if(m&MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)return FlightMode::Manual;return FlightMode::Unknown;}
}
MavlinkTelemetryDecoder::MavlinkTelemetryDecoder():latest_{0,0,0,0,0,0,0xFF,0,0,0,GpsFixStatus::NoGps,FlightMode::Unknown,false,SystemHealth::Unknown,0},freshnessMs_(3000),positionMs_(0),altitudeMs_(0),batteryMs_(0),speedMs_(0),heartbeatMs_(0),healthMs_(0),headingMs_(0),gpsFixMs_(0){}
// begin assigns the mesh identity once. It also stores the maximum age allowed
// for each measurement; an old GPS value must not look like a current fix.
void MavlinkTelemetryDecoder::begin(NodeId id,uint32_t f){latest_.originatingNodeId=id;freshnessMs_=f;}
void MavlinkTelemetryDecoder::ingestByte(uint8_t b,uint32_t now){
 // Generated MAVLink parsers keep one byte-assembly state per channel. Using
 // the mesh node ID as the channel prevents three SITL UDP streams from
 // corrupting one another when this process updates nodes in turn.
 mavlink_message_t msg;mavlink_status_t status;
 const mavlink_channel_t channel = static_cast<mavlink_channel_t>(latest_.originatingNodeId % MAVLINK_COMM_NUM_BUFFERS);
 if(!mavlink_parse_char(channel,b,&msg,&status))return;
 switch(msg.msgid){
 // HEARTBEAT does not contain measurements. It tells us whether PX4 is armed
 // and supplies broad mode flags that fit the project's compact FlightMode.
 case MAVLINK_MSG_ID_HEARTBEAT:{mavlink_heartbeat_t m;mavlink_msg_heartbeat_decode(&msg,&m);latest_.armed=(m.base_mode&MAV_MODE_FLAG_SAFETY_ARMED)!=0;latest_.flightMode=mode(m.base_mode);heartbeatMs_=now;break;}
 // GLOBAL_POSITION_INT is preferred because it includes E7 coordinates,
 // relative altitude, velocity, and heading in precise integer units.
 case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:{mavlink_global_position_int_t m;mavlink_msg_global_position_int_decode(&msg,&m);latest_.latitudeE7=m.lat;latest_.longitudeE7=m.lon;latest_.altitudeCm=m.relative_alt/10;latest_.groundSpeedCms=speed(m.vx,m.vy);positionMs_=altitudeMs_=speedMs_=now;if(m.hdg!=UINT16_MAX&&m.hdg<36000){latest_.headingCentidegrees=m.hdg;headingMs_=now;}break;}
 // GPS_RAW_INT adds fix quality. Below 3D, status is reported but the received
 // coordinates do not become valid. Course over ground is a heading fallback.
 case MAVLINK_MSG_ID_GPS_RAW_INT:{mavlink_gps_raw_int_t m;mavlink_msg_gps_raw_int_decode(&msg,&m);latest_.gpsFixStatus=static_cast<GpsFixStatus>(m.fix_type<=8?m.fix_type:0);gpsFixMs_=now;if(m.fix_type>=3){latest_.latitudeE7=m.lat;latest_.longitudeE7=m.lon;latest_.altitudeCm=m.alt/10;positionMs_=altitudeMs_=now;}if(m.vel!=UINT16_MAX){latest_.groundSpeedCms=m.vel;speedMs_=now;}if(m.cog!=UINT16_MAX&&m.cog<36000){latest_.headingCentidegrees=m.cog;headingMs_=now;}break;}
 // SYS_STATUS contains battery and sensor-health masks. All enabled sensor
 // bits must also be healthy before the compact result can be marked OK.
 case MAVLINK_MSG_ID_SYS_STATUS:{mavlink_sys_status_t m;mavlink_msg_sys_status_decode(&msg,&m);if(m.voltage_battery!=UINT16_MAX)latest_.batteryMillivolts=m.voltage_battery;if(m.battery_remaining>=0)latest_.batteryPercent=static_cast<uint8_t>(m.battery_remaining);batteryMs_=now;uint32_t e=m.onboard_control_sensors_enabled;latest_.systemHealth=e==0?SystemHealth::Unknown:((m.onboard_control_sensors_health&e)==e?SystemHealth::Ok:SystemHealth::Warning);healthMs_=now;break;}
 // BATTERY_STATUS lists cell voltages; UINT16_MAX marks an unknown/end cell.
 case MAVLINK_MSG_ID_BATTERY_STATUS:{mavlink_battery_status_t m;mavlink_msg_battery_status_decode(&msg,&m);if(m.voltages[0]!=UINT16_MAX){uint32_t sum=0;for(size_t i=0;i<10&&m.voltages[i]!=UINT16_MAX;++i)sum+=m.voltages[i];latest_.batteryMillivolts=sum>UINT16_MAX?UINT16_MAX:static_cast<uint16_t>(sum);}if(m.battery_remaining>=0)latest_.batteryPercent=static_cast<uint8_t>(m.battery_remaining);batteryMs_=now;break;}
 // VFR_HUD is a fallback when more precise position messages are not fresh.
 case MAVLINK_MSG_ID_VFR_HUD:{mavlink_vfr_hud_t m;mavlink_msg_vfr_hud_decode(&msg,&m);float cms=m.groundspeed*100;latest_.groundSpeedCms=cms<=0?0:(cms>=65535?65535:static_cast<uint16_t>(cms+.5F));speedMs_=now;if(!fresh(now,altitudeMs_,freshnessMs_)){latest_.altitudeCm=static_cast<int32_t>(m.alt*100);altitudeMs_=now;}if(m.heading>=0&&m.heading<360){latest_.headingCentidegrees=static_cast<uint16_t>(m.heading)*100;headingMs_=now;}break;}
 default:break;}}
// read rebuilds validity at the moment a mesh packet is prepared. Measurements
// age independently because MAVLink messages arrive at different rates.
bool MavlinkTelemetryDecoder::read(uint32_t now,TelemetryData& out){latest_.timestampMs=now;latest_.validityFlags=0;if(fresh(now,positionMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidPosition;if(fresh(now,altitudeMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidAltitude;if(fresh(now,batteryMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidBattery;if(fresh(now,speedMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidGroundSpeed;if(fresh(now,heartbeatMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidFlightMode|TelemetryValidArmedState;if(fresh(now,healthMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidSystemHealth;if(fresh(now,headingMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidHeading;if(fresh(now,gpsFixMs_,freshnessMs_))latest_.validityFlags|=TelemetryValidGpsFix;if(latest_.validityFlags==0)return false;out=latest_;return true;}
#else
MavlinkTelemetryDecoder::MavlinkTelemetryDecoder():latest_{},freshnessMs_(0),positionMs_(0),altitudeMs_(0),batteryMs_(0),speedMs_(0),heartbeatMs_(0),healthMs_(0),headingMs_(0),gpsFixMs_(0){}
void MavlinkTelemetryDecoder::begin(NodeId,uint32_t){}
void MavlinkTelemetryDecoder::ingestByte(uint8_t,uint32_t){}
bool MavlinkTelemetryDecoder::read(uint32_t,TelemetryData&){return false;}
#endif
}  // namespace AndyMesh
