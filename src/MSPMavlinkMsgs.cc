
#include <exception>

#include <logging.hh>
#include <MSPTelemetry.hh>

void MSPTelemetry::onStatus(const msp::msg::Status& status) {

  // Convert the MSP fleight mode to an appropriate mavlink flight mode
  fcu::FlightMode flight_mode = m_fcu.getFlightMode();
  
  // Send the heartbeat message.
  mavlink_message_t msg_hb;
  mavlink_msg_heartbeat_encode(m_sysid, m_compid, &msg_hb, &m_heartbeat);
  send_message(msg_hb);

  mavlink_param_value_t param_value;
  param_value.param_value = 255;
  param_value.param_count = 1;
  param_value.param_index = 0;
  strncpy(param_value.param_id, "SYSID_MYGCS", 
          MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  param_value.param_type = 0;
  mavlink_message_t msg_pv;
  mavlink_msg_param_value_encode(m_sysid, m_compid, &msg_pv, &param_value);
  send_message(msg_pv);
}

void MSPTelemetry::onAttitude(const msp::msg::Attitude& attitude) {

  // Send the attitude message.
  mavlink_attitude_t mav_attitude;
  mav_attitude.roll = static_cast<float>(attitude.roll) * M_PI / 1800.0;
  mav_attitude.pitch = static_cast<float>(attitude.pitch) * M_PI / 1800.0;
  mav_attitude.yaw = static_cast<float>(attitude.yaw) * M_PI / 1800.0;
  mav_attitude.rollspeed = 0;
  mav_attitude.pitchspeed = 0;
  mav_attitude.yawspeed = 0;
  mavlink_message_t msg_att;
  mavlink_msg_attitude_encode(m_sysid, m_compid, &msg_att, &mav_attitude);
  send_message(msg_att);
}

void MSPTelemetry::onAnalog(const msp::msg::Analog& analog) {
  uint32_t sensors = MAV_SYS_STATUS_SENSOR_3D_GYRO; //assume we always have gyro
  sensors |= MAV_SYS_STATUS_SENSOR_YAW_POSITION; 
  sensors |= m_fcu.hasAccelerometer() ?
    (MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION) : 0;
  sensors |= m_fcu.hasBarometer() ? MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL : 0;
  sensors |= m_fcu.hasMagnetometer() ? MAV_SYS_STATUS_SENSOR_3D_MAG : 0;
  sensors |= m_fcu.hasGPS() ? MAV_SYS_STATUS_SENSOR_GPS : 0;
  sensors |= m_fcu.hasSonar() ? MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL : 0;

  // Send the system status message
  mavlink_sys_status_t mav_sys_stat;
  mav_sys_stat.onboard_control_sensors_present = sensors;
  mav_sys_stat.onboard_control_sensors_enabled = sensors;
  mav_sys_stat.onboard_control_sensors_health = sensors;
  mav_sys_stat.load = 500;
  mav_sys_stat.voltage_battery = analog.vbat * 100;
  mav_sys_stat.current_battery = analog.amperage / 10.0;
  mav_sys_stat.battery_remaining = -1;
  mav_sys_stat.drop_rate_comm = 0;
  mav_sys_stat.errors_comm = 0;
  mav_sys_stat.errors_count1 = 0;
  mav_sys_stat.errors_count2 = 0;
  mav_sys_stat.errors_count3 = 0;
  mav_sys_stat.errors_count4 = 0;
  mavlink_message_t msg;
  mavlink_msg_sys_status_encode(m_sysid, m_compid, &msg, &mav_sys_stat);
  send_message(msg);

  // Send the battery status message
  mavlink_battery_status_t mav_bat_stat;
  mav_bat_stat.current_consumed = -1;
  mav_bat_stat.energy_consumed = -1;
  mav_bat_stat.temperature = INT16_MAX;
  mav_bat_stat.voltages[0] = analog.vbat * 10 / 3;
  mav_bat_stat.voltages[1] = analog.vbat * 10 / 3;
  mav_bat_stat.voltages[2] = analog.vbat * 10 / 3;
  for (uint8_t i = 3; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; ++i) {
    mav_bat_stat.voltages[i] = UINT16_MAX;
  }
  mav_bat_stat.current_battery = analog.amperage / 10.0;
  mav_bat_stat.id = 0;
  mav_bat_stat.battery_function = 0;
  mav_bat_stat.type = 0;
  mav_bat_stat.battery_remaining -1;
  mav_bat_stat.time_remaining = 0;
  mav_bat_stat.charge_state = 0;
  mavlink_msg_battery_status_encode(m_sysid, m_compid, &msg, &mav_bat_stat);
  send_message(msg);
}

void MSPTelemetry::onRawGPS(const msp::msg::RawGPS& gps) {

  // Send the raw GPS message
  mavlink_gps_raw_int_t mav_gps;
  mav_gps.time_usec = 0;
  mav_gps.fix_type = gps.fix;
  mav_gps.lat = gps.lat;
  mav_gps.lon = gps.lon;
  mav_gps.alt = gps.altitude;
  mav_gps.eph = gps.hdop_set ? gps.hdop : 0;
  mav_gps.epv = 0;
  mav_gps.vel = gps.ground_speed;
  mav_gps.cog = 0;
  mav_gps.satellites_visible = gps.numSat;
  mav_gps.alt_ellipsoid = 0;
  mav_gps.h_acc = 0;
  mav_gps.v_acc = 0;
  mav_gps.vel_acc = 0;
  mav_gps.hdg_acc = 0;
  mavlink_message_t msg_gps;
  mavlink_msg_gps_raw_int_encode(m_sysid, m_compid, &msg_gps, &mav_gps);
  send_message(msg_gps);

  mavlink_global_position_int_t mav_pos;
  mav_pos.time_boot_ms = 0;
  mav_pos.lat = gps.lat;
  mav_pos.lon = gps.lon;
  mav_pos.alt = gps.altitude;
  mav_pos.relative_alt = gps.altitude;
  mav_pos.vx = 0;
  mav_pos.vy = 0;
  mav_pos.vz = 0;
  mav_pos.hdg = 0;
  mavlink_msg_global_position_int_encode(m_sysid, m_compid, &msg_gps, &mav_pos);
  send_message(msg_gps);

  mavlink_vfr_hud_t mav_hud;
  mav_hud.airspeed = 0;
  mav_hud.groundspeed = 0;
  mav_hud.alt = gps.altitude;
  mav_hud.climb = 0;
  mav_hud.heading = 0;
  mav_hud.throttle = 0;
  mavlink_msg_vfr_hud_encode(m_sysid, m_compid, &msg_gps, &mav_hud);
  send_message(msg_gps);
}
