
#include <sys/time.h>

#include <iostream>
#include <thread>

#include <mavlink.h>

#include <CRSFTelemetry.hh>
#include <logging.hh>


CRSFTelemetry::CRSFTelemetry(bool mavlink,
                             uint16_t recv_port, uint16_t send_port, uint16_t status_port)
  : m_mavlink(mavlink), m_rec_bat_status(false), m_connected(false), m_rc_fresh(false),
    m_send_sock(m_send_service), m_sender_endpoint(boost::asio::ip::address_v4::any(), send_port),
    m_last_telemetry_packet_time(0) {
  std::thread([this, recv_port]() { this->reader_thread(recv_port); }).detach();
  std::thread([this, status_port]() { this->status_thread(status_port); }).detach();
  std::thread([this, send_port]() { this->control_thread(); }).detach();
  m_send_sock.open(boost::asio::ip::udp::v4());
  m_send_sock.set_option(boost::asio::socket_base::broadcast(true));
  boost::asio::ip::udp::endpoint sender_endpoint(boost::asio::ip::address_v4::any(), send_port);
}

void CRSFTelemetry::set_rc(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
			   uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8,
			   uint16_t ch9, uint16_t ch10, uint16_t ch11, uint16_t ch12,
			   uint16_t ch13, uint16_t ch14, uint16_t ch15, uint16_t ch16) {
  m_mavlink_rc_override.chan1_raw = ch1;
  m_mavlink_rc_override.chan2_raw = ch2;
  m_mavlink_rc_override.chan3_raw = ch3;
  m_mavlink_rc_override.chan4_raw = ch4;
  m_mavlink_rc_override.chan5_raw = ch5;
  m_mavlink_rc_override.chan6_raw = ch6;
  m_mavlink_rc_override.chan7_raw = ch7;
  m_mavlink_rc_override.chan8_raw = ch8;
  m_mavlink_rc_override.chan9_raw = ch9;
  m_mavlink_rc_override.chan10_raw = ch10;
  m_mavlink_rc_override.chan11_raw = ch11;
  m_mavlink_rc_override.chan12_raw = ch12;
  m_mavlink_rc_override.chan13_raw = ch13;
  m_mavlink_rc_override.chan14_raw = ch14;
  m_mavlink_rc_override.chan15_raw = ch15;
  m_mavlink_rc_override.chan16_raw = ch16;
  m_mavlink_rc_override.target_system = 1;
  m_mavlink_rc_override.target_component = 1;
  m_rc_fresh = true;
}

void CRSFTelemetry::reader_thread(uint16_t port, bool mavlink) {
  mavlink_message_t msg;
  mavlink_status_t status;
  int max_length = 1024;
  uint8_t data[max_length];
  bool messages_requested = false;
  boost::asio::io_service recv_service;
  boost::asio::ip::udp::socket recv_sock
    (recv_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port));
  boost::asio::ip::udp::endpoint sender_endpoint;

  recv_sock.set_option(boost::asio::socket_base::broadcast(true));

  while(1) {
    size_t length = recv_sock.receive_from(boost::asio::buffer(data, max_length), sender_endpoint);
    if (!m_connected) {
      //set_value("ip_address", m_sender_endpoint.address().to_string());
      m_connected = true;
    }

    if (mavlink) {
      bool heartbeat_received = false;
      for (size_t i = 0; i < length; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, data[i], &msg, &status)) {
          m_sysid = msg.sysid;
          m_compid = msg.compid;
          switch (msg.msgid) {
          case MAVLINK_MSG_ID_POWER_STATUS:
            break;
          case MAVLINK_MSG_ID_SYS_STATUS:
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            LOG_DEBUG << "System Status (battery) packet";
            send_battery_packet(sys_status.voltage_battery / 1000.0,
                                std::max(sys_status.current_battery, static_cast<short>(0)) / 100.0,
                                3000, sys_status.battery_remaining);
            break;
          case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT:
            mavlink_nav_controller_output_t nav;
            mavlink_msg_nav_controller_output_decode(&msg, &nav);
            break;
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            set_value("latitude", static_cast<float>(pos.lat) * 1e-7);
            set_value("longitude", static_cast<float>(pos.lon) * 1e-7);
            set_value("altitude", static_cast<float>(pos.alt) / 1000.0);
            set_value("relative_altitude", static_cast<float>(pos.relative_alt) / 1000.0);
            set_value("speed", sqrt(pos.vx * pos.vx + pos.vy * pos.vy + pos.vz * pos.vz) / 100.0);
            set_value("heading", static_cast<float>(pos.hdg) / 100.0);
            break;
          case MAVLINK_MSG_ID_ATTITUDE:
            mavlink_attitude_t att;
            mavlink_msg_attitude_decode(&msg, &att);
            send_attitude_packet(att.roll * 360.0 / M_PI,
                                 att.pitch * 360.0 / M_PI,
                                 att.yaw * 360.0 / M_PI);
            break;
          case MAVLINK_MSG_ID_STATUSTEXT:
            mavlink_statustext_t status;
            mavlink_msg_statustext_decode(&msg, &status);
            break;
          case MAVLINK_MSG_ID_MISSION_CURRENT:
            //std::cerr << "Mission current " << std::endl;
            break;
          case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
            //std::cerr << "Servo raw " << std::endl;
            break;
          case MAVLINK_MSG_ID_RC_CHANNELS:
            //std::cerr << "RC Channels " << std::endl;
            break;
          case MAVLINK_MSG_ID_PARAM_VALUE:
            //std::cerr << "Param value " << std::endl;
            break;
          case MAVLINK_MSG_ID_VIBRATION:
            //std::cerr << "Vibration " << std::endl;
            break;
          case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(&msg, &hb);
            bool is_armed = (hb.base_mode & 0x80);
            set_value("armed", is_armed ? 1.0 : 0.0);
            set_value("mode", static_cast<float>(hb.custom_mode));
            heartbeat_received = true;
            LOG_DEBUG << "Heartbeat received\n";
            uint8_t rx_signal_quality = 90;
            uint8_t rx_snr = 8;
            uint8_t rf_mode = 0;
            uint8_t rx_antenna = 0;
            uint8_t tx_power = 2;
            uint8_t tx_signal_quality = 100;
            uint8_t tx_snr = 8;
            send_link_packet(-30, -50, rx_signal_quality, rx_snr, rx_antenna,
                             rf_mode, tx_power, -20, tx_signal_quality, tx_snr);
            break;
          }
          case MAVLINK_MSG_ID_VFR_HUD:
            //std::cerr << "VFR HUD " << std::endl;
            break;
          case MAVLINK_MSG_ID_RAW_IMU:
            //std::cerr << "Raw IMU " << std::endl;
            break;
          case MAVLINK_MSG_ID_SCALED_PRESSURE:
            //std::cerr << "Scaled Pressure " << std::endl;
            break;
          case MAVLINK_MSG_ID_GPS_RAW_INT: {
            LOG_DEBUG << "GPS Raw";
            mavlink_gps_raw_int_t gps;
            mavlink_msg_gps_raw_int_decode(&msg, &gps);
            send_gps_packet(gps.lat, gps.lon,
                            3600.0 * float(gps.vel) / 10000.0, // cm/s -> km/h / 10
                            gps.cog, float(gps.alt) / 1000.0 + 1000, gps.satellites_visible);
            break;
          }
          case MAVLINK_MSG_ID_SYSTEM_TIME:
            //std::cerr << "System Time " << std::endl;
            break;
          case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
            //std::cerr << "Local position " << std::endl;
            break;
          case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
            //std::cerr << "Autopilot version " << std::endl;
            break;
          case MAVLINK_MSG_ID_COMMAND_ACK:
            //std::cerr << "Command ACK " << std::endl;
            break;
          case MAVLINK_MSG_ID_BATTERY_STATUS:
            mavlink_battery_status_t bat;
            mavlink_msg_battery_status_decode(&msg, &bat);
/*
	  if (bat.voltages[0] != INT16_MAX) {
	    set_value("voltage_battery", bat.voltages[0] / 1000.0);
	    set_value("current_battery",
		      std::max(bat.current_battery, static_cast<short>(0)) / 100.0);
	    set_value("battery_remaining", bat.battery_remaining);
	    m_rec_bat_status = true;
	  }
*/
            break;
          case MAVLINK_MSG_ID_HOME_POSITION:
            mavlink_home_position_t home;
            mavlink_msg_home_position_decode(&msg, &home);
            set_value("home_latitude", home.latitude * 1e-7);
            set_value("home_longitude", home.longitude * 1e-7);
            set_value("home_altitude", home.altitude);
            break;
          case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
            mavlink_rc_channels_raw_t rc_channels_raw;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
            break;
          default:
            LOG_DEBUG << "Received packet: SYS: " << int(msg.sysid)
                      << ", COMP: " << int(msg.compid)
                      << ", LEN: " << int(msg.len)
                      << ", MSG ID: " << int(msg.msgid) << std::endl;
            break;
          }
        }
      }

      if (heartbeat_received && !messages_requested) {
        const uint8_t MAVStreams[] = {
                                      MAV_DATA_STREAM_RAW_SENSORS,
                                      MAV_DATA_STREAM_EXTENDED_STATUS,
                                      MAV_DATA_STREAM_RC_CHANNELS,
                                      MAV_DATA_STREAM_POSITION,
                                      MAV_DATA_STREAM_EXTRA1,
                                      MAV_DATA_STREAM_EXTRA2,
                                      MAVLINK_MSG_ID_ATTITUDE,
                                      MAVLINK_MSG_ID_RADIO_STATUS
        };
        const uint16_t MAVRates[] = { 2, 5, 2, 5, 2, 2, 20, 2 };
        uint8_t data[max_length];
        for (size_t i = 0; i < sizeof(MAVStreams); ++i) {
          /*
            int len = mavlink_msg_request_data_stream_pack(m_sysid, m_compid,
            reinterpret_cast<mavlink_message_t*>(data),
            1, 1, MAVStreams[i], MAVRates[i], 1);
            m_send_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
          */
          /*
            int len = mavlink_msg_message_interval_pack(m_sysid, m_compid,
            reinterpret_cast<mavlink_message_t*>(data),
            MAVStreams[i], 1000000 / MAVRates[i]);
            send_sock.send_to(boost::asio::buffer(data, len), sender_endpoint);
          */
        }
        messages_requested = true;
      }
    } else {
      // Just send it on to the Tx for now
      LOG_DEBUG << "Telem recv: " << length;
      if (data[0] == CRSF_RADIO_ADDRESS_FC) {
        data[0] = CRSF_RADIO_ADDRESS_TX;
      }
      m_last_telemetry_packet_time = cur_time();
      m_send_queue.push(std::vector<uint8_t>(data, data + length));
    }
  }
}


void CRSFTelemetry::send_rc(const std::vector<uint8_t> &msg) {
  m_send_sock.send_to(boost::asio::buffer(msg.data(), msg.size()), m_sender_endpoint);
}

void CRSFTelemetry::control_thread() {
  int max_length = 1024;
  uint8_t data[max_length];
  bool done = false;

  // Don't send anything until we've recieved a packet.
  while (!m_connected || !m_mavlink) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
    
  while (!done) {

    // Send control messages every 10 ms
    if (m_rc_fresh) {
      mavlink_message_t msg_rc;
      mavlink_msg_rc_channels_override_encode(1, 1, &msg_rc, &m_mavlink_rc_override);
      int len = mavlink_msg_to_send_buffer(data, &msg_rc);
      m_send_sock.send_to(boost::asio::buffer(data, len), m_sender_endpoint);
      LOG_DEBUG << "Sending RC Override: " << len;
      m_rc_fresh = false;
    }

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
