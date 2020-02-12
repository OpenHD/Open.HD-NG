
#include <sys/time.h>

#include <iostream>
#include <thread>

#include <mavlink.h>

#include <CRSFTelemetry.hh>
#include <logging.hh>

#define CRSF_RADIO_ADDRESS 0xEA

typedef enum {
	      CRSF_ATTITUDE = 0x1E,
	      CRSF_BATTERY = 0x08,
	      CRSF_LINK = 0x14,
	      CRSF_GPS = 0x02
} CRSFTelemetryType;

// CRC8 implementation with polynom = x^8+x^7+x^6+x^4+x^2+1 (0xD5)
static const unsigned char crc8tab[256] = {
  0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54,
  0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
  0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06,
  0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
  0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0,
  0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
  0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2,
  0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
  0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9,
  0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
  0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B,
  0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
  0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D,
  0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
  0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F,
  0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
  0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB,
  0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
  0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9,
  0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
  0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F,
  0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
  0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D,
  0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
  0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26,
  0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
  0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74,
  0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
  0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82,
  0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
  0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0,
  0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

inline double cur_time() {
  struct timeval t;
  gettimeofday(&t, 0);
  return double(t.tv_sec) + double(t.tv_usec) * 1e-6;
}

static uint8_t crc8(const uint8_t *ptr, uint32_t len) {
  uint8_t crc = 0;
  for (uint32_t i=0; i<len; i++) {
    crc = crc8tab[crc ^ *ptr++];
  }
  return crc;
}

static void add_crc(std::vector<uint8_t> &buf) {
  uint8_t crc = crc8(&buf[2], buf.size() - 2);
  buf.push_back(crc);
}

void add_int8(std::vector<uint8_t> &buf, int8_t v) {
  const uint8_t *p = reinterpret_cast<const uint8_t*>(&v);
  buf.push_back(*p);
}

void big_endian_uint16(std::vector<uint8_t> &buf, uint16_t v) {
  const uint8_t *rvp = reinterpret_cast<const uint8_t*>(&v);
  buf.push_back(rvp[1]);
  buf.push_back(rvp[0]);
}

void big_endian_uint24(std::vector<uint8_t> &buf, uint32_t v) {
  const uint8_t *rvp = reinterpret_cast<const uint8_t*>(&v);
  buf.push_back(rvp[2]);
  buf.push_back(rvp[1]);
  buf.push_back(rvp[0]);
}

void big_endian_int32(std::vector<uint8_t> &buf, int32_t v) {
  const uint8_t *rvp = reinterpret_cast<const uint8_t*>(&v);
  buf.push_back(rvp[3]);
  buf.push_back(rvp[2]);
  buf.push_back(rvp[1]);
  buf.push_back(rvp[0]);
}

void big_endian_deg_to_radians_10000(std::vector<uint8_t> &buf, float v) {
  big_endian_uint16(buf, static_cast<uint16_t>(rint((v * 10000.0 * M_PI) / 360.0)));
}


CRSFTelemetry::CRSFTelemetry(uint16_t recv_port, uint16_t send_port, uint16_t status_port)
  : m_rec_bat_status(false), m_connected(false), m_rc_fresh(false) {
  std::thread([this, recv_port]() { this->reader_thread(recv_port); }).detach();
  std::thread([this, status_port]() { this->status_thread(status_port); }).detach();
  std::thread([this, send_port]() { this->control_thread(send_port); }).detach();
}

bool CRSFTelemetry::get_value(const std::string &name, float &value) const {
  NVMap::const_iterator mi = m_values.find(name);
  if (mi == m_values.end()) {
    return false;
  }
  value = mi->second;
  return true;
}

void CRSFTelemetry::set_value(const std::string &name, float value) {
  m_values[name] = value;
}

bool CRSFTelemetry::connected() const {
  return m_connected;
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

void CRSFTelemetry::send_link_packet(int8_t rx_rssi1,
				     int8_t rx_rssi2,
				     uint8_t rx_quality,
				     uint8_t rx_snr,
				     uint8_t rx_antenna,
				     uint8_t rf_mode,
				     uint8_t tx_power,
				     int8_t tx_rssi,
				     uint8_t tx_quality,
				     uint8_t tx_snr) {
  std::shared_ptr<std::vector<uint8_t> > bufp(new std::vector<uint8_t>());
  std::vector<uint8_t> &buf = *bufp.get();
  buf.push_back(CRSF_RADIO_ADDRESS);
  buf.push_back(12); // type (1) + Payload size + CRC size (1)
  buf.push_back(CRSF_LINK);
  add_int8(buf, rx_rssi1);
  add_int8(buf, rx_rssi2);
  buf.push_back(rx_quality);
  buf.push_back(rx_snr);
  buf.push_back(rx_antenna);
  buf.push_back(rf_mode);
  buf.push_back(tx_power);
  add_int8(buf, tx_rssi);
  buf.push_back(tx_quality);
  buf.push_back(tx_snr);
  add_crc(buf);
  m_send_queue.push(buf);
}

void CRSFTelemetry::send_attitude_packet(float yaw_deg, float pitch_deg, float roll_deg) {
  std::shared_ptr<std::vector<uint8_t> > bufp(new std::vector<uint8_t>());
  std::vector<uint8_t> &buf = *bufp.get();
  buf.push_back(CRSF_RADIO_ADDRESS);
  buf.push_back(8); // type (1) + Payload size + CRC size (1)
  buf.push_back(CRSF_ATTITUDE);
  big_endian_deg_to_radians_10000(buf, yaw_deg);
  big_endian_deg_to_radians_10000(buf, pitch_deg);
  big_endian_deg_to_radians_10000(buf, roll_deg);
  add_crc(buf);
  m_send_queue.push(buf);
}

void CRSFTelemetry::send_battery_packet(float v, float c, uint32_t cap, float remain) {
  std::shared_ptr<std::vector<uint8_t> > bufp(new std::vector<uint8_t>());
  std::vector<uint8_t> &buf = *bufp.get();
  buf.push_back(CRSF_RADIO_ADDRESS);
  buf.push_back(10); // type (1) + Payload size + CRC size (1)
  buf.push_back(CRSF_BATTERY);
  big_endian_uint16(buf, static_cast<uint16_t>(rint(v * 100.0)));
  big_endian_uint16(buf, static_cast<uint16_t>(rint(c * 100.0)));
  big_endian_uint24(buf, cap);
  buf.push_back(static_cast<uint8_t>(rint(remain)));
  add_crc(buf);
  m_send_queue.push(buf);
}

void CRSFTelemetry::send_gps_packet(int32_t lat, int32_t lon, uint16_t vel,
				    uint16_t heading, uint16_t alt, uint8_t nsat) {
  std::shared_ptr<std::vector<uint8_t> > bufp(new std::vector<uint8_t>());
  std::vector<uint8_t> &buf = *bufp.get();
  buf.push_back(CRSF_RADIO_ADDRESS);
  buf.push_back(17); // type (1) + Payload size + CRC size (1)
  buf.push_back(CRSF_GPS);
  big_endian_int32(buf, lat);
  big_endian_int32(buf, lon);
  big_endian_uint16(buf, vel);
  big_endian_uint16(buf, heading);
  big_endian_uint16(buf, alt);
  buf.push_back(nsat);
  add_crc(buf);
  m_send_queue.push(buf);
}

void CRSFTelemetry::reader_thread(uint16_t port) {
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

  }
}

void CRSFTelemetry::status_thread(uint16_t port) {
  boost::asio::io_service status_service;
  boost::asio::ip::udp::socket status_sock
    (status_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port));
  boost::asio::ip::udp::endpoint sender_endpoint;

  // Ensure we can receive broadcast messages.
  status_sock.set_option(boost::asio::socket_base::broadcast(true));

  double last_recv = 0;
  wifibroadcast_rx_status_forward_t prev_link_stats;
  double rx_quality = 0;
  while(1) {

    // Receive the next link status message
    size_t len = status_sock.receive_from(boost::asio::buffer(&m_link_stats, sizeof(m_link_stats)),
					  sender_endpoint);
    LOG_DEBUG << "Received OpenHD stats structure message: "
	      << len << "/" << sizeof(m_link_stats);
    if (len != sizeof(m_link_stats)) {
      continue;
    }

    // Initialize the previous stats when the first message is received
    // or the previous one is too old
    double time = cur_time();
    double tdiff = time - last_recv;
    if ((tdiff < 0) || (tdiff > 5)) {
      prev_link_stats = m_link_stats;
      rx_quality = 0;
    }
    last_recv = time;

    // Calculate th link quality from the package stats
    // Change to lost_packet_cnt when wfb_bridge fixed
    float cur_lost = m_link_stats.damaged_block_cnt - prev_link_stats.damaged_block_cnt;
    float cur_npkt = m_link_stats.received_packet_cnt - prev_link_stats.received_packet_cnt;
    double cur_rx_quality = (cur_npkt == 0) ? 100.0 :
      std::max(100.0 - 10.0 * cur_lost / cur_npkt, 0.0);
    rx_quality = (rx_quality == 0) ? cur_rx_quality : (rx_quality * 0.9 + cur_rx_quality * 0.1);

    // Send a link package to the Tx
    uint8_t rx_signal_quality = rx_quality;
    uint8_t rx_snr = 8;
    uint8_t rf_mode = 0;
    uint8_t rx_antenna = 0;
    uint8_t tx_power = 2;
    uint8_t tx_signal_quality = 100;
    uint8_t tx_snr = 8;
    send_link_packet(m_link_stats.adapter[0].current_signal_dbm,
		     m_link_stats.adapter[1].current_signal_dbm,
		     rx_signal_quality,
		     rx_snr,
		     rx_antenna,
		     rf_mode,
		     tx_power,
		     m_link_stats.current_signal_joystick_uplink,
		     tx_signal_quality,
		     tx_snr);
    prev_link_stats = m_link_stats;
  }
}

void CRSFTelemetry::control_thread(uint16_t port) {
  int max_length = 1024;
  uint8_t data[max_length];
  bool done = false;
  boost::asio::io_service send_service;
  boost::asio::ip::udp::socket send_sock(send_service);
  send_sock.open(boost::asio::ip::udp::v4());
  send_sock.set_option(boost::asio::socket_base::broadcast(true));
  boost::asio::ip::udp::endpoint sender_endpoint(boost::asio::ip::address_v4::any(), port);

  // Don't send anything until we've recieved a packet.
  while (!m_connected) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
    
  while (!done) {

    // Send control messages every 10 ms
    if (m_rc_fresh) {
      mavlink_message_t msg_rc;
      mavlink_msg_rc_channels_override_encode(1, 1, &msg_rc, &m_mavlink_rc_override);
      int len = mavlink_msg_to_send_buffer(data, &msg_rc);
      send_sock.send_to(boost::asio::buffer(data, len), sender_endpoint);
      LOG_DEBUG << "Sending RC Override: " << len;
      m_rc_fresh = false;
    }

    // Sleep for 10 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
