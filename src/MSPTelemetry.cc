
#include <exception>

#include <logging.hh>
#include <MSPTelemetry.hh>

#define MAVLINK_MAX_MSG 2048

MSPTelemetry::MSPTelemetry(const std::string &device, uint32_t baudrate,
			   const std::string &recv_host, uint16_t recv_port,
			   const std::string &send_host, uint16_t send_port) :
  m_recv_endpoint(boost::asio::ip::address_v4::any(), recv_port),
  m_recv_sock(m_recv_service, m_recv_endpoint),
  m_send_thread(0), m_recv_thread(0),
  m_stop(false) {

  m_recv_sock.set_option(boost::asio::socket_base::broadcast(true));

  // Connect to the flight controller via MSP over a UART
  m_fcu.setLoggingLevel(msp::client::LoggingLevel::INFO);
  try {
    LOG_INFO << "Opening " << device << " with baudrate = " << baudrate;
    m_fcu.connect(device, baudrate);
  } catch (std::exception &e) {
    LOG_CRITICAL << "Error opening " << device;
    LOG_CRITICAL << e.what();
    exit(-1);
  }

  // Initialize the mavlink messages
  m_heartbeat.custom_mode = 0;
  m_heartbeat.type = MAV_TYPE_QUADROTOR;
  m_heartbeat.autopilot = MAV_AUTOPILOT_GENERIC;
  m_heartbeat.base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  m_heartbeat.system_status = MAV_STATE_STANDBY;
  m_heartbeat.mavlink_version = 0;
}

void MSPTelemetry::start() {

  // define subscriptions with specific period
  m_fcu.subscribe(&MSPTelemetry::onStatus, this, 0.25);
  m_fcu.subscribe(&MSPTelemetry::onAttitude, this, 0.1);
  m_fcu.subscribe(&MSPTelemetry::onAnalog, this, 0.25);
  m_fcu.subscribe(&MSPTelemetry::onRawGPS, this, 0.25);

  // Receive Mavlink messages over UDP
  if (!m_recv_thread) {
    m_recv_thread = new std::thread(&MSPTelemetry::udp_receive_loop, this);
  }

  // Send Mavlink messages over UDP
  if (!m_send_thread) {
    m_send_thread = new std::thread(&MSPTelemetry::udp_send_loop, this);
  }
}

void MSPTelemetry::stop() {
  m_stop = true;
}

void MSPTelemetry::join() {
  m_recv_thread->join();
  m_send_thread->join();
}

void MSPTelemetry::onStatus(const msp::msg::Status& status) {

  // Send the heartbeat message.
  mavlink_message_t msg_hb;
  mavlink_msg_heartbeat_encode(255, 0, &msg_hb, &m_heartbeat);
  send_message(msg_hb);
}

void MSPTelemetry::onAttitude(const msp::msg::Attitude& attitude) {

  // Send the attitude message.
  mavlink_attitude_t mav_attitude;
  mav_attitude.roll = attitude.roll;
  mav_attitude.pitch = attitude.pitch;
  mav_attitude.yaw = attitude.yaw;
  mav_attitude.rollspeed = 0;
  mav_attitude.pitchspeed = 0;
  mav_attitude.yawspeed = 0;
  mavlink_message_t msg_att;
  mavlink_msg_attitude_encode(255, 0, &msg_att, &mav_attitude);
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
  mav_sys_stat.current_battery = analog.amperage;
  mav_sys_stat.battery_remaining = -1;
  mav_sys_stat.drop_rate_comm = 0;
  mav_sys_stat.errors_comm = 0;
  mav_sys_stat.errors_count1 = 0;
  mav_sys_stat.errors_count2 = 0;
  mav_sys_stat.errors_count3 = 0;
  mav_sys_stat.errors_count4 = 0;
  mavlink_message_t msg_ss;
  mavlink_msg_sys_status_encode(255, 0, &msg_ss, &mav_sys_stat);
  send_message(msg_ss);
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
  mavlink_msg_gps_raw_int_encode(255, 0, &msg_gps, &mav_gps);
  send_message(msg_gps);
}


void MSPTelemetry::send_message(const mavlink_message_t &msg) {
  std::shared_ptr<std::vector<uint8_t> > send_buf(new std::vector<uint8_t>(MAVLINK_MAX_MSG));
  int len = mavlink_msg_to_send_buffer(send_buf->data(), &msg);
  send_buf->resize(len);
  m_send_queue.push(send_buf);
}

void MSPTelemetry::udp_send_loop() {
  bool done = false;
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket send_socket(io_service);
  send_socket.open(boost::asio::ip::udp::v4());
  send_socket.set_option(boost::asio::socket_base::broadcast(true));
  boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::any(), 14551);

  while (!done) {
    std::shared_ptr<std::vector<uint8_t> > buf = m_send_queue.pop();
    send_socket.send_to(boost::asio::buffer(buf->data(), buf->size()), send_endpoint);
  }
}

void MSPTelemetry::udp_receive_loop() {
  bool done = false;
  uint8_t buf[MAVLINK_MAX_MSG];
  while (!done) {
    boost::asio::ip::udp::endpoint sender_endpoint;
    memset(buf, 0, MAVLINK_MAX_MSG);
    size_t recv = m_recv_sock.receive_from(boost::asio::buffer(buf, MAVLINK_MAX_MSG),
					     sender_endpoint);
    if (recv > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      LOG_DEBUG << "Received: " << recv;
      for (size_t i = 0; i < recv; ++i) {
	if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {

	  // Handle Message ID
	  switch (msg.msgid) {
	  case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
	    LOG_DEBUG << "MAVLINK_MSG_ID_RC_OVERRIDE";
	    mavlink_rc_channels_override_t rc_msg;
	    mavlink_msg_rc_channels_override_decode(&msg, &rc_msg);
	    std::vector<uint16_t> auxs(8);
	    auxs[0] = rc_msg.chan9_raw;
	    auxs[1] = rc_msg.chan10_raw;
	    auxs[2] = rc_msg.chan11_raw;
	    auxs[3] = rc_msg.chan12_raw;
	    auxs[4] = rc_msg.chan13_raw;
	    auxs[5] = rc_msg.chan14_raw;
	    auxs[6] = rc_msg.chan15_raw;
	    auxs[7] = rc_msg.chan16_raw;
	    m_fcu.setRc(rc_msg.chan1_raw, rc_msg.chan2_raw, rc_msg.chan3_raw, rc_msg.chan4_raw,
			rc_msg.chan5_raw, rc_msg.chan6_raw, rc_msg.chan7_raw, rc_msg.chan8_raw,
			auxs);
	    break;
	  }
	  default:
	    LOG_DEBUG << "Received packet: SYS: " << int(msg.sysid) << " COMP: " << int(msg.compid)
		      << " LEN: " << int(msg.len) << " MSG ID: " << int(msg.msgid);
	    break;
	  }
	}
      }
    }
  }
}
