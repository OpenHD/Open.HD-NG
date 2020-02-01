
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
  //m_fcu.subscribe(&MSPTelemetry::onIdent, this, 10);
  m_fcu.subscribe(&MSPTelemetry::onStatus, this, 1);

  // using class method callback
  // m_fcu.subscribe(&MSPTelemetry::onImu, this, 0.1);

  //m_fcu.subscribe(&MSPTelemetry::onServo, this, 0.1);
  //m_fcu.subscribe(&MSPTelemetry::onMotor, this, 0.1);
  //m_fcu.subscribe(&MSPTelemetry::onRc, this, 0.1);
  m_fcu.subscribe(&MSPTelemetry::onAttitude, this, 0.1);
  //m_fcu.subscribe(&MSPTelemetry::onAltitude, this);
  //m_fcu.subscribe(&MSPTelemetry::onAnalog, this, 10);
  //m_fcu.subscribe(&MSPTelemetry::onRcTuning, this, 20);
  //m_fcu.subscribe(&MSPTelemetry::onPID, this, 20);
  //m_fcu.subscribe(&MSPTelemetry::onBox, this, 1);
  //m_fcu.subscribe(&MSPTelemetry::onMisc, this, 1);
  //m_fcu.subscribe(&MSPTelemetry::onMotorPins, this, 20);
  //m_fcu.subscribe(&MSPTelemetry::onBoxNames, this, 1);
  //m_fcu.subscribe(&MSPTelemetry::onPidNames, this, 20);
  //m_fcu.subscribe(&MSPTelemetry::onBoxIds, this, 1);
  //m_fcu.subscribe(&MSPTelemetry::onServoConf, this, 20);
  //m_fcu.subscribe(&MSPTelemetry::onDebugMessage, this, 1);
  //m_fcu.subscribe(&MSPTelemetry::onDebug, this, 1);

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

void MSPTelemetry::onIdent(const msp::msg::Ident& ident) { std::cout << ident; }

void MSPTelemetry::onStatus(const msp::msg::Status& status) {

  // Send the heartbeat message.
  mavlink_message_t msg_hb;
  mavlink_msg_heartbeat_encode(255, 0, &msg_hb, &m_heartbeat);
  send_message(msg_hb);
}

void MSPTelemetry::onImu(const msp::msg::RawImu& imu_raw) { std::cout << imu_raw; }

void MSPTelemetry::onServo(const msp::msg::Servo& servo) { std::cout << servo; }

void MSPTelemetry::onMotor(const msp::msg::Motor& motor) { std::cout << motor; }

void MSPTelemetry::onRc(const msp::msg::Rc& rc) { std::cout << rc; }

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
  std::cerr << "Attutude\n";
  mavlink_msg_attitude_encode(255, 0, &msg_att, &mav_attitude);
  send_message(msg_att);
}

void MSPTelemetry::onAltitude(const msp::msg::Altitude& altitude) {
  //std::cout << altitude;
}

void MSPTelemetry::onAnalog(const msp::msg::Analog& analog) { std::cout << analog; }

void MSPTelemetry::onRcTuning(const msp::msg::RcTuning& rc_tuning) {
  //std::cout << rc_tuning;
}

void MSPTelemetry::onPID(const msp::msg::Pid& pid) { std::cout << pid; }

void MSPTelemetry::onBox(const msp::msg::ActiveBoxes& box) { std::cout << box; }

void MSPTelemetry::onMisc(const msp::msg::Misc& misc) { std::cout << misc; }

void MSPTelemetry::onMotorPins(const msp::msg::MotorPins& motor_pins) {
  //std::cout << motor_pins;
}

void MSPTelemetry::onBoxNames(const msp::msg::BoxNames& box_names) {
  //std::cout << box_names;
}

void MSPTelemetry::onPidNames(const msp::msg::PidNames& pid_names) {
  //std::cout << pid_names;
}

void MSPTelemetry::onBoxIds(const msp::msg::BoxIds& box_ids) { std::cout << box_ids; }

void MSPTelemetry::onServoConf(const msp::msg::ServoConf& servo_conf) {
  //std::cout << servo_conf;
}

void MSPTelemetry::onDebugMessage(const msp::msg::DebugMessage& debug_msg) {
  std::cout << "#Debug message:" << std::endl;
  std::cout << debug_msg.debug_msg << std::endl;
}

void MSPTelemetry::onDebug(const msp::msg::Debug& debug) { std::cout << debug; }

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
	    LOG_DEBUG << "Received packet: SYS: " << msg.sysid << " COMP: " << msg.compid
		      << " LEN: " << msg.len << " MSG ID: " << msg.msgid;
	    break;
	  }
	}
      }
    }
  }
}
