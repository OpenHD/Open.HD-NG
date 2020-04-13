
#include <exception>

#include <wifibroadcast/transfer_stats.hh>

#include <logging.hh>
#include <MSPTelemetry.hh>

#define MAVLINK_MAX_MSG 2048

MSPTelemetry::MSPTelemetry(const std::string &device, uint32_t baudrate,
                           const std::string &send_host, uint16_t send_port,
			   uint16_t recv_port, uint8_t sysid, uint8_t compid) :
  m_recv_endpoint(boost::asio::ip::address_v4::any(), recv_port),
  m_recv_sock(m_io_service, m_recv_endpoint),
  m_send_thread(0), m_recv_thread(0),
  m_stop(false), m_sysid(sysid), m_compid(compid) {

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

void MSPTelemetry::send_message(const mavlink_message_t &msg) {
  size_t msg_len = mavlink_msg_get_send_buffer_length(&msg);
  std::shared_ptr<std::vector<uint8_t> > buf(new std::vector<uint8_t>(msg_len));
  mavlink_msg_to_send_buffer(buf->data(), &msg);
  m_send_queue.push(buf);
}

void MSPTelemetry::udp_send_loop() {
  bool done = false;
  boost::asio::io_service io_service;
  boost::asio::ip::udp::socket send_socket(io_service);
  send_socket.open(boost::asio::ip::udp::v4());
  send_socket.set_option(boost::asio::socket_base::broadcast(true));
  boost::asio::ip::udp::endpoint send_endpoint(boost::asio::ip::address_v4::any(), 14650);

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
      LOG_DEBUG << "recv: " << recv;
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
	  case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
	    LOG_DEBUG << "MAVLINK_MSG_PARAM_REQUEST_READ";
	    mavlink_param_request_read_t param_read_msg;
	    mavlink_msg_param_request_read_decode(&msg, &param_read_msg);
            if ((param_read_msg.target_system == m_sysid) &&
                (param_read_msg.target_component == m_compid)) {
              if (!strncmp(param_read_msg.param_id, "SYSID_MYGCS",
                           MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN)) {
                mavlink_param_value_t param_value;
                param_value.param_value = 255;
                param_value.param_count = 1;
                param_value.param_index = 0;
                memcpy(param_value.param_id, param_read_msg.param_id,
                       MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
                param_value.param_type = 0;
                mavlink_message_t msg_pv;
                mavlink_msg_param_value_encode(m_sysid, m_compid, &msg_pv, &param_value);
                send_message(msg_pv);
              }
            }
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
