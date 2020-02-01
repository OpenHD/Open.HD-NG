#ifndef OPENHDNG_MSP_TELEMETRY_HH
#define OPENHDNG_MSP_TELEMETRY_HH

#include <boost/asio.hpp>

#include <msp/FlightController.hpp>
#include <msp/msp_msg.hpp>

#include <mavlink.h>

#include <shared_queue.hh>

class MSPTelemetry {
public:

  MSPTelemetry(const std::string &device, uint32_t baudrate,
	       const std::string &recv_host, uint16_t recv_port,
	       const std::string &send_host, uint16_t send_port);

  void start();
  void stop();
  void join();

private:
  void onIdent(const msp::msg::Ident& ident);
  void onStatus(const msp::msg::Status& status);
  void onImu(const msp::msg::RawImu& imu_raw);
  void onServo(const msp::msg::Servo& servo);
  void onMotor(const msp::msg::Motor& motor);
  void onRc(const msp::msg::Rc& rc);
  void onAttitude(const msp::msg::Attitude& attitude);
  void onAltitude(const msp::msg::Altitude& altitude);
  void onAnalog(const msp::msg::Analog& analog);
  void onRcTuning(const msp::msg::RcTuning& rc_tuning);
  void onPID(const msp::msg::Pid& pid);
  void onBox(const msp::msg::ActiveBoxes& box);
  void onMisc(const msp::msg::Misc& misc);
  void onMotorPins(const msp::msg::MotorPins& motor_pins);
  void onBoxNames(const msp::msg::BoxNames& box_names);
  void onPidNames(const msp::msg::PidNames& pid_names);
  void onBoxIds(const msp::msg::BoxIds& box_ids);
  void onServoConf(const msp::msg::ServoConf& servo_conf);
  void onDebugMessage(const msp::msg::DebugMessage& debug_msg);
  void onDebug(const msp::msg::Debug& debug);
  void send_message(const mavlink_message_t &msg);
  void udp_send_loop();
  void udp_receive_loop();

  fcu::FlightController m_fcu;
  boost::asio::io_service m_recv_service;
  boost::asio::ip::udp::endpoint m_recv_endpoint;
  boost::asio::ip::udp::socket m_recv_sock;
  SharedQueue<std::shared_ptr<std::vector<uint8_t> > > m_send_queue;
  std::thread *m_send_thread;
  std::thread *m_recv_thread;

  bool m_stop;

  mavlink_heartbeat_t m_heartbeat;
};

#endif /* OPENHDNG_MSP_TELEMETRY_HH */
