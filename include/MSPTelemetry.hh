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
  void onStatus(const msp::msg::Status& status);
  void onAttitude(const msp::msg::Attitude& attitude);
  void onAnalog(const msp::msg::Analog& analog);
  void onRawGPS(const msp::msg::RawGPS& gps);

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
