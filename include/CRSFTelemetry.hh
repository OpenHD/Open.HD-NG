#ifndef OPENHDNG_CRSFTELEMETRY_HH
#define OPENHDNG_CRSFTELEMETRY_HH

#include <string>
#include <vector>
#include <map>

#include <boost/asio.hpp>
#include <boost/asio/io_service.hpp>

#include <shared_queue.hh>


// Standard OpenHD stats structures.
typedef struct {
  uint32_t received_packet_cnt;
  int8_t current_signal_dbm;
  int8_t type; // 0 = Atheros, 1 = Ralink
  int8_t signal_good;
} __attribute__((packed)) wifi_adapter_rx_status_forward_t;

typedef struct {
  uint32_t damaged_block_cnt; // number bad blocks video downstream
  uint32_t lost_packet_cnt; // lost packets video downstream
  uint32_t skipped_packet_cnt; // skipped packets video downstream
  uint32_t injection_fail_cnt;  // Video injection failed downstream
  uint32_t received_packet_cnt; // packets received video downstream
  uint32_t kbitrate; // live video kilobitrate per second video downstream
  uint32_t kbitrate_measured; // max measured kbitrate during tx startup
  uint32_t kbitrate_set; // set kilobitrate (measured * bitrate_percent) during tx startup
  uint32_t lost_packet_cnt_telemetry_up; // lost packets telemetry uplink
  uint32_t lost_packet_cnt_telemetry_down; // lost packets telemetry downlink
  uint32_t lost_packet_cnt_msp_up; // lost packets msp uplink (not used at the moment)
  uint32_t lost_packet_cnt_msp_down; // lost packets msp downlink (not used at the moment)
  uint32_t lost_packet_cnt_rc; // lost packets rc link
  int8_t current_signal_joystick_uplink; // signal strength in dbm at air pi (telemetry upstream and rc link)
  int8_t current_signal_telemetry_uplink;
  int8_t joystick_connected; // 0 = no joystick connected, 1 = joystick connected
  float HomeLat;
  float HomeLon;
  uint8_t cpuload_gnd; // CPU load Ground Pi
  uint8_t temp_gnd; // CPU temperature Ground Pi
  uint8_t cpuload_air; // CPU load Air Pi
  uint8_t temp_air; // CPU temperature Air Pi
  uint32_t wifi_adapter_cnt; // number of wifi adapters
  wifi_adapter_rx_status_forward_t adapter[6]; // same struct as in wifibroadcast lib.h
} __attribute__((packed)) wifibroadcast_rx_status_forward_t;


class CRSFTelemetry {
public:
  typedef SharedQueue<std::vector<uint8_t> > BufferQueue;

  CRSFTelemetry(bool mavlink, uint16_t recv_port = 14551, uint16_t send_port = 14550,
                uint16_t status_port = 5800);

  bool get_value(const std::string &name, float &value) const;

  bool armed() const;
  void armed(bool val);

  bool connected() const;

  void set_rc(uint16_t ch1, uint16_t ch2, uint16_t ch3, uint16_t ch4,
	      uint16_t ch5, uint16_t ch6, uint16_t ch7, uint16_t ch8,
	      uint16_t ch9, uint16_t ch10, uint16_t ch11, uint16_t ch12,
	      uint16_t ch13, uint16_t ch14, uint16_t ch15, uint16_t ch16);

  const boost::asio::ip::udp::endpoint &sender_endpoint();

  BufferQueue &send_queue() { return m_send_queue; }

  void send_rc(const std::vector<uint8_t> &msg);

private:
  typedef std::map<std::string, float> NVMap;

  void set_value(const std::string &name, float value);

  void send_link_packet(int8_t rx_rssi1, int8_t rx_rssi2,
			uint8_t rx_quality, uint8_t rx_snr,
			uint8_t rx_antenna, uint8_t rf_mode,
			uint8_t tx_power, int8_t tx_rssi,
			uint8_t tx_quality, uint8_t tx_snr);
  void send_attitude_packet(float yaw_deg, float pitch_deg, float roll_deg);
  void send_battery_packet(float v, float c, uint32_t cap, float remain);
  void send_gps_packet(int32_t lat, int32_t lon, uint16_t vel,
		       uint16_t heading, uint16_t alt, uint8_t nsat);

  void reader_thread(uint16_t port = 14551, bool mavlink = false);
  void status_thread(uint16_t port = 5800);
  void control_thread();

  //boost::asio::io_service m_send_service;
  //boost::asio::ip::udp::socket m_send_sock;
  bool m_mavlink;
  NVMap m_values;
  uint8_t m_sysid;
  uint8_t m_compid;
  bool m_rec_bat_status;
  bool m_connected;
  bool m_rc_fresh;
  wifibroadcast_rx_status_forward_t m_link_stats;
  mavlink_rc_channels_override_t m_mavlink_rc_override;
  BufferQueue m_send_queue;
  boost::asio::io_service m_send_service;
  boost::asio::ip::udp::socket m_send_sock;
  boost::asio::ip::udp::endpoint m_sender_endpoint;
};

#endif /* OPENHDNG_CRSFTELEMETRY_HH */
