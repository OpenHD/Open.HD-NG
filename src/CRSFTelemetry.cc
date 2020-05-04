
#include <sys/time.h>

#include <iostream>
#include <thread>

#include <mavlink.h>

#include <CRSFTelemetry.hh>
#include <logging.hh>

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
  buf.push_back(CRSF_RADIO_ADDRESS_TX);
  buf.push_back(12); // type (1) + Payload size + CRC size (1)
  buf.push_back(CRSF_LINK);
  rx_rssi1 = -20;
  rx_rssi2 = -25;
  rx_quality = 99;
  rx_snr = 5;
  rx_antenna = 0;
  rf_mode = 1;
  tx_power = 3;
  tx_rssi = -40;
  tx_quality = 89;
  tx_snr = 3.4;
  LOG_DEBUG << "Link: " << int(rx_rssi1) << " " << int(rx_rssi2) << " " << int(rx_quality) << " "
            << int(rx_snr) << " " << int(rx_antenna) << " " << int(rf_mode) << " "
            << int(tx_power) << " " << int(tx_rssi) << " " << tx_snr;
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
  buf.push_back(CRSF_RADIO_ADDRESS_TX);
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
  buf.push_back(CRSF_RADIO_ADDRESS_TX);
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
  buf.push_back(CRSF_RADIO_ADDRESS_TX);
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

void CRSFTelemetry::status_thread(uint16_t port) {
  boost::asio::io_service status_service;
  boost::asio::ip::udp::socket status_sock
    (status_service, boost::asio::ip::udp::endpoint(boost::asio::ip::address_v4::any(), port));
  boost::asio::ip::udp::endpoint sender_endpoint;

  // Ensure we can receive broadcast messages.
  status_sock.set_option(boost::asio::socket_base::broadcast(true));

  double last_recv = 0;
  double last_send = 0;
  wifibroadcast_rx_status_forward_t prev_link_stats;
  double rx_quality = 0;
  float link_timeout = 5.0;
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
    if ((tdiff < 0) || (tdiff > link_timeout)) {
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
    // Don't send status if we haven't received any packets in a while.
    if (cur_npkt != 0 && ((cur_time() - m_last_telemetry_packet_time) < 1.0)) {
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
    }
    prev_link_stats = m_link_stats;
  }
}
