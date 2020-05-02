
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <time.h>

#include <iostream>
#include <thread>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time.hpp>
#include <boost/program_options.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/bind.hpp>

#include <mavlink.h>

#include <CRSFTelemetry.hh>
#include <logging.hh>

#define CRSF_DEVICE_TYPE 0xEE
#define CRSF_FRAME_LEN 0x18
#define CRSF_FRAME_TYPE 0x16
#define CRSF_TELEMETRY_CRC_SIZE 2

typedef enum {
	      DEVICE_ADDRESS,
	      FRAME_LENGTH,
	      FRAME_TYPE,
	      PAYLOAD,
	      CRC
} CRSFStates;

struct CRSFChannels {
  uint16_t ch0 : 11;
  uint16_t ch1 : 11;
  uint16_t ch2 : 11;
  uint16_t ch3 : 11;
  uint16_t ch4 : 11;
  uint16_t ch5 : 11;
  uint16_t ch6 : 11;
  uint16_t ch7 : 11;
  uint16_t ch8 : 11;
  uint16_t ch9 : 11;
  uint16_t ch10 : 11;
  uint16_t ch11 : 11;
  uint16_t ch12 : 11;
  uint16_t ch13 : 11;
  uint16_t ch14 : 11;
  uint16_t ch15 : 11;
} __attribute__ ((__packed__));

double cur_time() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return (double(t.tv_sec) + double(t.tv_usec) * 1e-6);
}

uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) {
  crc ^= a;
  for (int ii = 0; ii < 8; ++ii) {
    if (crc & 0x80) {
      crc = (crc << 1) ^ 0xD5;
    } else {
      crc = crc << 1;
    }
  }
  return crc;
}

uint8_t crsf_crc(const uint8_t *buf, uint8_t len, uint8_t init = CRSF_FRAME_TYPE) {
  uint8_t crc = crc8_dvb_s2(0, init);
  for(size_t i = 0; i < len; ++i) {
    crc = crc8_dvb_s2(crc, buf[i]);
  }
  return crc;
}


int main(int argc, char **argv) {
  namespace po=boost::program_options;

  // Declare a group of options that will be allowed only on command line
  bool help;
  std::string conf_file;
  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "produce help message")
    ("config,c", po::value(&conf_file), "an option configuration file")
    ;
    
  // Declare a group of options that will be allowed both on command line and in config file
  std::string device;
  uint32_t baudrate;
  uint16_t recv_port;
  uint16_t send_port;
  uint16_t status_port;
  std::string log_level;
  std::string syslog_level;
  std::string syslog_host;
  bool mavlink;
  po::options_description config("Allowed options");
  config.add_options()
    ("loglevel,l", po::value<std::string>(&log_level)->default_value("info"),
     "sets the level for logging debug messages to the console")
    ("sysloglevel", po::value<std::string>(&syslog_level)->default_value("info"),
     "sets the level for logging debug messages to the system log")
    ("sysloghost", po::value<std::string>(&syslog_host)->default_value("info"),
     "sets the host to send syslog messages to")
    ("mavlink,m", po::value<bool>(&mavlink),
     "translate the Crosssfire RC packets to Mavlink RC override packets")
    ("device,d", po::value<std::string>(&device)->default_value("/dev/ttyAMA0"),
     "change the UART to connect to (default=/dev/ttyS1)")
    ("baudrate,b", po::value<uint32_t>(&baudrate)->default_value(115200),
     "set the baudrate for the uart (default=115200)")
    ("statusport", po::value<uint16_t>(&status_port)->default_value(5800),
     "set the UDP port to receive status packets to (default=5800)")
    ("recvport", po::value<uint16_t>(&recv_port)->default_value(14650),
     "set the UDP port to receive telemetry packets to (default=14650)")
    ("sendport", po::value<uint16_t>(&send_port)->default_value(14651),
     "set the UDP port to send the packets to (default=14651)")
    ;

  // Declare the various combinations of options.
  po::options_description cmdline_options("Allowed options");
  cmdline_options.add(generic).add(config);
  po::options_description config_file_options;
  config_file_options.add(config);

  // Parse the options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(cmdline_options).run(), vm);

  // Display the help message if requested.
  if (vm.count("help")) {
    std::cout << "Usage: options_description [options]\n";
    std::cout << cmdline_options;
    return EXIT_SUCCESS;
  }

  // Parse the config file if requests
  if(vm.count("config")) {
    std::ifstream ifs(vm["config"].as<std::string>().c_str());
    if(ifs.fail()) {
      std::cerr << "Error opening config file: " << conf_file << std::endl;
      return EXIT_FAILURE;
    }
    po::store(po::parse_config_file(ifs, config_file_options), vm);
  }
  po::notify(vm);

  // Create the logger
  Logger::create(log_level, syslog_level, syslog_host);
  LOG_INFO << argv[0] << " reading from " << device << " at " << baudrate;
  LOG_INFO << "logging '" << log_level << "' to console and '" << syslog_level << "' to syslog";

  // Create the telemetry class for sending and receiving Mavlink messages
  CRSFTelemetry telemetry(mavlink, recv_port, send_port, status_port);

  // Open the UART
  boost::asio::io_service io_service;
  boost::asio::serial_port uart(io_service);
  uart.open(device);
  uart.set_option(boost::asio::serial_port_base::baud_rate(baudrate));

  // Spawn the UART read thread
  std::thread recv_thr
    ([&uart, &telemetry, mavlink]() {

       // Parse the Crossfire receiver messages
       uint8_t stage = DEVICE_ADDRESS;
       bool reset = false;
       CRSFChannels channels;
       uint8_t *cbuf = reinterpret_cast<uint8_t*>(&channels);
       uint8_t idx = 0;
       double prev_time = cur_time();
       std::vector<uint8_t> obuf;
       uint32_t CRC_errors = 0;
       uint32_t valid_packets = 0;
       while(1) {

         // Read a single byte from the UART
         uint8_t c;
         uint8_t count = boost::asio::read(uart, boost::asio::buffer(&c,1));
         if (count <= 0) {
           continue;
         }

         // Output the packet verbatim if we're not translating to mavlink
         if (!mavlink) {
           obuf.push_back(c);
         }

         // Parse the frame
         switch(stage) {
         case DEVICE_ADDRESS:
           // The first byte of  a frame is the device type
           if (c == CRSF_DEVICE_TYPE) {
             stage = FRAME_LENGTH;
           } else {
             obuf.clear();
           }
           break;
         case FRAME_LENGTH:
           // The second byte is the frame length
           if (c == CRSF_FRAME_LEN) {
             stage = FRAME_TYPE;
           } else {
             obuf.clear();
             reset = true;
           }
           break;
         case FRAME_TYPE:
           // The frame type is the third byte
           if (c == CRSF_FRAME_TYPE) {
             stage = PAYLOAD;
           } else {
             obuf.clear();
             reset = true;
           }
           break;
         case PAYLOAD:
           // Extract the bits into the payload buffer
           cbuf[idx++] = c;
           if(idx == sizeof(channels)) {
             stage = CRC;
           }
           break;
         case CRC:
           uint8_t crc = crsf_crc(cbuf, sizeof(channels)); 
           if (crc != c) {
             ++CRC_errors;
           } else {
             float scale = 0.62477120195241;
             float offset = 880.53935326418548;
             ++valid_packets;
             if (mavlink) {
               telemetry.set_rc(offset + channels.ch0 * scale,
                                offset + channels.ch1 * scale,
                                offset + channels.ch2 * scale,
                                offset + channels.ch3 * scale,
                                offset + channels.ch4 * scale,
                                offset + channels.ch5 * scale,
                                offset + channels.ch6 * scale,
                                offset + channels.ch7 * scale,
                                offset + channels.ch8 * scale,
                                offset + channels.ch9 * scale,
                                offset + channels.ch10 * scale,
                                offset + channels.ch11 * scale,
                                offset + channels.ch12 * scale,
                                offset + channels.ch13 * scale,
                                offset + channels.ch14 * scale,
                                offset + channels.ch15 * scale);
             } else {
               telemetry.send_rc(obuf);
             }
           }

           obuf.clear();
           reset = true;
           break;
         }

         if ((cur_time() - prev_time) > 5) {
           LOG_INFO << "RC packet stats " << valid_packets << "/" << CRC_errors
                    << " (" << 100.0 * static_cast<float>(valid_packets) /
             static_cast<float>(valid_packets + CRC_errors) << "%)";
           prev_time = cur_time();
         }

         // Reset if requested
         if (reset) {
           stage = DEVICE_ADDRESS;
           idx = 0;
           reset = false;
         }
       }
     });

  // Create a thread for relaying telemetry to the transmitter
  std::thread telem_thr
    ([&uart, &telemetry]() {

       // Send telemetry to the Tx
       while (true) {
         auto out_buf = telemetry.send_queue().pop();
         LOG_DEBUG << "Sending to Tx: " << out_buf.size();
         boost::asio::write(uart, boost::asio::buffer(out_buf.data(), out_buf.size()));
       }
     });

  // Cleanup
  recv_thr.join();
  telem_thr.join();
  uart.close();
}
