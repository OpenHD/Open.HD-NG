
#include <fstream>
#include <chrono>

#include <boost/program_options.hpp>

#include <MSPTelemetry.hh>
#include <logging.hh>

namespace po=boost::program_options;


int main(int argc, char **argv) {

  // Declare a group of options that will be allowed only on command line
  bool help;
  std::string conf_file;
  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", po::bool_switch(&help), "produce help message")
    ("config,c", po::value(&conf_file), "an option configuration file")
    ;
    
  // Declare a group of options that will be allowed both on command line and in config file
  std::string device;
  uint32_t baudrate;
  std::string recvhost;
  uint16_t recvport;
  std::string sendhost;
  uint16_t sendport;
  std::string log_level;
  std::string syslog_level;
  std::string syslog_host;
  po::options_description config("Configuration");
  config.add_options()
    ("loglevel,l", po::value<std::string>(&log_level)->default_value("info"),
     "sets the level for logging debug messages to the console")
    ("sysloglevel", po::value<std::string>(&syslog_level)->default_value("info"),
     "sets the level for logging debug messages to the system log")
    ("sysloghost", po::value<std::string>(&syslog_host)->default_value("info"),
     "sets the host to send syslog messages to")
    ("device,d", po::value<std::string>(&device)->default_value("/dev/ttyS1"),
     "change the UART to connect to (default=/dev/ttyS1)")
    ("baudrate,b", po::value<uint32_t>(&baudrate)->default_value(115200),
     "set the baudrate for the uart (default=115200)")
    ("recvhost", po::value<std::string>(&recvhost)->default_value("127.0.0.1"),
     "receive Mavlink packets from the specified host (default=127.0.0.1)")
    ("recvport", po::value<uint16_t>(&recvport)->default_value(14550),
     "set the UDP port to receive Mavlink packets from (default=14551)")
    ("sendhost", po::value<std::string>(&sendhost)->default_value("127.0.0.1"),
     "send packets to the specified host (default=127.0.0.1)")
    ("sendport", po::value<uint16_t>(&sendport)->default_value(14551),
     "set the UDP port to send the packets to (default=14551)")
    ;

  // Declare the various combinations of options.
  po::options_description cmdline_options("Allowed options");
  cmdline_options.add(generic).add(config);
  po::options_description config_file_options;
  config_file_options.add(config);

  // Parse the options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).options(cmdline_options).run(), vm);
  po::notify(vm);

  // Display the help message if requested.
  if (help) {
    std::cout << "Usage: options_description [options]\n";
    std::cout << cmdline_options;
    return EXIT_SUCCESS;
  }

  // Parse the config file if requestes
  if(!conf_file.empty()) {
    std::ifstream ifs(conf_file.c_str());
    if(ifs.fail()) {
      std::cerr << "Error opening config file: " << conf_file << std::endl;
      return EXIT_FAILURE;
    }
    po::store(po::parse_config_file(ifs, config_file_options), vm);
  }

  // Create the logger
  Logger::create(log_level, syslog_level, syslog_host);
  LOG_INFO << argv[0] << " reading from " << device << " at " << baudrate
	   << " baud and receiving Mavlink from " << recvhost << ":" << recvport
	   << " to " << sendhost << ":" << sendport;
  LOG_INFO << "logging '" << log_level << "' to console and '" << syslog_level << "' to syslog";

  MSPTelemetry telemetry(device, baudrate, recvhost, recvport, sendhost, sendport);
  telemetry.start();
  telemetry.join();

  return 0;
}
