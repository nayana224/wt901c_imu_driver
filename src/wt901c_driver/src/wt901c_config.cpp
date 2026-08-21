#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "serial/serial.h"

namespace
{
constexpr uint8_t kReadRegister = 0x27;
constexpr uint8_t kUnlockRegister = 0x69;
constexpr uint16_t kUnlockValue = 0xB588;
constexpr uint8_t kSaveRegister = 0x00;
constexpr uint8_t kCalibrationRegister = 0x01;
constexpr uint8_t kOutputContentRegister = 0x02;
constexpr uint8_t kOutputRateRegister = 0x03;
constexpr uint8_t kBaudRegister = 0x04;
constexpr uint8_t kLedOffRegister = 0x1B;
constexpr uint8_t kBandwidthRegister = 0x1F;
constexpr uint8_t kInstallationDirectionRegister = 0x23;
constexpr uint8_t kAlgorithmRegister = 0x24;
constexpr uint8_t kPowerOnOutputRegister = 0x2D;
constexpr uint8_t kVersionRegister = 0x2E;
constexpr uint8_t kReadReplyType = 0x5F;
constexpr std::size_t kReadReplySize = 11;
constexpr auto kCommandDelay = std::chrono::milliseconds(20);
constexpr auto kBaudSwitchDelay = std::chrono::milliseconds(500);
constexpr auto kReadTimeout = std::chrono::milliseconds(350);

struct Options
{
  std::string port{"/dev/ttyUSB0"};
  int baudrate{115200};
  int timeout_ms{50};
  bool dry_run{false};
  std::vector<std::string> positional;
};

const std::map<int, uint16_t> kBaudToCode{
  {4800, 0x01}, {9600, 0x02}, {19200, 0x03}, {38400, 0x04},
  {57600, 0x05}, {115200, 0x06}, {230400, 0x07}};

const std::map<uint16_t, int> kCodeToBaud{
  {0x01, 4800}, {0x02, 9600}, {0x03, 19200}, {0x04, 38400},
  {0x05, 57600}, {0x06, 115200}, {0x07, 230400}};

const std::map<std::string, uint16_t> kRateToCode{
  {"0.2", 0x01}, {"0.5", 0x02}, {"1", 0x03}, {"2", 0x04},
  {"5", 0x05}, {"10", 0x06}, {"20", 0x07}, {"50", 0x08},
  {"100", 0x09}, {"200", 0x0B}};

const std::map<uint16_t, std::string> kCodeToRate{
  {0x01, "0.2"}, {0x02, "0.5"}, {0x03, "1"}, {0x04, "2"},
  {0x05, "5"}, {0x06, "10"}, {0x07, "20"}, {0x08, "50"},
  {0x09, "100"}, {0x0B, "200"}, {0x0C, "single"}, {0x0D, "none"}};

const std::map<std::string, uint16_t> kBandwidthToCode{
  {"256", 0x00}, {"188", 0x01}, {"98", 0x02}, {"42", 0x03},
  {"20", 0x04}, {"10", 0x05}, {"5", 0x06}};

const std::map<uint16_t, std::string> kCodeToBandwidth{
  {0x00, "256"}, {0x01, "188"}, {0x02, "98"}, {0x03, "42"},
  {0x04, "20"}, {0x05, "10"}, {0x06, "5"}};

const std::map<std::string, uint16_t> kOutputBits{
  {"time", 1u << 0}, {"accel", 1u << 1}, {"gyro", 1u << 2},
  {"angle", 1u << 3}, {"mag", 1u << 4}, {"port", 1u << 5},
  {"pressure", 1u << 6}, {"gps", 1u << 7}, {"velocity", 1u << 8},
  {"quaternion", 1u << 9}, {"gsa", 1u << 10}};

std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> parts;
  std::stringstream stream(text);
  std::string item;
  while (std::getline(stream, item, delimiter)) {
    if (!item.empty()) {
      parts.push_back(item);
    }
  }
  return parts;
}

std::string hex_byte(uint8_t value)
{
  std::ostringstream stream;
  stream << std::uppercase << std::hex << std::setfill('0') << std::setw(2)
         << static_cast<int>(value);
  return stream.str();
}

std::string hex_word(uint16_t value)
{
  std::ostringstream stream;
  stream << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(4)
         << value;
  return stream.str();
}

std::vector<uint8_t> make_packet(uint8_t reg, uint16_t value)
{
  return {
    0xFF,
    0xAA,
    reg,
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF)};
}

void print_packet(const std::vector<uint8_t> & packet, const std::string & prefix = "TX")
{
  std::cout << prefix << ":";
  for (const auto byte : packet) {
    std::cout << ' ' << hex_byte(byte);
  }
  std::cout << '\n';
}

bool checksum_valid(const std::array<uint8_t, kReadReplySize> & frame)
{
  uint8_t sum = 0;
  for (std::size_t index = 0; index < frame.size() - 1; ++index) {
    sum = static_cast<uint8_t>(sum + frame[index]);
  }
  return sum == frame.back();
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--port") {
      if (++index >= argc) {
        throw std::invalid_argument("--port requires a value");
      }
      options.port = argv[index];
    } else if (argument == "--baudrate") {
      if (++index >= argc) {
        throw std::invalid_argument("--baudrate requires a value");
      }
      options.baudrate = std::stoi(argv[index]);
    } else if (argument == "--timeout-ms") {
      if (++index >= argc) {
        throw std::invalid_argument("--timeout-ms requires a value");
      }
      options.timeout_ms = std::stoi(argv[index]);
    } else if (argument == "--dry-run") {
      options.dry_run = true;
    } else {
      options.positional.push_back(argument);
    }
  }
  return options;
}

void print_help()
{
  std::cout
    << "WT901C device configuration tool\n\n"
    << "Usage:\n"
    << "  wt901c_config [--port DEV] [--baudrate CURRENT] COMMAND [ARGS]\n\n"
    << "Read commands:\n"
    << "  detect-baud\n"
    << "  status\n"
    << "  read <baudrate|output-rate|output-content|bandwidth|install-direction|led|algorithm|power-on-output|version|0xNN>\n\n"
    << "Write commands:\n"
    << "  baudrate <4800|9600|19200|38400|57600|115200|230400>\n"
    << "  output-rate <0.2|0.5|1|2|5|10|20|50|100|200>\n"
    << "  output-content <comma-separated names>\n"
    << "  bandwidth <256|188|98|42|20|10|5>\n"
    << "  heading-zero\n"
    << "  install-direction <horizontal|vertical>\n"
    << "  led <on|off>\n"
    << "  mag-calibration <start|stop>\n"
    << "  save\n\n"
    << "Output-content names:\n"
    << "  time,accel,gyro,angle,mag,port,pressure,gps,velocity,quaternion,gsa\n\n"
    << "Safety:\n"
    << "  Stop imu_driver before running this tool so only one process owns the serial port.\n"
    << "  Use --dry-run to inspect write packets without opening the serial device.\n";
}

std::unique_ptr<serial::Serial> open_serial(const Options & options, int baudrate)
{
  auto port = std::make_unique<serial::Serial>(
    options.port,
    static_cast<uint32_t>(baudrate),
    serial::Timeout::simpleTimeout(static_cast<uint32_t>(options.timeout_ms)));
  if (!port->isOpen()) {
    throw std::runtime_error("failed to open " + options.port);
  }
  return port;
}

void drain_input(serial::Serial & port)
{
  for (int iteration = 0; iteration < 5; ++iteration) {
    const std::size_t available = port.available();
    if (available == 0) {
      return;
    }
    std::vector<uint8_t> bytes(std::min<std::size_t>(available, 512));
    port.read(bytes.data(), bytes.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void write_packet(serial::Serial & port, uint8_t reg, uint16_t value)
{
  const auto packet = make_packet(reg, value);
  const std::size_t written = port.write(packet);
  if (written != packet.size()) {
    throw std::runtime_error("short serial write");
  }
  std::this_thread::sleep_for(kCommandDelay);
}

void dry_write(uint8_t reg, uint16_t value)
{
  print_packet(make_packet(reg, value), "[dry-run] TX");
}

void unlock(serial::Serial & port)
{
  write_packet(port, kUnlockRegister, kUnlockValue);
}

void save(serial::Serial & port)
{
  unlock(port);
  write_packet(port, kSaveRegister, 0x0000);
}

void write_and_save(serial::Serial & port, uint8_t reg, uint16_t value)
{
  unlock(port);
  write_packet(port, reg, value);
  save(port);
}

void dry_write_and_save(uint8_t reg, uint16_t value)
{
  dry_write(kUnlockRegister, kUnlockValue);
  dry_write(reg, value);
  dry_write(kUnlockRegister, kUnlockValue);
  dry_write(kSaveRegister, 0x0000);
}

bool read_reply_value(serial::Serial & port, uint16_t & value)
{
  std::vector<uint8_t> buffer;
  const auto deadline = std::chrono::steady_clock::now() + kReadTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    const std::size_t available = port.available();
    if (available > 0) {
      std::vector<uint8_t> bytes(std::min<std::size_t>(available, 256));
      const std::size_t count = port.read(bytes.data(), bytes.size());
      buffer.insert(buffer.end(), bytes.begin(), bytes.begin() + count);
    }

    while (buffer.size() >= kReadReplySize) {
      auto header = std::find(buffer.begin(), buffer.end(), static_cast<uint8_t>(0x55));
      if (header == buffer.end()) {
        buffer.clear();
        break;
      }
      if (header != buffer.begin()) {
        buffer.erase(buffer.begin(), header);
      }
      if (buffer.size() < kReadReplySize) {
        break;
      }
      if (buffer[1] != kReadReplyType) {
        buffer.erase(buffer.begin());
        continue;
      }

      std::array<uint8_t, kReadReplySize> frame{};
      std::copy_n(buffer.begin(), kReadReplySize, frame.begin());
      if (!checksum_valid(frame)) {
        buffer.erase(buffer.begin());
        continue;
      }

      value = static_cast<uint16_t>(frame[2]) |
        static_cast<uint16_t>(static_cast<uint16_t>(frame[3]) << 8);
      return true;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

bool read_register(serial::Serial & port, uint8_t reg, uint16_t & value)
{
  drain_input(port);
  const auto request = make_packet(kReadRegister, reg);
  const std::size_t written = port.write(request);
  if (written != request.size()) {
    throw std::runtime_error("short serial write while reading register");
  }
  return read_reply_value(port, value);
}

uint8_t register_from_name(const std::string & name)
{
  static const std::map<std::string, uint8_t> registers{
    {"output-content", kOutputContentRegister},
    {"output-rate", kOutputRateRegister},
    {"baudrate", kBaudRegister},
    {"led", kLedOffRegister},
    {"bandwidth", kBandwidthRegister},
    {"install-direction", kInstallationDirectionRegister},
    {"algorithm", kAlgorithmRegister},
    {"power-on-output", kPowerOnOutputRegister},
    {"version", kVersionRegister}};

  const auto found = registers.find(name);
  if (found != registers.end()) {
    return found->second;
  }

  std::size_t parsed = 0;
  const int value = std::stoi(name, &parsed, 0);
  if (parsed != name.size() || value < 0 || value > 0xFF) {
    throw std::invalid_argument("unknown register name: " + name);
  }
  return static_cast<uint8_t>(value);
}

std::string decode_output_content(uint16_t mask)
{
  std::vector<std::string> enabled;
  for (const auto & [name, bit] : kOutputBits) {
    if ((mask & bit) != 0) {
      enabled.push_back(name);
    }
  }
  std::ostringstream stream;
  for (std::size_t index = 0; index < enabled.size(); ++index) {
    if (index > 0) {
      stream << ',';
    }
    stream << enabled[index];
  }
  return stream.str();
}

std::string decode_register(uint8_t reg, uint16_t value)
{
  if (reg == kBaudRegister) {
    const auto found = kCodeToBaud.find(value & 0x0F);
    return found == kCodeToBaud.end() ? "unknown" : std::to_string(found->second);
  }
  if (reg == kOutputRateRegister) {
    const auto found = kCodeToRate.find(value & 0x0F);
    return found == kCodeToRate.end() ? "unknown" : found->second;
  }
  if (reg == kOutputContentRegister) {
    return decode_output_content(value);
  }
  if (reg == kBandwidthRegister) {
    const auto found = kCodeToBandwidth.find(value & 0x0F);
    return found == kCodeToBandwidth.end() ? "unknown" : found->second;
  }
  if (reg == kInstallationDirectionRegister) {
    return (value & 0x01) == 0 ? "horizontal" : "vertical";
  }
  if (reg == kLedOffRegister) {
    return (value & 0x01) == 0 ? "on" : "off";
  }
  if (reg == kAlgorithmRegister) {
    return (value & 0x01) == 0 ? "9-axis" : "6-axis";
  }
  if (reg == kPowerOnOutputRegister) {
    return (value & 0x01) == 0 ? "off" : "on";
  }
  return std::to_string(value);
}

void print_register_value(uint8_t reg, uint16_t value)
{
  std::cout << "register=0x" << hex_byte(reg) << '\n';
  std::cout << "raw=" << hex_word(value) << '\n';
  std::cout << "decoded=" << decode_register(reg, value) << '\n';
}

uint16_t output_mask_from_csv(const std::string & csv)
{
  uint16_t mask = 0;
  const auto names = split(csv, ',');
  if (names.empty()) {
    throw std::invalid_argument("output-content requires at least one packet name");
  }
  for (const auto & name : names) {
    const auto found = kOutputBits.find(name);
    if (found == kOutputBits.end()) {
      throw std::invalid_argument("unknown output-content name: " + name);
    }
    mask = static_cast<uint16_t>(mask | found->second);
  }
  return mask;
}

void print_status(serial::Serial & port, int connected_baud)
{
  const std::vector<std::pair<std::string, uint8_t>> fields{
    {"output_content", kOutputContentRegister},
    {"output_rate_hz", kOutputRateRegister},
    {"baudrate", kBaudRegister},
    {"led", kLedOffRegister},
    {"bandwidth_hz", kBandwidthRegister},
    {"installation_direction", kInstallationDirectionRegister},
    {"algorithm", kAlgorithmRegister},
    {"power_on_output", kPowerOnOutputRegister},
    {"version", kVersionRegister}};

  std::cout << "connected_baudrate=" << connected_baud << '\n';
  for (const auto & [label, reg] : fields) {
    uint16_t value = 0;
    if (!read_register(port, reg, value)) {
      std::cout << label << "=READ_TIMEOUT\n";
      continue;
    }
    if (reg == kOutputContentRegister) {
      std::cout << "output_content_mask=" << hex_word(value) << '\n';
    }
    std::cout << label << '=' << decode_register(reg, value) << '\n';
  }
}

bool probe_baud(const Options & options, int baudrate, uint16_t & version)
{
  try {
    auto port = open_serial(options, baudrate);
    return read_register(*port, kVersionRegister, version);
  } catch (const std::exception &) {
    return false;
  }
}

int detect_baud(const Options & options)
{
  const std::vector<int> candidates{115200, 9600, 57600, 38400, 19200, 230400, 4800};
  for (const int candidate : candidates) {
    std::cout << "probing=" << candidate << '\n';
    uint16_t version = 0;
    if (probe_baud(options, candidate, version)) {
      std::cout << "detected_baudrate=" << candidate << '\n';
      std::cout << "version=" << version << '\n';
      return candidate;
    }
  }
  throw std::runtime_error("WT901C did not respond at any supported baudrate");
}

void require_argument_count(const Options & options, std::size_t count, const std::string & usage)
{
  if (options.positional.size() != count) {
    throw std::invalid_argument("usage: " + usage);
  }
}

int run_read_command(const Options & options)
{
  require_argument_count(options, 2, "wt901c_config read <register-name|0xNN>");
  if (options.dry_run) {
    throw std::invalid_argument("--dry-run is only valid for write commands");
  }
  auto port = open_serial(options, options.baudrate);
  const uint8_t reg = register_from_name(options.positional[1]);
  uint16_t value = 0;
  if (!read_register(*port, reg, value)) {
    throw std::runtime_error("register read timed out; check current baudrate and serial ownership");
  }
  print_register_value(reg, value);
  return 0;
}

int run_write_command(const Options & options)
{
  const std::string & command = options.positional.front();

  if (command == "baudrate") {
    require_argument_count(options, 2, "wt901c_config baudrate <rate>");
    const int new_baud = std::stoi(options.positional[1]);
    const auto found = kBaudToCode.find(new_baud);
    if (found == kBaudToCode.end()) {
      throw std::invalid_argument("unsupported baudrate");
    }
    if (options.dry_run) {
      std::cout << "[dry-run] current_baudrate=" << options.baudrate << '\n';
      dry_write(kUnlockRegister, kUnlockValue);
      dry_write(kBaudRegister, found->second);
      std::cout << "[dry-run] reopen host serial at " << new_baud << " baud\n";
      dry_write(kUnlockRegister, kUnlockValue);
      dry_write(kSaveRegister, 0x0000);
      return 0;
    }

    auto port = open_serial(options, options.baudrate);
    unlock(*port);
    write_packet(*port, kBaudRegister, found->second);
    port->close();
    std::this_thread::sleep_for(kBaudSwitchDelay);

    port = open_serial(options, new_baud);
    save(*port);
    std::cout << "baudrate=" << new_baud << '\n';
    return 0;
  }

  if (command == "output-rate") {
    require_argument_count(options, 2, "wt901c_config output-rate <hz>");
    const auto found = kRateToCode.find(options.positional[1]);
    if (found == kRateToCode.end()) {
      throw std::invalid_argument("unsupported output rate");
    }
    if (options.dry_run) {
      dry_write_and_save(kOutputRateRegister, found->second);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kOutputRateRegister, found->second);
    }
    std::cout << "output_rate_hz=" << options.positional[1] << '\n';
    return 0;
  }

  if (command == "output-content") {
    require_argument_count(options, 2, "wt901c_config output-content <csv>");
    const uint16_t mask = output_mask_from_csv(options.positional[1]);
    if (options.dry_run) {
      dry_write_and_save(kOutputContentRegister, mask);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kOutputContentRegister, mask);
    }
    std::cout << "output_content_mask=" << hex_word(mask) << '\n';
    std::cout << "output_content=" << decode_output_content(mask) << '\n';
    return 0;
  }

  if (command == "bandwidth") {
    require_argument_count(options, 2, "wt901c_config bandwidth <hz>");
    const auto found = kBandwidthToCode.find(options.positional[1]);
    if (found == kBandwidthToCode.end()) {
      throw std::invalid_argument("unsupported bandwidth");
    }
    if (options.dry_run) {
      dry_write_and_save(kBandwidthRegister, found->second);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kBandwidthRegister, found->second);
    }
    std::cout << "bandwidth_hz=" << options.positional[1] << '\n';
    return 0;
  }

  if (command == "heading-zero") {
    require_argument_count(options, 1, "wt901c_config heading-zero");
    if (options.dry_run) {
      dry_write_and_save(kCalibrationRegister, 0x0004);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kCalibrationRegister, 0x0004);
    }
    std::cout << "heading_zero=applied\n";
    return 0;
  }

  if (command == "install-direction") {
    require_argument_count(options, 2, "wt901c_config install-direction <horizontal|vertical>");
    uint16_t value = 0;
    if (options.positional[1] == "horizontal") {
      value = 0;
    } else if (options.positional[1] == "vertical") {
      value = 1;
    } else {
      throw std::invalid_argument("installation direction must be horizontal or vertical");
    }
    if (options.dry_run) {
      dry_write_and_save(kInstallationDirectionRegister, value);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kInstallationDirectionRegister, value);
    }
    std::cout << "installation_direction=" << options.positional[1] << '\n';
    return 0;
  }

  if (command == "led") {
    require_argument_count(options, 2, "wt901c_config led <on|off>");
    uint16_t value = 0;
    if (options.positional[1] == "on") {
      value = 0;
    } else if (options.positional[1] == "off") {
      value = 1;
    } else {
      throw std::invalid_argument("LED value must be on or off");
    }
    if (options.dry_run) {
      dry_write_and_save(kLedOffRegister, value);
    } else {
      auto port = open_serial(options, options.baudrate);
      write_and_save(*port, kLedOffRegister, value);
    }
    std::cout << "led=" << options.positional[1] << '\n';
    return 0;
  }

  if (command == "mag-calibration") {
    require_argument_count(options, 2, "wt901c_config mag-calibration <start|stop>");
    const std::string mode = options.positional[1];
    if (mode != "start" && mode != "stop") {
      throw std::invalid_argument("mag-calibration mode must be start or stop");
    }
    const uint16_t value = mode == "start" ? 0x0007 : 0x0000;
    if (options.dry_run) {
      dry_write(kUnlockRegister, kUnlockValue);
      dry_write(kCalibrationRegister, value);
      if (mode == "stop") {
        dry_write(kUnlockRegister, kUnlockValue);
        dry_write(kSaveRegister, 0x0000);
      }
    } else {
      auto port = open_serial(options, options.baudrate);
      unlock(*port);
      write_packet(*port, kCalibrationRegister, value);
      if (mode == "stop") {
        save(*port);
      }
    }
    std::cout << "mag_calibration=" << mode << '\n';
    return 0;
  }

  if (command == "save") {
    require_argument_count(options, 1, "wt901c_config save");
    if (options.dry_run) {
      dry_write(kUnlockRegister, kUnlockValue);
      dry_write(kSaveRegister, 0x0000);
    } else {
      auto port = open_serial(options, options.baudrate);
      save(*port);
    }
    std::cout << "saved=true\n";
    return 0;
  }

  throw std::invalid_argument("unknown command: " + command);
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parse_options(argc, argv);
    if (options.positional.empty() || options.positional.front() == "--help" ||
      options.positional.front() == "help")
    {
      print_help();
      return 0;
    }

    const std::string & command = options.positional.front();
    if (command == "detect-baud") {
      require_argument_count(options, 1, "wt901c_config detect-baud");
      if (options.dry_run) {
        throw std::invalid_argument("--dry-run is not valid with detect-baud");
      }
      detect_baud(options);
      return 0;
    }

    if (command == "status") {
      require_argument_count(options, 1, "wt901c_config status");
      if (options.dry_run) {
        throw std::invalid_argument("--dry-run is not valid with status");
      }
      auto port = open_serial(options, options.baudrate);
      print_status(*port, options.baudrate);
      return 0;
    }

    if (command == "read") {
      return run_read_command(options);
    }

    return run_write_command(options);
  } catch (const std::exception & error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
  }
}
