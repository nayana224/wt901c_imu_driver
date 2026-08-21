#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "serial/serial.h"

namespace
{
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
constexpr auto kCommandDelay = std::chrono::milliseconds(20);
constexpr auto kBaudSwitchDelay = std::chrono::milliseconds(500);

struct Options
{
  std::string port{"/dev/ttyUSB0