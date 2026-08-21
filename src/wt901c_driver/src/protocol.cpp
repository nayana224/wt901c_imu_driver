#include "wt901c_driver/protocol.hpp"

#include <algorithm>
#include <cmath>

namespace wt901c_driver::protocol
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kAccelerationRangeG = 16.0;
constexpr double kAngularVelocityRangeDps = 2000.0;
constexpr double kRawScale = 32768.0;
}

bool checksum_valid(const Frame & frame)
{
  uint8_t checksum = 0;
  for (std::size_t index = 0; index < kFrameSize - 1; ++index) {
    checksum = static_cast<uint8_t>(checksum + frame[index]);
  }
  return checksum == frame.back();
}

bool try_extract_frame(std::vector<uint8_t> & buffer, Frame & frame)
{
  while (true) {
    const auto header = std::find(buffer.begin(), buffer.end(), kFrameHeader);
    if (header == buffer.end()) {
      buffer.clear();
      return false;
    }

    if (header != buffer.begin()) {
      buffer.erase(buffer.begin(), header);
    }

    if (buffer.size() < kFrameSize) {
      return false;
    }

    std::copy_n(buffer.begin(), kFrameSize, frame.begin());
    if (checksum_valid(frame)) {
      buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(kFrameSize));
      return true;
    }

    // Checksum 오류 시 header 한 바이트만 버려 다음 0x55에서 다시 동기화한다.
    buffer.erase(buffer.begin());
  }
}

int16_t decode_int16(uint8_t low, uint8_t high)
{
  const uint16_t raw = static_cast<uint16_t>(
    static_cast<uint16_t>(high) << 8U | static_cast<uint16_t>(low));
  return static_cast<int16_t>(raw);
}

PacketType packet_type(const Frame & frame)
{
  return static_cast<PacketType>(frame[1]);
}

Vector3 decode_acceleration_mps2(const Frame & frame)
{
  const double scale = kAccelerationRangeG * kStandardGravityMps2 / kRawScale;
  return {
    decode_int16(frame[2], frame[3]) * scale,
    decode_int16(frame[4], frame[5]) * scale,
    decode_int16(frame[6], frame[7]) * scale,
  };
}

Vector3 decode_angular_velocity_radps(const Frame & frame)
{
  const double scale = kAngularVelocityRangeDps * (kPi / 180.0) / kRawScale;
  return {
    decode_int16(frame[2], frame[3]) * scale,
    decode_int16(frame[4], frame[5]) * scale,
    decode_int16(frame[6], frame[7]) * scale,
  };
}

Vector3 decode_euler_radians(const Frame & frame)
{
  const double scale = kPi / kRawScale;
  return {
    decode_int16(frame[2], frame[3]) * scale,
    decode_int16(frame[4], frame[5]) * scale,
    decode_int16(frame[6], frame[7]) * scale,
  };
}

Vector3 decode_magnetic_field_tesla(const Frame & frame, double tesla_per_lsb)
{
  return {
    decode_int16(frame[2], frame[3]) * tesla_per_lsb,
    decode_int16(frame[4], frame[5]) * tesla_per_lsb,
    decode_int16(frame[6], frame[7]) * tesla_per_lsb,
  };
}

QuaternionWxyz decode_quaternion_wxyz(const Frame & frame)
{
  return {
    decode_int16(frame[2], frame[3]) / kRawScale,
    decode_int16(frame[4], frame[5]) / kRawScale,
    decode_int16(frame[6], frame[7]) / kRawScale,
    decode_int16(frame[8], frame[9]) / kRawScale,
  };
}

double decode_temperature_celsius(const Frame & frame)
{
  return decode_int16(frame[8], frame[9]) / 100.0;
}

}  // namespace wt901c_driver::protocol
