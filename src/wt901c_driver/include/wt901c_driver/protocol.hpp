#ifndef WT901C_DRIVER__PROTOCOL_HPP_
#define WT901C_DRIVER__PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace wt901c_driver::protocol
{

constexpr std::size_t kFrameSize = 11;
constexpr uint8_t kFrameHeader = 0x55;
constexpr double kStandardGravityMps2 = 9.80665;
constexpr double kDefaultMagneticTeslaPerLsb = 6.67e-9;

using Frame = std::array<uint8_t, kFrameSize>;
using Vector3 = std::array<double, 3>;
using QuaternionWxyz = std::array<double, 4>;

enum class PacketType : uint8_t
{
  TIME = 0x50,
  ACCELERATION = 0x51,
  ANGULAR_VELOCITY = 0x52,
  ANGLE = 0x53,
  MAGNETIC_FIELD = 0x54,
  QUATERNION = 0x59,
};

bool checksum_valid(const Frame & frame);
bool try_extract_frame(std::vector<uint8_t> & buffer, Frame & frame);

int16_t decode_int16(uint8_t low, uint8_t high);
PacketType packet_type(const Frame & frame);

Vector3 decode_acceleration_mps2(const Frame & frame);
Vector3 decode_angular_velocity_radps(const Frame & frame);
Vector3 decode_euler_radians(const Frame & frame);
Vector3 decode_magnetic_field_tesla(
  const Frame & frame,
  double tesla_per_lsb = kDefaultMagneticTeslaPerLsb);
QuaternionWxyz decode_quaternion_wxyz(const Frame & frame);
double decode_temperature_celsius(const Frame & frame);

}  // namespace wt901c_driver::protocol

#endif  // WT901C_DRIVER__PROTOCOL_HPP_
