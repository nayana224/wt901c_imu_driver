#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <vector>

#include "wt901c_driver/protocol.hpp"

namespace
{
using wt901c_driver::protocol::Frame;
using wt901c_driver::protocol::PacketType;

Frame make_frame(PacketType type, const std::array<uint8_t, 8> & data)
{
  Frame frame{};
  frame[0] = wt901c_driver::protocol::kFrameHeader;
  frame[1] = static_cast<uint8_t>(type);
  for (std::size_t index = 0; index < data.size(); ++index) {
    frame[index + 2] = data[index];
  }

  uint8_t checksum = 0;
  for (std::size_t index = 0; index < frame.size() - 1; ++index) {
    checksum = static_cast<uint8_t>(checksum + frame[index]);
  }
  frame.back() = checksum;
  return frame;
}

TEST(Protocol, ExtractsValidFrameAfterNoiseAndCorruption)
{
  const auto valid = make_frame(
    PacketType::ACCELERATION,
    {0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xC4, 0x09});

  auto corrupted = valid;
  corrupted.back() = static_cast<uint8_t>(corrupted.back() + 1U);

  std::vector<uint8_t> buffer{0x01, 0x02, 0x03};
  buffer.insert(buffer.end(), corrupted.begin(), corrupted.end());
  buffer.insert(buffer.end(), valid.begin(), valid.end());

  Frame extracted{};
  ASSERT_TRUE(wt901c_driver::protocol::try_extract_frame(buffer, extracted));
  EXPECT_EQ(extracted, valid);
  EXPECT_TRUE(buffer.empty());
}

TEST(Protocol, DecodesAccelerationAndTemperature)
{
  const auto frame = make_frame(
    PacketType::ACCELERATION,
    {0x00, 0x08, 0x00, 0xF8, 0x00, 0x00, 0xC4, 0x09});

  const auto acceleration = wt901c_driver::protocol::decode_acceleration_mps2(frame);
  EXPECT_NEAR(acceleration[0], wt901c_driver::protocol::kStandardGravityMps2, 1e-6);
  EXPECT_NEAR(acceleration[1], -wt901c_driver::protocol::kStandardGravityMps2, 1e-6);
  EXPECT_NEAR(acceleration[2], 0.0, 1e-9);
  EXPECT_NEAR(wt901c_driver::protocol::decode_temperature_celsius(frame), 25.0, 1e-9);
}

TEST(Protocol, DecodesAngularVelocityInRadiansPerSecond)
{
  const auto frame = make_frame(
    PacketType::ANGULAR_VELOCITY,
    {0x00, 0xC0, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00});

  const auto angular_velocity =
    wt901c_driver::protocol::decode_angular_velocity_radps(frame);
  EXPECT_NEAR(angular_velocity[0], -17.453292519943295, 1e-9);
  EXPECT_NEAR(angular_velocity[1], 17.453292519943295, 1e-9);
  EXPECT_NEAR(angular_velocity[2], 0.0, 1e-9);
}

TEST(Protocol, DecodesQuaternionAsWxyz)
{
  const auto frame = make_frame(
    PacketType::QUATERNION,
    {0xFF, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});

  const auto quaternion = wt901c_driver::protocol::decode_quaternion_wxyz(frame);
  EXPECT_NEAR(quaternion[0], 32767.0 / 32768.0, 1e-12);
  EXPECT_DOUBLE_EQ(quaternion[1], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[2], 0.0);
  EXPECT_DOUBLE_EQ(quaternion[3], 0.0);
}

}  // namespace
