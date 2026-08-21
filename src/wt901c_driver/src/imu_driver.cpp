#include "wt901c_driver/imu_driver.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "tf2/LinearMath/Quaternion.h"

namespace wt901c_driver
{
namespace
{
constexpr std::size_t kMaxReadBytes = 512;
constexpr std::size_t kMaxRxBufferBytes = 8192;
constexpr std::size_t kMaxFramesPerPoll = 128;
constexpr auto kCommandDelay = std::chrono::milliseconds(20);
constexpr uint8_t kUnlockRegister = 0x69;
constexpr uint8_t kCalibrationRegister = 0x01;
constexpr uint8_t kSaveRegister = 0x00;
}

Wt901cDriver::Wt901cDriver()
: Node("imu_driver")
{
  load_parameters();
  validate_parameters();
  initialize_covariances();

  const auto sensor_qos = rclcpp::SensorDataQoS();
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", sensor_qos);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
    "imu/temperature", sensor_qos);
  magnetic_field_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(
    "imu/mag", sensor_qos);

  legacy_calibration_srv_ = create_service<std_srvs::srv::Empty>(
    "calibrate_imu",
    std::bind(
      &Wt901cDriver::handle_legacy_calibration,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  calibration_trigger_srv_ = create_service<std_srvs::srv::Trigger>(
    "calibrate_accelerometer",
    std::bind(
      &Wt901cDriver::handle_calibration_trigger,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  poll_timer_ = create_wall_timer(
    std::chrono::milliseconds(poll_interval_ms_),
    std::bind(&Wt901cDriver::poll_serial, this));

  try_connect_serial();
}

Wt901cDriver::~Wt901cDriver()
{
  if (!serial_ptr_) {
    return;
  }

  try {
    if (serial_ptr_->isOpen()) {
      serial_ptr_->close();
    }
  } catch (const std::exception &) {
    // 소멸 중에는 예외를 외부로 전달하지 않는다.
  }
}

void Wt901cDriver::load_parameters()
{
  port_ = declare_parameter<std::string>("port", "/dev/ttyUSB0");
  baudrate_ = declare_parameter<int>("baudrate", 115200);
  frame_id_ = declare_parameter<std::string>("frame_id", "imu_link");
  orientation_source_ = declare_parameter<std::string>("orientation_source", "angle");
  poll_interval_ms_ = declare_parameter<int>("poll_interval_ms", 2);
  serial_timeout_ms_ = declare_parameter<int>("serial_timeout_ms", 20);
  reconnect_interval_ms_ = declare_parameter<int>("reconnect_interval_ms", 1000);
  calibration_duration_seconds_ = declare_parameter<double>("calibration_duration_seconds", 5.0);
  yaw_offset_rad_ = declare_parameter<double>("yaw_offset_rad", 0.0);
  magnetic_tesla_per_lsb_ = declare_parameter<double>(
    "magnetic_tesla_per_lsb", protocol::kDefaultMagneticTeslaPerLsb);
  temperature_variance_ = declare_parameter<double>("temperature_variance", 0.0);

  linear_acceleration_covariance_diagonal_ = declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance_diagonal", std::vector<double>{0.0, 0.0, 0.0});
  angular_velocity_covariance_diagonal_ = declare_parameter<std::vector<double>>(
    "angular_velocity_covariance_diagonal", std::vector<double>{0.0, 0.0, 0.0});
  orientation_covariance_diagonal_ = declare_parameter<std::vector<double>>(
    "orientation_covariance_diagonal", std::vector<double>{0.0, 0.0, 0.0});
  magnetic_field_covariance_diagonal_ = declare_parameter<std::vector<double>>(
    "magnetic_field_covariance_diagonal", std::vector<double>{0.0, 0.0, 0.0});
}

void Wt901cDriver::validate_parameters() const
{
  if (port_.empty()) {
    throw std::invalid_argument("Parameter 'port' must not be empty.");
  }
  if (baudrate_ <= 0) {
    throw std::invalid_argument("Parameter 'baudrate' must be greater than zero.");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("Parameter 'frame_id' must not be empty.");
  }
  if (orientation_source_ != "angle" && orientation_source_ != "quaternion") {
    throw std::invalid_argument("Parameter 'orientation_source' must be 'angle' or 'quaternion'.");
  }
  if (poll_interval_ms_ <= 0 || serial_timeout_ms_ <= 0 || reconnect_interval_ms_ <= 0) {
    throw std::invalid_argument(
            "poll_interval_ms, serial_timeout_ms, and reconnect_interval_ms must be positive.");
  }
  if (!std::isfinite(calibration_duration_seconds_) || calibration_duration_seconds_ <= 0.0) {
    throw std::invalid_argument("Parameter 'calibration_duration_seconds' must be positive.");
  }
  if (!std::isfinite(yaw_offset_rad_)) {
    throw std::invalid_argument("Parameter 'yaw_offset_rad' must be finite.");
  }
  if (!std::isfinite(magnetic_tesla_per_lsb_) || magnetic_tesla_per_lsb_ <= 0.0) {
    throw std::invalid_argument("Parameter 'magnetic_tesla_per_lsb' must be positive.");
  }
  if (!std::isfinite(temperature_variance_) || temperature_variance_ < 0.0) {
    throw std::invalid_argument("Parameter 'temperature_variance' must be non-negative.");
  }

  const auto validate_diagonal = [](const std::vector<double> & diagonal, const char * name) {
      if (diagonal.size() != 3) {
        throw std::invalid_argument(std::string("Parameter '") + name + "' must contain 3 values.");
      }
      for (const double value : diagonal) {
        if (!std::isfinite(value) || value < 0.0) {
          throw std::invalid_argument(
                  std::string("Parameter '") + name + "' must contain non-negative values.");
        }
      }
    };

  validate_diagonal(
    linear_acceleration_covariance_diagonal_, "linear_acceleration_covariance_diagonal");
  validate_diagonal(angular_velocity_covariance_diagonal_, "angular_velocity_covariance_diagonal");
  validate_diagonal(orientation_covariance_diagonal_, "orientation_covariance_diagonal");
  validate_diagonal(magnetic_field_covariance_diagonal_, "magnetic_field_covariance_diagonal");
}

void Wt901cDriver::initialize_covariances()
{
  set_covariance_diagonal(
    imu_msg_.linear_acceleration_covariance, linear_acceleration_covariance_diagonal_);
  set_covariance_diagonal(
    imu_msg_.angular_velocity_covariance, angular_velocity_covariance_diagonal_);
  set_covariance_diagonal(imu_msg_.orientation_covariance, orientation_covariance_diagonal_);
  set_covariance_diagonal(
    magnetic_field_msg_.magnetic_field_covariance, magnetic_field_covariance_diagonal_);
}

void Wt901cDriver::poll_serial()
{
  const auto steady_now = std::chrono::steady_clock::now();
  if (calibration_in_progress_ && steady_now >= calibration_finish_time_) {
    finish_accelerometer_calibration();
  }

  if (!serial_ptr_ || !serial_ptr_->isOpen()) {
    if (!try_connect_serial()) {
      return;
    }
  }

  if (!read_available_bytes()) {
    return;
  }
  process_rx_buffer();
}

bool Wt901cDriver::try_connect_serial()
{
  const auto steady_now = std::chrono::steady_clock::now();
  if (steady_now < next_reconnect_attempt_) {
    return false;
  }
  next_reconnect_attempt_ = steady_now + std::chrono::milliseconds(reconnect_interval_ms_);

  try {
    auto serial = std::make_unique<serial::Serial>(
      port_,
      static_cast<uint32_t>(baudrate_),
      serial::Timeout::simpleTimeout(static_cast<uint32_t>(serial_timeout_ms_)));

    if (!serial->isOpen()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Serial port '%s' could not be opened.", port_.c_str());
      return false;
    }

    serial_ptr_ = std::move(serial);
    rx_buffer_.clear();
    reset_sample_cycle();
    RCLCPP_INFO(get_logger(), "Connected to WT901C on %s at %d baud.", port_.c_str(), baudrate_);
    return true;
  } catch (const std::exception & error) {
    serial_ptr_.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Failed to open serial port '%s': %s", port_.c_str(), error.what());
    return false;
  }
}

void Wt901cDriver::disconnect_serial(const std::string & reason)
{
  if (!reason.empty()) {
    RCLCPP_ERROR(get_logger(), "Serial connection lost on %s: %s", port_.c_str(), reason.c_str());
  }

  if (serial_ptr_) {
    try {
      if (serial_ptr_->isOpen()) {
        serial_ptr_->close();
      }
    } catch (const std::exception &) {
      // 연결 실패 경로에서는 close 오류보다 재연결 상태 전환을 우선한다.
    }
  }

  serial_ptr_.reset();
  rx_buffer_.clear();
  reset_sample_cycle();
  next_reconnect_attempt_ =
    std::chrono::steady_clock::now() + std::chrono::milliseconds(reconnect_interval_ms_);

  if (calibration_in_progress_) {
    abort_calibration("serial connection was lost");
  }
}

bool Wt901cDriver::read_available_bytes()
{
  try {
    const std::size_t available = serial_ptr_->available();
    if (available == 0) {
      return true;
    }

    const std::size_t read_size = std::min(available, kMaxReadBytes);
    std::vector<uint8_t> bytes(read_size);
    const std::size_t bytes_read = serial_ptr_->read(bytes.data(), read_size);
    rx_buffer_.insert(rx_buffer_.end(), bytes.begin(), bytes.begin() + bytes_read);

    if (rx_buffer_.size() > kMaxRxBufferBytes) {
      RCLCPP_WARN(
        get_logger(),
        "Serial receive buffer exceeded %zu bytes. Dropping buffered data and resynchronizing.",
        kMaxRxBufferBytes);
      rx_buffer_.clear();
      reset_sample_cycle();
    }
    return true;
  } catch (const std::exception & error) {
    disconnect_serial(error.what());
    return false;
  }
}

void Wt901cDriver::process_rx_buffer()
{
  protocol::Frame frame{};
  std::size_t processed_frames = 0;
  while (
    processed_frames < kMaxFramesPerPoll &&
    protocol::try_extract_frame(rx_buffer_, frame))
  {
    handle_frame(frame);
    ++processed_frames;
  }
}

void Wt901cDriver::handle_frame(const protocol::Frame & frame)
{
  if (calibration_in_progress_) {
    return;
  }

  switch (protocol::packet_type(frame)) {
    case protocol::PacketType::ACCELERATION:
    {
      reset_sample_cycle();
      sample_active_ = true;
      have_acceleration_ = true;
      imu_msg_.header.stamp = now();
      imu_msg_.header.frame_id = frame_id_;

      const auto acceleration = protocol::decode_acceleration_mps2(frame);
      imu_msg_.linear_acceleration.x = acceleration[0];
      imu_msg_.linear_acceleration.y = acceleration[1];
      imu_msg_.linear_acceleration.z = acceleration[2];
      publish_temperature(frame);
      break;
    }

    case protocol::PacketType::ANGULAR_VELOCITY:
    {
      if (!sample_active_ || !have_acceleration_) {
        break;
      }
      const auto angular_velocity = protocol::decode_angular_velocity_radps(frame);
      imu_msg_.angular_velocity.x = angular_velocity[0];
      imu_msg_.angular_velocity.y = angular_velocity[1];
      imu_msg_.angular_velocity.z = angular_velocity[2];
      have_angular_velocity_ = true;
      break;
    }

    case protocol::PacketType::ANGLE:
      if (orientation_source_ == "angle" && sample_active_) {
        if (update_orientation_from_euler(frame)) {
          publish_imu_if_complete();
        } else {
          reset_sample_cycle();
        }
      }
      break;

    case protocol::PacketType::QUATERNION:
      if (orientation_source_ == "quaternion" && sample_active_) {
        if (update_orientation_from_quaternion(frame)) {
          publish_imu_if_complete();
        } else {
          reset_sample_cycle();
        }
      }
      break;

    case protocol::PacketType::MAGNETIC_FIELD:
      publish_magnetic_field(frame);
      break;

    default:
      break;
  }
}

void Wt901cDriver::reset_sample_cycle()
{
  sample_active_ = false;
  have_acceleration_ = false;
  have_angular_velocity_ = false;
}

bool Wt901cDriver::update_orientation_from_euler(const protocol::Frame & frame)
{
  const auto euler = protocol::decode_euler_radians(frame);
  tf2::Quaternion orientation;
  orientation.setRPY(euler[0], euler[1], euler[2] + yaw_offset_rad_);
  orientation.normalize();

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();
  return true;
}

bool Wt901cDriver::update_orientation_from_quaternion(const protocol::Frame & frame)
{
  const auto decoded = protocol::decode_quaternion_wxyz(frame);
  const double norm = std::sqrt(
    decoded[0] * decoded[0] + decoded[1] * decoded[1] +
    decoded[2] * decoded[2] + decoded[3] * decoded[3]);
  if (norm < 1e-6) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Received an invalid near-zero quaternion from WT901C.");
    return false;
  }

  tf2::Quaternion orientation(decoded[1], decoded[2], decoded[3], decoded[0]);
  orientation.normalize();

  if (yaw_offset_rad_ != 0.0) {
    tf2::Quaternion yaw_offset;
    yaw_offset.setRPY(0.0, 0.0, yaw_offset_rad_);
    orientation = yaw_offset * orientation;
    orientation.normalize();
  }

  imu_msg_.orientation.x = orientation.x();
  imu_msg_.orientation.y = orientation.y();
  imu_msg_.orientation.z = orientation.z();
  imu_msg_.orientation.w = orientation.w();
  return true;
}

void Wt901cDriver::publish_imu_if_complete()
{
  if (!sample_active_ || !have_acceleration_ || !have_angular_velocity_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Dropping incomplete WT901C sample. Ensure acceleration, gyro, and the selected orientation packet are enabled.");
    reset_sample_cycle();
    return;
  }

  imu_pub_->publish(imu_msg_);
  reset_sample_cycle();
}

void Wt901cDriver::publish_temperature(const protocol::Frame & frame)
{
  sensor_msgs::msg::Temperature temperature_msg;
  temperature_msg.header = imu_msg_.header;
  temperature_msg.temperature = protocol::decode_temperature_celsius(frame);
  temperature_msg.variance = temperature_variance_;
  temperature_pub_->publish(temperature_msg);
}

void Wt901cDriver::publish_magnetic_field(const protocol::Frame & frame)
{
  magnetic_field_msg_.header.stamp = now();
  magnetic_field_msg_.header.frame_id = frame_id_;

  const auto magnetic_field =
    protocol::decode_magnetic_field_tesla(frame, magnetic_tesla_per_lsb_);
  magnetic_field_msg_.magnetic_field.x = magnetic_field[0];
  magnetic_field_msg_.magnetic_field.y = magnetic_field[1];
  magnetic_field_msg_.magnetic_field.z = magnetic_field[2];
  magnetic_field_pub_->publish(magnetic_field_msg_);
}

bool Wt901cDriver::send_command(uint8_t reg, uint8_t low, uint8_t high)
{
  if (!serial_ptr_ || !serial_ptr_->isOpen()) {
    RCLCPP_ERROR(get_logger(), "Cannot write WT901C command because serial port is not open.");
    return false;
  }

  const std::vector<uint8_t> command{0xFF, 0xAA, reg, low, high};
  try {
    const std::size_t bytes_written = serial_ptr_->write(command);
    if (bytes_written != command.size()) {
      disconnect_serial("short serial write while sending a WT901C command");
      return false;
    }
  } catch (const std::exception & error) {
    disconnect_serial(error.what());
    return false;
  }

  // 장치가 연속 register write를 놓치지 않도록 명령 사이에 짧은 간격을 둔다.
  std::this_thread::sleep_for(kCommandDelay);
  return true;
}

bool Wt901cDriver::start_accelerometer_calibration()
{
  if (calibration_in_progress_) {
    RCLCPP_WARN(get_logger(), "WT901C accelerometer calibration is already running.");
    return false;
  }
  if (!serial_ptr_ || !serial_ptr_->isOpen()) {
    RCLCPP_ERROR(get_logger(), "Cannot start calibration because the WT901C is disconnected.");
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Starting WT901C accelerometer calibration. Keep the sensor horizontal and completely still.");

  if (!send_command(kUnlockRegister, 0x88, 0xB5)) {
    return false;
  }
  if (!send_command(kCalibrationRegister, 0x01, 0x00)) {
    return false;
  }

  calibration_in_progress_ = true;
  calibration_finish_time_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(calibration_duration_seconds_));
  rx_buffer_.clear();
  reset_sample_cycle();
  return true;
}

void Wt901cDriver::finish_accelerometer_calibration()
{
  if (!calibration_in_progress_) {
    return;
  }

  if (!send_command(kCalibrationRegister, 0x00, 0x00)) {
    return;
  }
  if (!send_command(kSaveRegister, 0x00, 0x00)) {
    return;
  }

  calibration_in_progress_ = false;
  rx_buffer_.clear();
  reset_sample_cycle();
  RCLCPP_INFO(get_logger(), "WT901C accelerometer calibration completed and configuration saved.");
}

void Wt901cDriver::abort_calibration(const std::string & reason)
{
  if (!calibration_in_progress_) {
    return;
  }

  calibration_in_progress_ = false;
  reset_sample_cycle();
  RCLCPP_ERROR(get_logger(), "WT901C calibration aborted: %s", reason.c_str());
}

void Wt901cDriver::handle_legacy_calibration(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  (void)request;
  (void)response;

  if (!start_accelerometer_calibration()) {
    RCLCPP_ERROR(
      get_logger(),
      "calibrate_imu request could not be started. Use calibrate_accelerometer for an explicit result.");
  }
}

void Wt901cDriver::handle_calibration_trigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  response->success = start_accelerometer_calibration();
  if (response->success) {
    response->message =
      "Accelerometer calibration started. Keep the WT901C horizontal and still until completion is logged.";
    return;
  }

  if (calibration_in_progress_) {
    response->message = "Accelerometer calibration is already running.";
  } else {
    response->message = "Calibration could not start. Check the WT901C serial connection.";
  }
}

void Wt901cDriver::set_covariance_diagonal(
  std::array<double, 9> & covariance,
  const std::vector<double> & diagonal)
{
  covariance.fill(0.0);
  covariance[0] = diagonal[0];
  covariance[4] = diagonal[1];
  covariance[8] = diagonal[2];
}

}  // namespace wt901c_driver

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wt901c_driver::Wt901cDriver>());
  rclcpp::shutdown();
  return 0;
}
