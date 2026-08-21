#ifndef WT901C_DRIVER__IMU_DRIVER_HPP_
#define WT901C_DRIVER__IMU_DRIVER_HPP_

#include <array>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "serial/serial.h"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "wt901c_driver/protocol.hpp"

namespace wt901c_driver
{

class Wt901cDriver : public rclcpp::Node
{
public:
  Wt901cDriver();
  ~Wt901cDriver() override;

private:
  void load_parameters();
  void validate_parameters() const;
  void initialize_covariances();

  void poll_serial();
  bool try_connect_serial();
  void disconnect_serial(const std::string & reason);
  bool read_available_bytes();
  void process_rx_buffer();
  void handle_frame(const protocol::Frame & frame);
  void reset_sample_cycle();

  bool update_orientation_from_euler(const protocol::Frame & frame);
  bool update_orientation_from_quaternion(const protocol::Frame & frame);
  void publish_imu_if_complete();
  void publish_temperature(const protocol::Frame & frame);
  void publish_magnetic_field(const protocol::Frame & frame);

  bool send_command(uint8_t reg, uint8_t low, uint8_t high);
  bool start_accelerometer_calibration();
  void finish_accelerometer_calibration();
  void abort_calibration(const std::string & reason);
  void handle_legacy_calibration(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void handle_calibration_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  static void set_covariance_diagonal(
    std::array<double, 9> & covariance,
    const std::vector<double> & diagonal);

  std::string port_;
  std::string frame_id_;
  std::string orientation_source_;
  int baudrate_{115200};
  int poll_interval_ms_{2};
  int serial_timeout_ms_{20};
  int reconnect_interval_ms_{1000};
  double calibration_duration_seconds_{5.0};
  double yaw_offset_rad_{0.0};
  double magnetic_tesla_per_lsb_{protocol::kDefaultMagneticTeslaPerLsb};
  double temperature_variance_{0.0};
  std::vector<double> linear_acceleration_covariance_diagonal_;
  std::vector<double> angular_velocity_covariance_diagonal_;
  std::vector<double> orientation_covariance_diagonal_;
  std::vector<double> magnetic_field_covariance_diagonal_;

  std::unique_ptr<serial::Serial> serial_ptr_;
  std::vector<uint8_t> rx_buffer_;
  std::chrono::steady_clock::time_point next_reconnect_attempt_{};

  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField magnetic_field_msg_;
  bool sample_active_{false};
  bool have_acceleration_{false};
  bool have_angular_velocity_{false};

  bool calibration_in_progress_{false};
  std::chrono::steady_clock::time_point calibration_finish_time_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_field_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr legacy_calibration_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_trigger_srv_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
};

}  // namespace wt901c_driver

#endif  // WT901C_DRIVER__IMU_DRIVER_HPP_
