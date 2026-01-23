/**
 * Copyright (c) Inpyo Lee
 * last updated 26.01.23
 * 파일 기능: 
 * IMU 센서 드라이버 헤더 파일
 */

#ifndef IMU_DRIVER_HPP
#define IMU_DRIVER_HPP

#include <memory>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "std_srvs/srv/empty.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"


class IMUDriver
: public rclcpp::Node
{
public:
  IMUDriver();

private:
// 멤버 변수
  std::unique_ptr<serial::Serial> serial_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Imu imu_msgs_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calib_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  /**
   * @brief 버퍼 확인 콜백함수
   */
  void process_serial();


  /**
   * @brief 데이터 값을 받아 Publish
   */
  void parse_and_publish_imu_data(const std::vector<uint8_t>& payload);


  /**
   * @brief WT901C 레지스터 제어를 위한 공통 명령 전송 함수
   * @param reg 레지스터 주소 (Byte 2)
   * @param low 데이터 하위 바이트 (Byte 3)
   * @param high 데이터 상위 바이트 (Byte 4)
   */
  void send_command(uint8_t reg, uint8_t low, uint8_t high);

    
  /**
   * @brief 가속도 및 자이로스코프 영점 보정 수행
   * @details Unlock -> Calibration Start -> Save 순서로 진행
   */
  void calibrate_sensor();

  /**
   * @brief 레지스터 설정 서비스 콜백 함수 선언
   */
  void handle_calibration(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
  );

  


};



#endif