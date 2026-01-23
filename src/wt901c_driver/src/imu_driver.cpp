/**
 * Copyright (c) Inpyo Lee
 * last updated 26.01.24
 * 파일 기능: 
 * IMU 센서 드라이버 소스 코드
 */
#include "wt901c_driver/imu_driver.hpp"

// 생성자
IMUDriver::IMUDriver()
: Node("imu_driver")
{
  serial_ptr_ = std::make_unique<serial::Serial>(
    "/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(10)
  );

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1), std::bind(&IMUDriver::process_serial, this)
  );

// TF Broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

// Service
  calib_srv_ = this->create_service<std_srvs::srv::Empty>(
    "calibrate_imu",
    std::bind(&IMUDriver::handle_calibration, this, std::placeholders::_1, std::placeholders::_2)
  );
}


// 버퍼 확인 콜백함수
void IMUDriver::process_serial()
{
  // 패킷 길이(11바이트) 확인
  while(serial_ptr_->available() >= 11)
  {
    // 맨앞 헤더 데이터 1개 읽기
    uint8_t header;
    serial_ptr_->read(&header, 1);

    // 헤더 데이터값 확인
    if (header != 0x55) 
    {
      // 헤더가 0x55가 아니면 루프 다시 시작
      continue;
    }

    // 헤더 확인 후, 나머지 10바이트 읽기 (헤더 제외)
    std::vector<uint8_t> payload(10); // 10바이트 크기 변수 선언 (헤더 제외)
    serial_ptr_->read(payload.data(), 10);

    // 체크섬 검증
    uint8_t checksum = header;
    for (int i = 0; i < 9; i++)
    {
      checksum += payload[i];
    }

    if (checksum != payload[9])
    {
      // check sum이 마지막 바이트가 아니면 루프 다시 시작
      continue;
    }

    // 데이터 처리
    parse_and_publish_imu_data(payload);
  } // while
}


// 데이터 값을 받아 Publish
void IMUDriver::parse_and_publish_imu_data(const std::vector<uint8_t>& payload)
{
  imu_msgs_.header.stamp = this->get_clock()->now();
  imu_msgs_.header.frame_id = "imu_link";

  // 데이터 타입 확인
  uint8_t type = payload[0]; 

  // 2바이트 결합 람다 함수 
  auto combine_bytes = [] (uint8_t low, uint8_t high)
  {
    return static_cast<int16_t>((high << 8) | low);
  };

  // 가속도
  if (type == 0x51)
  {
    double ax = combine_bytes(payload[1], payload[2]) / 32768.0 * 16.0 * 9.8;
    imu_msgs_.linear_acceleration.x = ax;
    double ay = combine_bytes(payload[3], payload[4]) / 32768.0 * 16.0 * 9.8;
    imu_msgs_.linear_acceleration.y = ay;
    double az = combine_bytes(payload[5], payload[6]) / 32768.0 * 16.0 * 9.8;
    imu_msgs_.linear_acceleration.z = az;
  }
  // 각속도
  else if (type == 0x52)
  {
    double gx = combine_bytes(payload[1], payload[2]) / 32768.0 * 2000.0 * (M_PI / 180.0);
    imu_msgs_.angular_velocity.x = gx;
    double gy = combine_bytes(payload[3], payload[4]) / 32768.0 * 2000.0 * (M_PI / 180.0);
    imu_msgs_.angular_velocity.y = gy;
    double gz = combine_bytes(payload[5], payload[6]) / 32768.0 * 2000.0 * (M_PI / 180.0);
    imu_msgs_.angular_velocity.z = gz;
  }
  // 각도
  else if (type == 0x53)
  {
    double roll = combine_bytes(payload[1], payload[2]) / 32768.0 * M_PI;
    double pitch = combine_bytes(payload[3], payload[4]) / 32768.0 * M_PI;
    double yaw = combine_bytes(payload[5], payload[6]) / 32768.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    imu_msgs_.orientation.x = q.x();
    imu_msgs_.orientation.y = q.y();
    imu_msgs_.orientation.z = q.z();
    imu_msgs_.orientation.w = q.w();

    // msg Publish
    imu_pub_->publish(imu_msgs_);

  // RViz2 시각화
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = imu_msgs_.header.stamp;
    t.header.frame_id = "world";
    t.child_frame_id = "imu_link";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation = imu_msgs_.orientation;

    tf_broadcaster_->sendTransform(t);
  }
}

// WT901C 레지스터 제어를 위한 공통 명령 전송 함수
void IMUDriver::send_command(uint8_t reg, uint8_t low, uint8_t high)
{
  if (!serial_ptr_ || !serial_ptr_->isOpen())
  {
    RCLCPP_ERROR(this->get_logger(), "Serial port is not open.");
    return;
  }

  std::vector<uint8_t> cmd_packet = {0xFF, 0xAA, reg, low, high};
  serial_ptr_->write(cmd_packet);

  // delay
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
}


// 가속도 및 자이로스코프 영점 보정 수행
void IMUDriver::calibrate_sensor()
{
  RCLCPP_INFO(this->get_logger(), "Starting IMU Calibration... Keep the sensor horizontal and still.");
  
  // 레지스터 쓰기 잠금 해제
  send_command(0x69, 0x88, 0xB5);

  // 가속도/자이로 보정 실행
  // 1: 가속도 및 자이로 보정 시작
  send_command(0x01, 0x01, 0x00);

  // delay
  std::this_thread::sleep_for(std::chrono::seconds(5));

  send_command(0x00, 0x00, 0x00);

  RCLCPP_INFO(this->get_logger(), "Calibration completed and saved.");
}


// 레지스터 설정 서비스 콜백 함수 선언
void IMUDriver::handle_calibration(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response
)
{
  RCLCPP_INFO(this->get_logger(), "Service requested: Starting IMU Calibration...");

  // 타이머 일시 정지 (데이터 파싱과 명령어 송신 간의 충돌 방지)
  timer_->cancel();

  this->calibrate_sensor();

  // 타이머 재개
  timer_->reset();

  RCLCPP_INFO(this->get_logger(), "IMU Calibration process finished successfully.");
}




int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUDriver>());
  rclcpp::shutdown();
  return 0;
}