/**
 * Copyright (c) Inpyo Lee
 * last updated 26.01.23
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
    double yaw = combine_bytes(payload[3], payload[4]) / 32768.0 * M_PI;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    imu_msgs_.orientation.x = q.x();
    imu_msgs_.orientation.y = q.y();
    imu_msgs_.orientation.z = q.z();
    imu_msgs_.orientation.w = q.w();

    // msg Publish
    imu_pub_->publish(imu_msgs_);
  }
}

// WT901C 레지스터 제어를 위한 공통 명령 전송 함수
void IMUDriver::send_command(uint8_t reg, uint8_t low, uint8_t high)
{

}

// 가속도 및 자이로스코프 영점 보정 수행
void IMUDriver::calibrate_sensor()
{

}





int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUDriver>());
  rclcpp::shutdown();
  return 0;
}