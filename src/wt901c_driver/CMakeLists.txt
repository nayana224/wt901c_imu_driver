cmake_minimum_required(VERSION 3.8)
project(wt901c_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


####################################################
# 외부 라이브러리 패키지 경로 찾기
####################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(serial REQUIRED)


####################################################
# 실행파일 생성
####################################################
add_executable(imu_driver src/imu_driver.cpp)


####################################################
# 라이브러리와 링크하기
####################################################
ament_target_dependencies(imu_driver
  rclcpp
  sensor_msgs
  tf2
  serial
)

####################################################
# 헤더파일 경로 등록(include 폴더)
####################################################
target_include_directories(imu_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


####################################################
# install 폴더에 복사하여 노드 실행파일 만들기
####################################################
install(TARGETS imu_driver
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
