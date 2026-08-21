cmake_minimum_required(VERSION 3.8)
project(wt901c_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)

add_library(wt901c_protocol STATIC
  src/protocol.cpp
)
target_compile_features(wt901c_protocol PUBLIC cxx_std_17)
target_include_directories(wt901c_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(imu_driver
  src/imu_driver.cpp
)
target_compile_features(imu_driver PUBLIC cxx_std_17)
target_include_directories(imu_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_driver
  wt901c_protocol
)
ament_target_dependencies(imu_driver
  rclcpp
  sensor_msgs
  serial
  std_srvs
  tf2
)

add_executable(wt901c_config
  src/wt901c_config.cpp
)
target_compile_features(wt901c_config PUBLIC cxx_std_17)
ament_target_dependencies(wt901c_config
  serial
)

install(TARGETS
  imu_driver
  wt901c_config
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/wt901c_config_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wt901c_config_gui
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_protocol
    test/test_protocol.cpp
  )
  if(TARGET test_protocol)
    target_compile_features(test_protocol PUBLIC cxx_std_17)
    target_link_libraries(test_protocol wt901c_protocol)
  endif()
endif()

ament_package()
