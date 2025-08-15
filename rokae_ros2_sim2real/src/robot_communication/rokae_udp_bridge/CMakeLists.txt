cmake_minimum_required(VERSION 3.8)
project(rokae_udp_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(rokae_udp_bridge src/rokae_udp_bridge.cpp)

ament_target_dependencies(rokae_udp_bridge
  rclcpp
  sensor_msgs
  control_msgs
)

target_include_directories(rokae_udp_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(rokae_udp_bridge PUBLIC c_std_99 cxx_std_17)

install(TARGETS rokae_udp_bridge
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()