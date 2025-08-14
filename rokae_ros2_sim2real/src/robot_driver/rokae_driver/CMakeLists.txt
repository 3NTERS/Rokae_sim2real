cmake_minimum_required(VERSION 3.8)
project(rokae_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)

set(CMAKE_dir "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_include_dir "${CMAKE_dir}/include")
set(CMAKE_lib_dir "${CMAKE_dir}/lib/Linux/x86_64")
# 设置Qt5的MOC、UIC和RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(${CMAKE_include_dir})
include_directories(${EIGEN3_INCLUDE_DIR})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_compile_options(-Wno-unused-but-set-variable)

file(GLOB CPP_FILES "src/*.cpp")
foreach(cpp_file ${CPP_FILES})
	get_filename_component(exe_name ${cpp_file} NAME_WE)
	add_executable(${exe_name} ${cpp_file})
	ament_target_dependencies(${exe_name} 
	rclcpp
	std_msgs
	std_srvs
	sensor_msgs
	trajectory_msgs
	geometry_msgs)
	install(TARGETS
	    ${exe_name} 
	  DESTINATION lib/${PROJECT_NAME}
	)
	target_link_libraries(${exe_name}
	  ${CMAKE_lib_dir}/libxCoreSDK.a
	  ${CMAKE_lib_dir}/libxMateModel.a
	  ${CMAKE_lib_dir}/libxCoreSDK.so.0.4.1
	  Qt5::Core
  	  Qt5::Widgets
	)

endforeach()

ament_package()
