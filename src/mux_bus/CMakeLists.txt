cmake_minimum_required(VERSION 3.8)
project(mux_bus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
   include
   ${rclcpp_INCLUDE_DIRS}
   ${sensor_msgs_INCLUDE_DIRS}
   ${std_msgs_INCLUDE_DIRS}
   ${diagnostic_msgs_INCLUDE_DIRS}
   ${geometry_msgs_INCLUDE_DIRS}
   ${tf2_INCLUDE_DIRS}
   ${tf2_geometry_msgs_INCLUDE_DIRS}
)

add_executable(bno055_i2c_node
  src/bno055_i2c_driver.cpp
  src/mux_node.cpp
  src/watchdog.cpp
)

ament_target_dependencies(bno055_i2c_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  mux_manager
)

install(TARGETS bno055_i2c_node
  DESTINATION lib/imu_bno055
)

ament_package()
