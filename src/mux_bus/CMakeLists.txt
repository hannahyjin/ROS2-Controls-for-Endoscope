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

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MPRLSPressures.msg"
)

ament_export_dependencies(rosidl_default_runtime)

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

add_executable(mux_node
  src/bno055_i2c_driver.cpp
  src/Adafruit_MPRLS.cpp
  src/TCA9548A.cpp
  src/Mux_Manager.cpp
  src/mux_node.cpp
  src/watchdog.cpp
)

ament_target_dependencies(mux_node
  rclcpp
  sensor_msgs
  std_msgs
  diagnostic_msgs
  std_srvs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

rosidl_target_interfaces(mux_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS mux_node
  DESTINATION lib/mux_bus
)
install(DIRECTORY msg/ DESTINATION share/mux_bus/msg)

ament_package()
