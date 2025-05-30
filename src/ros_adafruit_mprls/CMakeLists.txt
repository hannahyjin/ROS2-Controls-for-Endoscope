cmake_minimum_required(VERSION 3.8)
project(ros_adafruit_mprls)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mux_manager REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/MPRLSPressures.msg"
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

add_library(sensor_drivers SHARED
    src/Adafruit_MPRLS.cpp
)

target_include_directories(sensor_drivers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(sensor_drivers 
  i2c
)

ament_target_dependencies(sensor_drivers
    rclcpp 
    mux_manager
)

add_executable(mprls_node src/mprls_publisher.cpp)
target_link_libraries(mprls_node sensor_drivers)

ament_target_dependencies(mprls_node
    rclcpp
    std_msgs
    mux_manager
)

rosidl_target_interfaces(mprls_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

set_target_properties(mprls_node PROPERTIES
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "$ORIGIN/../lib"
)

install(TARGETS
    sensor_drivers
    DESTINATION lib
)

install(TARGETS 
    mprls_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ 
    DESTINATION include
)

install(DIRECTORY msg/ 
    DESTINATION share/${PROJECT_NAME}/msg
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
