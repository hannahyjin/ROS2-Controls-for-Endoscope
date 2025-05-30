cmake_minimum_required(VERSION 3.8)
project(mux_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/SetInt32.srv"
)

include_directories(include)

add_executable(mux_service_node 
    src/Mux_Service.cpp
    src/TCA9548A.cpp
)

ament_target_dependencies(mux_service_node
    rclcpp
)

rosidl_target_interfaces(mux_service_node
    ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
    mux_service_node 
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
