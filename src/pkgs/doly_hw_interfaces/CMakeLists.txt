cmake_minimum_required(VERSION 3.8)
project(doly_hw_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(doly_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2_fmt_logger REQUIRED)
find_package(spdlog REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
)

# Drive interface node
add_executable(drive_interface_node src/drive_interface.cpp)
target_include_directories(drive_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(drive_interface_node
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

# Arm interface node
add_executable(arm_interface_node src/arm_interface.cpp)
target_include_directories(arm_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(arm_interface_node
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

# Imu interface node
add_executable(imu_interface_node src/imu_interface.cpp)
target_include_directories(imu_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(imu_interface_node
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
  tf2::tf2
)

# Led interface node
add_executable(led_interface_node src/led_interface.cpp)
target_include_directories(led_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(led_interface_node
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

# ToF interface node
add_executable(tof_interface_node src/tof_interface.cpp)
target_include_directories(tof_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(tof_interface_node
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

# Edge interface node
add_executable(edge_interface_node src/edge_interface.cpp)
target_include_directories(edge_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(edge_interface_node
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

install(TARGETS
  drive_interface_node
  arm_interface_node
  imu_interface_node
  led_interface_node
  tof_interface_node
  edge_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
