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
find_package(std_msgs REQUIRED)
find_package(fmt REQUIRED)

include_directories(
  include
)

# Glob all .so files in the doly_sdk lib directory (after install)
file(GLOB DOLY_SDK_LIBS "${CMAKE_INSTALL_PREFIX}/lib/lib*.so")

# Drive interface node
add_executable(drive_interface_node src/drive_interface.cpp)
target_include_directories(drive_interface_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(drive_interface_node
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  doly_sdk::doly_sdk
  ${DOLY_SDK_LIBS}
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
  ${DOLY_SDK_LIBS}
  ros2_fmt_logger::ros2_fmt_logger
  fmt::fmt
  spdlog::spdlog
)

install(TARGETS
  drive_interface_node
  arm_interface_node
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
