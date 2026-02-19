cmake_minimum_required(VERSION 3.8)

project(doly_sdk)

# Download the repo and copy only SDK/lib and SDK/include to the current directory
include(ExternalProject)
ExternalProject_Add(
  doly_diy_sdk
  GIT_REPOSITORY https://github.com/robotdoly/DOLY-DIY.git
  GIT_TAG main
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND
    ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/DOLY-DIY/SDK/lib ${CMAKE_CURRENT_SOURCE_DIR}/lib
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_BINARY_DIR}/DOLY-DIY/SDK/include ${CMAKE_CURRENT_SOURCE_DIR}/include
  INSTALL_COMMAND ""
  SOURCE_DIR ${CMAKE_BINARY_DIR}/DOLY-DIY
)

find_package(ament_cmake REQUIRED)

# Create an interface library target for the vendor package
add_library(doly_sdk INTERFACE)

# Set include directories for build and install
target_include_directories(doly_sdk INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Find all .so files in the lib directory
file(GLOB VENDOR_LIBRARIES "${CMAKE_CURRENT_SOURCE_DIR}/lib/*.so")

# Set link directories and libraries for build and install
target_link_directories(doly_sdk INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
  $<INSTALL_INTERFACE:lib>
)

target_link_libraries(doly_sdk INTERFACE
  $<BUILD_INTERFACE:${VENDOR_LIBRARIES}>
  $<INSTALL_INTERFACE:${VENDOR_LIBRARIES}>
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install shared libraries
install(
  DIRECTORY lib/
  DESTINATION lib
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.so*"
)

# Install the target and export it
install(
  TARGETS doly_sdk
  EXPORT export_doly_sdk
)

# Export the target for downstream packages
ament_export_targets(export_doly_sdk HAS_LIBRARY_TARGET)
ament_export_dependencies()

ament_package()
