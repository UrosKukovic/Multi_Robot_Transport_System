cmake_minimum_required(VERSION 3.20)
project(mrts_fleet_manager)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# Plain C++ core: no rclcpp, so it is unit-testable without a running ROS graph.
add_library(fleet_core src/fleet_core.cpp)
target_include_directories(fleet_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fleet_core test/test_fleet_core.cpp)
  target_link_libraries(test_fleet_core fleet_core)
endif()

ament_package()
