cmake_minimum_required(VERSION 3.20)
project(mrts_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generates C++/Python types from the .srv file, e.g. mrts_interfaces::srv::SubmitTask
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SubmitTask.srv")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
