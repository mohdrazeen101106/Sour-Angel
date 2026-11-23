cmake_minimum_required(VERSION 3.8)
project(sour_angel_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PicoSetpoint.msg"
  "msg/PicoTelemetry.msg"
  "msg/PicoStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
