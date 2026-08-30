cmake_minimum_required(VERSION 3.8)
project(turtlesim_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetMode.srv"
  "srv/GetMode.srv"
  "action/NavigateWaypoints.action"
)

ament_package()
