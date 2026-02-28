cmake_minimum_required(VERSION 3.8)
project(communication_software)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(ib2_msgs REQUIRED)
find_package(platform_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Message.msg"
  "msg/Telemetry.msg"
  "srv/Telecommand.srv"
  "srv/Setting.srv"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces ib2_msgs platform_msgs
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install Python scripts
install(PROGRAMS
  script/telemetry_bridge.py
  script/telecommand_bridge.py
  script/send_telemetry_test.py
  script/send_telecommand_test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
