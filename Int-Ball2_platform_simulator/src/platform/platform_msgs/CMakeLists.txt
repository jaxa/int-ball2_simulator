cmake_minimum_required(VERSION 3.8)
project(platform_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CameraStatus.msg"
  "msg/ContainerStatus.msg"
  "msg/ManagerStatus.msg"
  "msg/MicrophoneStatus.msg"
  "msg/Mode.msg"
  "msg/MonitorStatus.msg"
  "msg/NodeStatusValue.msg"
  "msg/OperationType.msg"
  "msg/PowerStatus.msg"
  "msg/UserLogic.msg"
  "msg/UserNodeStatus.msg"
  "srv/SetOperationType.srv"
  "srv/StopProcessingUserNode.srv"
  "srv/UserLogicCommand.srv"
  "srv/UserNodeCommand.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
