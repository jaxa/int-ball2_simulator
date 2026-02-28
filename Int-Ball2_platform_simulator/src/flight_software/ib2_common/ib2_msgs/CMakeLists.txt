cmake_minimum_required(VERSION 3.8)
project(ib2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/AliveStatus.msg"
  "msg/BatteryChargeInfo.msg"
  "msg/CtlProfile.msg"
  "msg/CtlStatus.msg"
  "msg/CtlStatusType.msg"
  "msg/DiskSpace.msg"
  "msg/DisplayManagerStatus.msg"
  "msg/FanStatus.msg"
  "msg/FileMonitoringStatus.msg"
  "msg/IMU.msg"
  "msg/LEDColors.msg"
  "msg/MainCameraResolutionType.msg"
  "msg/MainCameraStatus.msg"
  "msg/MainCameraWhiteBalanceMode.msg"
  "msg/Marker.msg"
  "msg/Mode.msg"
  "msg/MonitoredAliveStatuses.msg"
  "msg/Navigation.msg"
  "msg/NavigationDebug.msg"
  "msg/NavigationStatus.msg"
  "msg/PowerStatus.msg"
  "msg/RosParam.msg"
  "msg/RosParams.msg"
  "msg/Slam.msg"
  "msg/SystemStatus.msg"
)

set(srv_files
  "srv/Dock.srv"
  "srv/DumpRosParams.srv"
  "srv/ExitDockingMode.srv"
  "srv/GetRosParam.srv"
  "srv/GetRosParams.srv"
  "srv/LoadRosParams.srv"
  "srv/MarkerCorrection.srv"
  "srv/Record.srv"
  "srv/SetMaintenanceMode.srv"
  "srv/SetRosParam.srv"
  "srv/SetRosParams.srv"
  "srv/SwitchPower.srv"
  "srv/UpdateParameter.srv"
)

set(action_files
  "action/CtlCommand.action"
  "action/NavigationStartUp.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
