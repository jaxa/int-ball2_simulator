#ifndef IB2_MSGS_H
#define IB2_MSGS_H

// ROS 2 standard messages
#include "action_msgs/msg/goal_info.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

// ib2_msgs messages
#include "ib2_msgs/msg/alive_status.hpp"
#include "ib2_msgs/msg/battery_charge_info.hpp"
#include "ib2_msgs/msg/ctl_profile.hpp"
#include "ib2_msgs/msg/ctl_status.hpp"
#include "ib2_msgs/msg/ctl_status_type.hpp"
#include "ib2_msgs/msg/display_manager_status.hpp"
#include "ib2_msgs/msg/fan_status.hpp"
#include "ib2_msgs/msg/file_monitoring_status.hpp"
#include "ib2_msgs/msg/imu.hpp"
#include "ib2_msgs/msg/led_colors.hpp"
#include "ib2_msgs/msg/main_camera_status.hpp"
#include "ib2_msgs/msg/main_camera_white_balance_mode.hpp"
#include "ib2_msgs/msg/main_camera_resolution_type.hpp"
#include "ib2_msgs/msg/marker.hpp"
#include "ib2_msgs/msg/mode.hpp"
#include "ib2_msgs/msg/monitored_alive_statuses.hpp"
#include "ib2_msgs/msg/navigation.hpp"
#include "ib2_msgs/msg/navigation_debug.hpp"
#include "ib2_msgs/msg/navigation_status.hpp"
#include "ib2_msgs/msg/power_status.hpp"
#include "ib2_msgs/msg/ros_param.hpp"
#include "ib2_msgs/msg/system_status.hpp"

// ib2_msgs services
#include "ib2_msgs/srv/dock.hpp"
#include "ib2_msgs/srv/dump_ros_params.hpp"
#include "ib2_msgs/srv/exit_docking_mode.hpp"
#include "ib2_msgs/srv/get_ros_param.hpp"
#include "ib2_msgs/srv/get_ros_params.hpp"
#include "ib2_msgs/srv/load_ros_params.hpp"
#include "ib2_msgs/srv/marker_correction.hpp"
#include "ib2_msgs/srv/record.hpp"
#include "ib2_msgs/srv/set_maintenance_mode.hpp"
#include "ib2_msgs/srv/set_ros_param.hpp"
#include "ib2_msgs/srv/set_ros_params.hpp"
#include "ib2_msgs/srv/switch_power.hpp"
#include "ib2_msgs/srv/update_parameter.hpp"

// ib2_msgs actions
#include "ib2_msgs/action/ctl_command.hpp"
#include "ib2_msgs/action/navigation_start_up.hpp"

// platform_msgs (used in some ib2_msgs contexts)
#include "platform_msgs/msg/node_status_value.hpp"

// Namespace aliases for ROS 1 → ROS 2 compatibility
namespace ib2_msgs = ib2_msgs;

// Type aliases for easier migration from ROS 1 flat namespace
namespace ib2_msgs_compat {
  // Messages
  using Mode = ib2_msgs::msg::Mode;
  using PowerStatus = ib2_msgs::msg::PowerStatus;
  using CtlStatusType = ib2_msgs::msg::CtlStatusType;
  using CtlStatus = ib2_msgs::msg::CtlStatus;
  using CtlProfile = ib2_msgs::msg::CtlProfile;
  using Navigation = ib2_msgs::msg::Navigation;
  using NavigationDebug = ib2_msgs::msg::NavigationDebug;
  using NavigationStatus = ib2_msgs::msg::NavigationStatus;
  using IMU = ib2_msgs::msg::IMU;
  using SystemStatus = ib2_msgs::msg::SystemStatus;
  using AliveStatus = ib2_msgs::msg::AliveStatus;
  using MonitoredAliveStatuses = ib2_msgs::msg::MonitoredAliveStatuses;
  using BatteryChargeInfo = ib2_msgs::msg::BatteryChargeInfo;
  using DisplayManagerStatus = ib2_msgs::msg::DisplayManagerStatus;
  using FanStatus = ib2_msgs::msg::FanStatus;
  using FileMonitoringStatus = ib2_msgs::msg::FileMonitoringStatus;
  using LEDColors = ib2_msgs::msg::LEDColors;
  using MainCameraStatus = ib2_msgs::msg::MainCameraStatus;
  using MainCameraWhiteBalanceMode = ib2_msgs::msg::MainCameraWhiteBalanceMode;
  using MainCameraResolutionType = ib2_msgs::msg::MainCameraResolutionType;
  using Marker = ib2_msgs::msg::Marker;
  using RosParam = ib2_msgs::msg::RosParam;

  // Action types
  using CtlCommandGoal = ib2_msgs::action::CtlCommand_Goal;
  using CtlCommandResult = ib2_msgs::action::CtlCommand_Result;
  using CtlCommandFeedback = ib2_msgs::action::CtlCommand_Feedback;
  using CtlCommandFeedbackMessage = ib2_msgs::action::CtlCommand_FeedbackMessage;
  using NavigationStartUpGoal = ib2_msgs::action::NavigationStartUp_Goal;
  using NavigationStartUpResult = ib2_msgs::action::NavigationStartUp_Result;
  using NavigationStartUpFeedback = ib2_msgs::action::NavigationStartUp_Feedback;
  using NavigationStartUpFeedbackMessage = ib2_msgs::action::NavigationStartUp_FeedbackMessage;

  // Service types
  using UpdateParameterRequest = ib2_msgs::srv::UpdateParameter::Request;
  using UpdateParameterResponse = ib2_msgs::srv::UpdateParameter::Response;
  using SwitchPowerRequest = ib2_msgs::srv::SwitchPower::Request;
  using RecordRequest = ib2_msgs::srv::Record::Request;
  using SetRosParamRequest = ib2_msgs::srv::SetRosParam::Request;
  using SetRosParamsRequest = ib2_msgs::srv::SetRosParams::Request;
  using GetRosParamRequest = ib2_msgs::srv::GetRosParam::Request;
  using GetRosParamResponse = ib2_msgs::srv::GetRosParam::Response;
  using GetRosParamsRequest = ib2_msgs::srv::GetRosParams::Request;
  using GetRosParamsResponse = ib2_msgs::srv::GetRosParams::Response;
  using DumpRosParamsRequest = ib2_msgs::srv::DumpRosParams::Request;
  using LoadRosParamsRequest = ib2_msgs::srv::LoadRosParams::Request;
  using ExitDockingModeRequest = ib2_msgs::srv::ExitDockingMode::Request;
  using ExitDockingModeResponse = ib2_msgs::srv::ExitDockingMode::Response;
  using SetMaintenanceModeRequest = ib2_msgs::srv::SetMaintenanceMode::Request;
  using SetMaintenanceModeResponse = ib2_msgs::srv::SetMaintenanceMode::Response;
  using MarkerCorrectionRequest = ib2_msgs::srv::MarkerCorrection::Request;
  using MarkerCorrectionResponse = ib2_msgs::srv::MarkerCorrection::Response;
}

#endif // IB2_MSGS_H
