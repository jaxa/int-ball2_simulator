#ifndef PLATFORM_MSGS_H
#define PLATFORM_MSGS_H

#include "builtin_interfaces/msg/time.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"

// platform_msgs messages
#include "platform_msgs/msg/camera_status.hpp"
#include "platform_msgs/msg/container_status.hpp"
#include "platform_msgs/msg/manager_status.hpp"
#include "platform_msgs/msg/microphone_status.hpp"
#include "platform_msgs/msg/mode.hpp"
#include "platform_msgs/msg/monitor_status.hpp"
#include "platform_msgs/msg/node_status_value.hpp"
#include "platform_msgs/msg/operation_type.hpp"
#include "platform_msgs/msg/power_status.hpp"
#include "platform_msgs/msg/user_logic.hpp"
#include "platform_msgs/msg/user_node_status.hpp"

// platform_msgs services
#include "platform_msgs/srv/set_operation_type.hpp"
#include "platform_msgs/srv/user_logic_command.hpp"
#include "platform_msgs/srv/user_node_command.hpp"

// Namespace compat aliases
namespace platform_msgs_compat {
  using Mode = platform_msgs::msg::Mode;
  using OperationType = platform_msgs::msg::OperationType;
  using ContainerStatus = platform_msgs::msg::ContainerStatus;
  using ManagerStatus = platform_msgs::msg::ManagerStatus;
  using UserLogic = platform_msgs::msg::UserLogic;
  using PowerStatus = platform_msgs::msg::PowerStatus;
  using UserNodeStatus = platform_msgs::msg::UserNodeStatus;

  // Service types
  using SetOperationTypeRequest = platform_msgs::srv::SetOperationType::Request;
  using UserNodeCommandRequest = platform_msgs::srv::UserNodeCommand::Request;
  using UserNodeCommandResponse = platform_msgs::srv::UserNodeCommand::Response;
  using UserLogicCommandRequest = platform_msgs::srv::UserLogicCommand::Request;
  using UserLogicCommandResponse = platform_msgs::srv::UserLogicCommand::Response;
}

#endif // PLATFORM_MSGS_H
