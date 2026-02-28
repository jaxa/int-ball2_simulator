#ifndef ROS_RELATED_TYPE_DEFINITIONS_H
#define ROS_RELATED_TYPE_DEFINITIONS_H

#include <rclcpp/rclcpp.hpp>
#include "builtin_interfaces/msg/time.hpp"

namespace intball
{

enum class RosParamType {
    STRING,
    BOOL,
    INTEGER,
    FLOAT,
    LIST,
    DICT,
};

struct RosParam
{
    std::string id;
    std::string value;
    RosParamType type;
    builtin_interfaces::msg::Time stamp;
};

struct NodeStatus
{
    std::string node;
    std::string value;
    builtin_interfaces::msg::Time stamp;
};

struct ContainerStatus
{
    std::string id;
    std::string image;
    unsigned char status;
    builtin_interfaces::msg::Time stamp;
};
}

#endif // ROS_RELATED_TYPE_DEFINITIONS_H
