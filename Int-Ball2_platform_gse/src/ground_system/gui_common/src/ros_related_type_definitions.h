#ifndef ROS_RELATED_TYPE_DEFINITIONS_H
#define ROS_RELATED_TYPE_DEFINITIONS_H

#include <ros/ros.h>

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
    ros::Time stamp;
};

struct NodeStatus
{
    std::string node;
    std::string value;
    ros::Time stamp;
};

struct ContainerStatus
{
    std::string id;
    std::string image;
    unsigned char status;
    ros::Time stamp;
};
}

#endif // ROS_RELATED_TYPE_DEFINITIONS_H
