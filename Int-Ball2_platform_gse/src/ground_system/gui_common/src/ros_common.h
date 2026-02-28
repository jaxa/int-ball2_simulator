#ifndef ROS_COMMON_H
#define ROS_COMMON_H
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <QString>

namespace intball
{
namespace rosframe
{
static const std::string BASE = "base";
static const std::string ISS_BODY = "iss_body";
static const std::string BODY = "body";
static const std::string CAMERA = "camera";
static const std::string FIXED = BASE;
}

rclcpp::Node::SharedPtr getNode();
void setNode(rclcpp::Node::SharedPtr node);
std::shared_ptr<tf2_ros::Buffer> getTfBuffer();
std::shared_ptr<tf2_ros::TransformListener> getTransformListener();
tf2::Transform& getStaticBaseToIssBodyTransform();
}

#endif // ROS_COMMON_H
