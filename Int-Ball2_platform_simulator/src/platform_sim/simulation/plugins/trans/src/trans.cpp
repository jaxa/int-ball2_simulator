// trans node for ROS 2 + Gazebo Harmonic
//
// Subscribes to a Gazebo model's pose topic (bridged via ros_gz_bridge)
// and broadcasts the pose as a TF transform.
//
// In Gazebo Harmonic, each model's pose is published on:
//   /model/<model_name>/pose  (gz.msgs.Pose)
// which ros_gz_bridge converts to geometry_msgs/msg/PoseStamped.
//
// Parameters:
//   model_name   - Name of the Gazebo model (used for default topic)
//   world_frame  - Parent TF frame
//   base_frame   - Child TF frame
//   updateFreqHz - Update rate (Hz)
//   pose_topic   - Override the pose subscription topic
//                  (default: /model/<model_name>/pose)

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class TransNode : public rclcpp::Node
{
public:
  TransNode()
  : Node("trans")
  {
    // Declare parameters
    this->declare_parameter<std::string>("model_name", "");
    this->declare_parameter<std::string>("world_frame", "");
    this->declare_parameter<std::string>("base_frame", "");
    this->declare_parameter<int>("updateFreqHz", 20);
    this->declare_parameter<std::string>("pose_topic", "");

    model_name_ = this->get_parameter("model_name").as_string();
    world_frame_ = this->get_parameter("world_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();

    std::string pose_topic = this->get_parameter("pose_topic").as_string();
    if (pose_topic.empty()) {
      pose_topic = "/model/" + model_name_ + "/pose";
    }

    int update_freq_hz = this->get_parameter("updateFreqHz").as_int();

    RCLCPP_INFO(
      this->get_logger(),
      "[trans] model_name:%s, world_frame:%s, base_frame:%s, updateFreqHz:%d, topic:%s",
      model_name_.c_str(), world_frame_.c_str(), base_frame_.c_str(),
      update_freq_hz, pose_topic.c_str());

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      std::bind(&TransNode::pose_callback, this, std::placeholders::_1));

    (void)update_freq_hz;  // Rate is determined by incoming messages
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped t;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
      t.header.stamp = this->now();
    else
      t.header.stamp = msg->header.stamp;
    t.header.frame_id = world_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = msg->pose.position.x;
    t.transform.translation.y = msg->pose.position.y;
    t.transform.translation.z = msg->pose.position.z;
    t.transform.rotation = msg->pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::string model_name_;
  std::string world_frame_;
  std::string base_frame_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TransNode>());
  rclcpp::shutdown();
  return 0;
}
