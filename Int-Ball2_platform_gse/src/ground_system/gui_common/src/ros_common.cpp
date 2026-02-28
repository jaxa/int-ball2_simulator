#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include "qdebug_custom.h"
#include "ros_common.h"

namespace intball
{

static rclcpp::Node::SharedPtr g_node;
static std::shared_ptr<tf2_ros::Buffer> g_tf_buffer;
static std::shared_ptr<tf2_ros::TransformListener> g_tf_listener;

void setNode(rclcpp::Node::SharedPtr node)
{
    g_node = node;
    g_tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    g_tf_listener = std::make_shared<tf2_ros::TransformListener>(*g_tf_buffer);
}

rclcpp::Node::SharedPtr getNode()
{
    return g_node;
}

std::shared_ptr<tf2_ros::Buffer> getTfBuffer()
{
    return g_tf_buffer;
}

std::shared_ptr<tf2_ros::TransformListener> getTransformListener()
{
    return g_tf_listener;
}

tf2::Transform& getStaticBaseToIssBodyTransform()
{
    static QSharedPointer<tf2::Transform> transform;
    if(transform.isNull())
    {
        LOG_INFO() << __FUNCTION__ << ": Lookup a transform between " << rosframe::BASE << " and " << rosframe::ISS_BODY;
        auto buffer = getTfBuffer();

        try
        {
            auto transformStamped = buffer->lookupTransform(
                rosframe::BASE, rosframe::ISS_BODY,
                tf2::TimePointZero,
                tf2::durationFromSec(10.0));

            tf2::Transform tmpTransform;
            tmpTransform.setOrigin(tf2::Vector3(
                transformStamped.transform.translation.x,
                transformStamped.transform.translation.y,
                transformStamped.transform.translation.z));
            tmpTransform.setRotation(tf2::Quaternion(
                transformStamped.transform.rotation.x,
                transformStamped.transform.rotation.y,
                transformStamped.transform.rotation.z,
                transformStamped.transform.rotation.w));

            qDebug() << "A transform between " << rosframe::BASE << " and " << rosframe::ISS_BODY << ": origin(" <<
                     tmpTransform.getOrigin().x() << tmpTransform.getOrigin().y() << tmpTransform.getOrigin().z() << ") rotation(" <<
                     tmpTransform.getRotation().x() << tmpTransform.getRotation().y() <<
                     tmpTransform.getRotation().z() << tmpTransform.getRotation().w() << ")";
            transform.reset(new tf2::Transform(tmpTransform));
        }
        catch (tf2::TransformException &ex)
        {
            LOG_WARNING() << " Can't lookup transform: " <<  ex.what();
            transform.reset(new tf2::Transform());
        }
    }

    return *transform;
}

}
