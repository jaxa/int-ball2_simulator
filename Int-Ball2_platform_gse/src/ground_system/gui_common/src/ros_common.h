#ifndef ROS_COMMON_H
#define ROS_COMMON_H
#include <boost/shared_ptr.hpp>
#include <tf/transform_datatypes.h>
#include <QString>
namespace ros
{
class NodeHandle;
}
namespace tf
{
class TransformListener;
}

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

ros::NodeHandle* getNodeHandle();
boost::shared_ptr<tf::TransformListener> getTransformListener();
tf::Transform& getStaticBaseToIssBodyTransform();
}

#endif // ROS_COMMON_H
