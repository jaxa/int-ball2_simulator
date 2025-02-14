#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "qdebug_custom.h"
#include "ros_common.h"

namespace intball
{

ros::NodeHandle* getNodeHandle()
{
    static ros::NodeHandle& nodeHandle = *new ros::NodeHandle();
    return &nodeHandle;
}

boost::shared_ptr<tf::TransformListener> getTransformListener()
{
    static boost::shared_ptr<tf::TransformListener> listener(new tf::TransformListener());
    return listener;
}

tf::Transform& getStaticBaseToIssBodyTransform()
{
    static QSharedPointer<tf::Transform> transform;
    if(transform.isNull())
    {
        LOG_INFO() << __FUNCTION__ << ": Lookup a transform between " << rosframe::BASE << " and " << rosframe::ISS_BODY;
        auto listener = getTransformListener();

        std::string errorMsg("");
        if (!listener->waitForTransform(rosframe::BASE, rosframe::ISS_BODY, ros::Time(0), ros::Duration(10, 0), ros::Duration(0, 1000000), &errorMsg))
        {
            LOG_WARNING() << " waitForTransform failed: " <<  errorMsg;
        }

        tf::StampedTransform tmpTransform;
        try
        {
            listener->lookupTransform(rosframe::BASE, rosframe::ISS_BODY, ros::Time(0), tmpTransform);
        }
        catch (tf::TransformException &ex)
        {
            LOG_WARNING() << " Can't lookup transform: " <<  ex.what();
        }
        qDebug() << "A transform between " << rosframe::BASE << " and " << rosframe::ISS_BODY << ": orgin(" <<
                 tmpTransform.getOrigin().x() << tmpTransform.getOrigin().y() << tmpTransform.getOrigin().z() << ") rotation(" <<
                 tmpTransform.getRotation().x() << tmpTransform.getRotation().y() <<
                 tmpTransform.getRotation().z() << tmpTransform.getRotation().w() << ")";
        transform.reset(new tf::Transform(tmpTransform));
    }

    return *transform;
}

}
