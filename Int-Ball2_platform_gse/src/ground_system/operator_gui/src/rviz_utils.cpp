#include "rviz_utils.h"
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>
#include <tf/transform_datatypes.h>
#include "main_camera_view_controller.h"
#include "ros_common.h"
#include "utils.h"

namespace intball
{

void issBodyValuesToBaseFrameCameraPosition(rviz::MainCameraViewController* controller, const QVector3D& position, const QQuaternion& quaternion)
{
    // iss_body座標系からbase座標系に変換する.
    tf::Transform baseToIssBody = getStaticBaseToIssBodyTransform();
    auto rvizPosition = baseToIssBody * qtToTf(position);
    auto rvizQuaternion = baseToIssBody * qtToTf(quaternion);
    tfScalar tmpRoll, tmpPitch, tmpYaw;
    tf::Matrix3x3 m(rvizQuaternion);
    m.getRPY(tmpRoll, tmpPitch, tmpYaw);

    // rvizのcontrollerに値を設定する.
    controller->subProp("Position")->setValue(QVariant(QString("%1;%2;%3")
            .arg(static_cast<qreal>(rvizPosition.x()))
            .arg(static_cast<qreal>(rvizPosition.y()))
            .arg(static_cast<qreal>(rvizPosition.z()))));
    controller->subProp("Roll")->setValue(tmpRoll);
    controller->subProp("Pitch")->setValue(tmpPitch);
    controller->subProp("Yaw")->setValue(tmpYaw);
}

void issBodyValuesToIssBodyFrameCameraFocalPoint(rviz::ViewController* controller, const QVector3D& position)
{
    // iss_body座標系からbase座標系に変換する.
    tf::Transform baseToIssBody = getStaticBaseToIssBodyTransform();
    auto rvizPosition = baseToIssBody * qtToTf(position);

    // rvizのcontrollerに値を設定する.
    controller->subProp("Focal Point")->setValue(QVariant(QString("%1;%2;%3")
                                                          .arg(static_cast<qreal>(rvizPosition.x()))
                                                          .arg(static_cast<qreal>(rvizPosition.y()))
                                                          .arg(static_cast<qreal>(rvizPosition.z()))));
    // カメラ位置座標は初期値に戻す.
    controller->subProp("Distance")->setValue(RVIZ_VIEW_CONTROLLER_DEFAULT_CAMERA_DISTANCE);
    controller->subProp("Position")->setValue(QVariant(QString("%1;%2;%3")
                                                       .arg(0)
                                                       .arg(0)
                                                       .arg(0)));
    controller->subProp("Pitch")->setValue(0);
    controller->subProp("Yaw")->setValue(0);
}

void changeTargetFrame(rviz::ViewController* controller, const QString& targetFrame)
{
    controller->subProp("Target Frame")->setValue(QVariant(targetFrame));
}

void resetViewController(rviz::ViewController* controller, const QString& targetFrame)
{
    changeTargetFrame(controller, targetFrame);
    controller->subProp("Distance")->setValue(RVIZ_VIEW_CONTROLLER_DEFAULT_CAMERA_DISTANCE);
    controller->subProp("Focal Point")->setValue(QVariant(QString("%1;%2;%3")
            .arg(0)
            .arg(0)
            .arg(0)));
    controller->subProp("Position")->setValue(QVariant(QString("%1;%2;%3")
            .arg(0)
            .arg(0)
            .arg(0)));
    controller->subProp("Pitch")->setValue(0);
    controller->subProp("Yaw")->setValue(0);
}

} // namespace intball
