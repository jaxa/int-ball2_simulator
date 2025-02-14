#ifndef RVIZ_UTILS_H
#define RVIZ_UTILS_H

#include <QQuaternion>
#include <QVector3D>
#include <rviz/view_controller.h>
#include "main_camera_view_controller.h"

namespace intball {

/**
 * @brief rvizのViewControllerにおけるカメラ位置とFocal point（フォーカスする座標）の距離デフォルト値.
 */
static const int RVIZ_VIEW_CONTROLLER_DEFAULT_CAMERA_DISTANCE = 10;

/**
 * @brief iss_body座標系の値を,base座標系ViewControllerのカメラ位置に反映する.
 * @param controller rvizのViewController.
 * @param position 座標.
 * @param quaternion 向き.
 */
void issBodyValuesToBaseFrameCameraPosition(rviz::MainCameraViewController* controller, const QVector3D& position, const QQuaternion& quaternion);

/**
 * @brief iss_body座標系の値を,iss_body座標系ViewControllerのFocal point（フォーカスする座標）に反映する.
 * @param controller rvizのViewController.
 * @param position 座標.
 * @param quaternion 向き.
 */
void issBodyValuesToIssBodyFrameCameraFocalPoint(rviz::ViewController* controller, const QVector3D& position);

/**
 * @brief ViewControllerの表示をリセットする.
 * @param controller rvizのViewController.
 */
void resetViewController(rviz::MainCameraViewController* controller);

/**
 * @brief ViewControllerの表示をリセットする.
 * @param controller rvizのViewController.
 * @param targetFrame ViewControllerのTarget frame(フォーカス対象).
 */
void resetViewController(rviz::ViewController* controller, const QString& targetFrame);

/**
 * @brief ViewControllerのTarget frameを変更する.
 * @param controller rvizのViewController.
 * @param targetFrame ViewControllerのTarget frame.
 */
void changeTargetFrame(rviz::ViewController* controller, const QString& targetFrame);

} // namespace intball

#endif // RVIZ_UTILS_H
