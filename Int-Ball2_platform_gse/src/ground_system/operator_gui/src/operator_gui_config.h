#ifndef OPERATOR_GUI_CONFIG_H
#define OPERATOR_GUI_CONFIG_H

#include "gui_config_base.h"

namespace intball
{
namespace qsettings
{

static const QString THIS_PACKAGE_NAME = "operator_gui";

namespace key
{
static const QString KEY_ROUTE_POINT_SIZE_MAX = "App/RoutePointSizeMax";
static const QString KEY_MAX_XYZ = "App/XYZMax";
static const QString KEY_MIN_XYZ = "App/XYZMin";
static const QString KEY_MAX_RPY_DEG = "App/RPYDegMax";
static const QString KEY_MIN_RPY_DEG = "App/RPYDegMin";
static const QString KEY_STATUS_LOG_NUMBER_MAX = "App/StatusLogNumberMax";
static const QString KEY_ENABLE_MARKER_CORRECTION_ON_DOCK = "App/EnableMarkerCorrectionOnDock";
static const QString KEY_EDITED_ROUTE_SAVE_DIRECTORY = "App/EditedRouteSaveDirectory";
static const QString KEY_RVIZ_CONFIG_ROUTE = "Rviz/RoutePanelConfig";
static const QString KEY_RVIZ_CONFIG_CAMERA = "Rviz/CameraPanelConfig";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_1_POSITION_XYZ = "RvizPanel/CameraPanel1PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_1_DISTANCE = "RvizPanel/CameraPanel1Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_1_PITCH_YAW = "RvizPanel/CameraPanel1PitchYaw";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_2_POSITION_XYZ = "RvizPanel/CameraPanel2PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_2_DISTANCE = "RvizPanel/CameraPanel2Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_2_PITCH_YAW = "RvizPanel/CameraPanel2PitchYaw";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_3_POSITION_XYZ = "RvizPanel/CameraPanel3PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_3_DISTANCE = "RvizPanel/CameraPanel3Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_3_PITCH_YAW = "RvizPanel/CameraPanel3PitchYaw";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_4_POSITION_XYZ = "RvizPanel/CameraPanel4PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_4_DISTANCE = "RvizPanel/CameraPanel4Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_4_PITCH_YAW = "RvizPanel/CameraPanel4PitchYaw";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_5_POSITION_XYZ = "RvizPanel/CameraPanel5PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_5_DISTANCE = "RvizPanel/CameraPanel5Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_5_PITCH_YAW = "RvizPanel/CameraPanel5PitchYaw";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_6_POSITION_XYZ = "RvizPanel/CameraPanel6PositionXYZ";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_6_DISTANCE = "RvizPanel/CameraPanel6Distance";
static const QString KEY_RVIZ_PANEL_CAMERA_PANEL_6_PITCH_YAW = "RvizPanel/CameraPanel6PitchYaw";
static const QString KEY_VIDEO_TYPE= "Video/Type";
static const QString KEY_VIDEO_INPUT = "Video/Input";
static const QString KEY_VIDEO_RESTART_WAITING_TIME = "Video/RestartWaitingTime";
static const QString KEY_SNAPSHOT_DIRECTORY = "Video/SnapshotDirectory";
static const QString KEY_ALERT_EVENT_FILE = "Sound/AlertEventFile";
static const QString KEY_ALERT_EVENT_VOLUME = "Sound/AlertEventVolume";
static const QString KEY_ALERT_EVENT_RINGING_TIME = "Sound/AlertEventRingingTime";
static const QString KEY_WARNING_EVENT_FILE = "Sound/WarningEventFile";
static const QString KEY_WARNING_EVENT_VOLUME = "Sound/WarningEventVolume";
static const QString KEY_WARNING_EVENT_RINGING_TIME = "Sound/WarningEventRingingTime";

} // namespace key

} // namespace qsettings

} // namespace intball

#endif // OPERATOR_GUI_CONFIG_H
