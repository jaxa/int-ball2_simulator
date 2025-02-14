#ifndef PLATFORM_GUI_CONFIG_H
#define PLATFORM_GUI_CONFIG_H

#include "gui_config_base.h"

namespace intball
{
namespace qsettings
{

static const QString THIS_PACKAGE_NAME = "platform_gui";

namespace key
{
static const QString KEY_USER_PACKAGE_LIST_FILE = "App/DefaultUserPackageListPath";
static const QString KEY_CONTAINER_IMAGE_LIST_FILE = "App/DefaultContainerImageListPath";
static const QString KEY_STATUS_LOG_NUMBER_MAX = "App/StatusLogNumberMax";
static const QString KEY_NODE_STATUS_CLEAR_INTERVAL = "App/NodeStatusClearInterval";
static const QString KEY_ALERT_EVENT_FILE = "Sound/AlertEventFile";
static const QString KEY_ALERT_EVENT_VOLUME = "Sound/AlertEventVolume";
static const QString KEY_ALERT_EVENT_RINGING_TIME = "Sound/AlertEventRingingTime";
static const QString KEY_WARNING_EVENT_FILE = "Sound/WarningEventFile";
static const QString KEY_WARNING_EVENT_VOLUME = "Sound/WarningEventVolume";
static const QString KEY_WARNING_EVENT_RINGING_TIME = "Sound/WarningEventRingingTime";
} // namespace key

} // namespace qsettings

} // namespace intball

#endif // PLATFORM_GUI_CONFIG_H
