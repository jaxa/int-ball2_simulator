#include "camera_config.h"
#include <string>
#include <QFile>
#include <QString>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include "exception/config_error.h"
#include "gui_config_base.h"
#include "qdebug_custom.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

const std::string CameraConfig::KEY_RESOLUTION = "resolution_settings";

CameraConfig::CameraConfig()
{
    std::string path = ros::package::getPath(Config::valueAsStdString(KEY_CAMERA_CONFIG_PACKAGE));
    std::string file = Config::valueAsStdString(KEY_CAMERA_CONFIG_FILE_PATH);
    if(file.front() != '/')
    {
        file = "/" + file;
    }

    std::string fullPath = path + file;
    QFile configFile(QString::fromStdString(fullPath));
    if(!configFile.exists())
    {
        throwIntBallConfigError(fullPath, "File not found.");
    }

    config_ = YAML::LoadFile(path + file);
    if(!config_.IsDefined() || !config_[KEY_RESOLUTION].IsSequence())
    {
        throwIntBallConfigError(path + file, "Invalid YAML file.");
    }

    filePath_ = QString::fromStdString(path + file);

    // カメラ設定情報の読み出し.
    try
    {
        for(auto i = config_[KEY_RESOLUTION].begin(); i != config_[KEY_RESOLUTION].end(); ++i)
        {
            ResolutionSettingsItem configItem;
            configItem.id = (*i)["id"].as<unsigned int>();
            configItem.name = QString::fromStdString((*i)["name"].as<std::string>());
            configItem.width = (*i)["image_width"].as<unsigned int>();
            configItem.height = (*i)["image_height"].as<unsigned int>();
            resolutionConfig_.insert(configItem.id, configItem);

            LOG_DEBUG() << QString("Load config: id=%1 name=%2 width=%3 height=%4")
                           .arg(configItem.id).arg(configItem.name).arg(configItem.width).arg(configItem.height);
        }
    }
    catch(...)
    {
        throwIntBallConfigError(path + file, "Loading failed.");
    }
}

QString CameraConfig::getFilePath()
{
    return filePath_;
}

const QMap<unsigned int, ResolutionSettingsItem>& CameraConfig::getAllQualitySettings()
{
    return resolutionConfig_;
}

ResolutionSettingsItem CameraConfig::getQualitySettingById(const unsigned int id)
{
    Q_ASSERT(config_.IsDefined());

    if(resolutionConfig_.count(id) > 0)
    {
        return resolutionConfig_.value(id);
    }
    else
    {
        LOG_WARNING() << "The ID(" << id << ") not found in " << KEY_RESOLUTION;
        return ResolutionSettingsItem();
    }
}

QString CameraConfig::getQualitySettingAsStringById(const unsigned int id)
{
    Q_ASSERT(config_.IsDefined());

    auto value = getQualitySettingById(id);
    return QString("%1(%2) %3x%4").arg(value.name)
                                  .arg(id).arg(value.width).arg(value.height);
}
