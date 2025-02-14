#include "communication_config.h"
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

const std::string CommunicationConfig::KEY_TELECOMMAND = "telecommand";
const std::string CommunicationConfig::KEY_TELECOMMAND_ID = "id";
const std::string CommunicationConfig::KEY_TELECOMMAND_NAME = "name";
const std::string CommunicationConfig::KEY_TELECOMMAND_DATA_CLASS = "data_class";
const std::string CommunicationConfig::KEY_TELEMETRY = "telemetry";
const std::string CommunicationConfig::KEY_TELEMETRY_NORMAL = "normal";
const std::string CommunicationConfig::KEY_TELEMETRY_SPLIT = "split";


CommunicationConfig::CommunicationConfig()
{
    std::string path = ros::package::getPath(Config::valueAsStdString(KEY_COMMUNICATION_CONFIG_PACKAGE));
    std::string file = Config::valueAsStdString(KEY_COMMUNICATION_CONFIG_FILE_PATH);
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
    if(!config_.IsDefined() || !config_[KEY_TELEMETRY].IsDefined() || !config_[KEY_TELECOMMAND].IsDefined() ||
            !config_[KEY_TELEMETRY][KEY_TELEMETRY_NORMAL].IsSequence() || !config_[KEY_TELEMETRY][KEY_TELEMETRY_SPLIT].IsSequence())
    {
        throwIntBallConfigError(path + file, "Invalid YAML file.");
    }

    filePath_ = QString::fromStdString(path + file);
}

QString CommunicationConfig::getFilePath() const
{
    return filePath_;
}

std::string CommunicationConfig::getDataClassByName(const std::string& name) const
{
    std::string returnString = "";

    Q_ASSERT(config_.IsDefined());

    for(auto i = config_[KEY_TELECOMMAND].begin(); i != config_[KEY_TELECOMMAND].end(); ++i)
    {
        if(!(*i)[KEY_TELECOMMAND_NAME].IsDefined() || !(*i)[KEY_TELECOMMAND_DATA_CLASS].IsDefined())
        {
            LOG_WARNING() << "Omitted reading '" << KEY_TELECOMMAND_NAME << "'";
            continue;
        }

        if((*i)[KEY_TELECOMMAND_NAME].as<std::string>() == name)
        {
            returnString = (*i)[KEY_TELECOMMAND_DATA_CLASS].as<std::string>();
            break;
        }
    }

    if(returnString.empty())
    {
        LOG_WARNING() << name << " is not listed in yaml(" << filePath_ << ").";
    }

    return returnString;
}

QString CommunicationConfig::getTelecommandNameById(const unsigned short id) const
{
    QString returnString = "";

    std::string name = "Invalid command (or empty)";
    for(auto i = config_[KEY_TELECOMMAND].begin(); i != config_[KEY_TELECOMMAND].end(); ++i)
    {
        if(!(*i)[KEY_TELECOMMAND_NAME].IsDefined() || !(*i)[KEY_TELECOMMAND_ID].IsDefined())
        {
            LOG_WARNING() << "Omitted reading '" << KEY_TELECOMMAND_NAME << "'";
            continue;
        }

        if((*i)[KEY_TELECOMMAND_ID].as<unsigned short>() == id)
        {
            name = (*i)[KEY_TELECOMMAND_NAME].as<std::string>();
            break;
        }
    }

    return QString("%1(%2)").arg(QString::fromStdString(name)).arg(id);
}
