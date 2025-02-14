#include "telecommand_client.h"
#include <QHostAddress>
#include <QVector3D>
#include <QQuaternion>
#include "communication_software/Telecommand.h"
#include "ib2_msgs.h"
#include "platform_msgs.h"
#include "qdebug_custom.h"
#include "telemetry_telecommand_config.h"
#include "common_log_object.h"

using namespace intball;
using namespace communication_software;

const std::string TelecommandClient::SERVICE_NAME = "telecommand_bridge";

TelecommandClient::TelecommandClient(ros::NodeHandle& nodeHandle, QObject* parent)
    : QObject(parent)
{
    client_ = nodeHandle.serviceClient<communication_software::Telecommand>(SERVICE_NAME);
    communicationConfig_.reset(new CommunicationConfig());
}

TelecommandClient::~TelecommandClient()
{
    client_.shutdown();
}

bool TelecommandClient::send(const communication_software::Message& command)
{
    INFO_START_FUNCTION() << QString("name=%1 dataSize=%2")
                          .arg(QString::fromStdString(command.name)).arg(command.data.size());
    Telecommand commandService;
    commandService.request.command = command;

    if (client_.call(commandService))
    {
        if(commandService.response.result == TelecommandResponse::SUCCESS)
        {
            QString logMessage = "Command sent successfully: name=" + QString::fromStdString(commandService.request.command.name);
            LOG_INFO() << logMessage;
            emit executed(CommandLog(QDateTime::currentDateTime(), CommandLogLevel::INFO, logMessage));
            return true;
        }
        else
        {
            QString logMessage = QString::asprintf("Failed to send command: name=%s result=%d message=\"%s\"",
                                                   commandService.request.command.name.c_str(),
                                                   commandService.response.result,
                                                   commandService.response.message.c_str());
            LOG_WARNING() << logMessage;
            emit executed(CommandLog(QDateTime::currentDateTime(), CommandLogLevel::WARN, logMessage));
            return false;
        }
    }
    else
    {
        QString logMessage = QString("Failed to call service %1. Probably telecommand_bridge node is not up.")
                                     .arg(QString::fromStdString(SERVICE_NAME));
        LOG_WARNING() << logMessage;
        emit executed(CommandLog(QDateTime::currentDateTime(), CommandLogLevel::WARN, logMessage));

        return false;
    }
}

bool TelecommandClient::sendTargetGoalAbsolute(const QVector3D& position, const QQuaternion& attitude)
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET, position, attitude);
}

bool TelecommandClient::sendTargetGoalRelative(const QVector3D& position, const QQuaternion& attitude)
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET, position, attitude);
}

bool TelecommandClient::sendDockingWithMarkerCorrection()
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::DOCK, QVector3D(), QQuaternion(1, 0, 0, 0));
}

bool TelecommandClient::sendDockingWithoutMarkerCorrection()
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::DOCK_WITHOUT_CORRECTION, QVector3D(), QQuaternion(1, 0, 0, 0));
}

bool TelecommandClient::sendRelease()
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::RELEASE, QVector3D(), QQuaternion(1, 0, 0, 0));
}

bool TelecommandClient::sendCtlCommand(const int type, const QVector3D& position, const QQuaternion& attitude)
{
    ib2_msgs::CtlCommandGoal targetGoal;

    targetGoal.target.header.stamp = ros::Time::now();
    if(type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET)
    {
        targetGoal.target.header.frame_id = "iss_body";
    }
    else if(type == ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET)
    {
        targetGoal.target.header.frame_id = "body";
    }

    targetGoal.type.type = type;
    targetGoal.target.pose.position.x = position.x();
    targetGoal.target.pose.position.y = position.y();
    targetGoal.target.pose.position.z = position.z();
    targetGoal.target.pose.orientation.x = attitude.x();
    targetGoal.target.pose.orientation.y = attitude.y();
    targetGoal.target.pose.orientation.z = attitude.z();
    targetGoal.target.pose.orientation.w = attitude.scalar();

    return send(telecommand::NAME_CTL_ACTION_GOAL, targetGoal);
}

bool TelecommandClient::sendCtlCommandStop()
{
    return sendCtlCommand(ib2_msgs::CtlStatusType::STOP_MOVING, QVector3D(), QQuaternion());
}

bool TelecommandClient::sendCtlCommandCancel()
{
    return send(telecommand::NAME_CTL_ACTION_CANCEL, actionlib_msgs::GoalID());
}

bool TelecommandClient::sendUpdateParameter(const telecommand::UPDATE_PARAMETER_TARGET target)
{
    auto targetName = telecommand::UPDATE_PARAMETER_SERVICE_NAME.value(target);
    Q_ASSERT(!targetName.empty());
    return send(targetName, ib2_msgs::UpdateParameterRequest());
}

bool TelecommandClient::sendSwitchPower(const telecommand::SWITCH_POWER_TARGET target, const bool on)
{
    auto targetName = telecommand::SWITCH_POWER_SERVICE_NAME.value(target);
    Q_ASSERT(!targetName.empty());

    ib2_msgs::SwitchPowerRequest request;
    request.power.status = on ? ib2_msgs::PowerStatus::ON : ib2_msgs::PowerStatus::OFF;

    return send(targetName, request);
}

bool TelecommandClient::sendNavigationStartUp(const bool on)
{
    auto targetName = telecommand::NAME_NAVIGATION_STARTUP;

    ib2_msgs::NavigationStartUpActionGoal request;
    request.goal.command = on ? ib2_msgs::NavigationStartUpGoal::ON : ib2_msgs::NavigationStartUpGoal::OFF;

    return send(targetName ,request);
}

bool TelecommandClient::sendRecord(const bool on)
{
    ib2_msgs::RecordRequest request;
    request.command = on ? ib2_msgs::RecordRequest::START : ib2_msgs::RecordRequest::STOP;

    return send(telecommand::NAME_CAMERA_ANC_MICROPHONE_RECORD, request);
};

bool TelecommandClient::sendSetRosParam(const RosParam& setParams)
{
    ib2_msgs::SetRosParamRequest request;
    ib2_msgs::RosParam set;
    set.id = setParams.id;
    set.value = setParams.value;
    switch(setParams.type)
    {
    case RosParamType::STRING:
        set.type = std::string("string");
        break;
    case RosParamType::INTEGER:
        set.type = std::string("int");
        break;
    case RosParamType::FLOAT:
        set.type = std::string("float");
        break;
    case RosParamType::BOOL:
        set.type = std::string("bool");
        break;
    case RosParamType::LIST:
        set.type = std::string("list");
        break;
    case RosParamType::DICT:
        set.type = std::string("dict");
        break;
    }
    request.param = set;

    return send(telecommand::NAME_SET_ROSPARAM, request);
}

bool TelecommandClient::sendSetRosParams(const QList<RosParam>& setParams)
{
    ib2_msgs::SetRosParamsRequest request;
    for(auto i = setParams.begin(); i != setParams.end(); ++i)
    {
        ib2_msgs::RosParam set;
        set.id = (*i).id;
        set.value = (*i).value;
        switch((*i).type)
        {
        case RosParamType::STRING:
            set.type = std::string("string");
            break;
        case RosParamType::INTEGER:
            set.type = std::string("int");
            break;
        case RosParamType::FLOAT:
            set.type = std::string("float");
            break;
        case RosParamType::BOOL:
            set.type = std::string("bool");
            break;
        case RosParamType::LIST:
            set.type = std::string("list");
            break;
        case RosParamType::DICT:
            set.type = std::string("dict");
            break;
        }
        request.params.push_back(set);
    }

    return send(telecommand::NAME_SET_ROSPARAMS, request);
}

bool TelecommandClient::getRosParam(const QString& key)
{
    ib2_msgs::GetRosParamRequest request;
    request.id = key.toStdString();
    return send(telecommand::NAME_GET_ROSPARAM, request);
}

bool TelecommandClient::getRosParams(const QList<QString>& keyList)
{
    ib2_msgs::GetRosParamsRequest request;
    for(auto key : keyList)
    {
        request.ids.push_back(key.toStdString());
    }
    return send(telecommand::NAME_GET_ROSPARAMS, request);
}

bool TelecommandClient::sendDumpRosparams(const QString& path)
{
    ib2_msgs::DumpRosParamsRequest request;
    request.path = path.toStdString();
    return send(telecommand::NAME_DUMP_ROSPARAMS, request);
}

bool TelecommandClient::sendLoadRosparams(const QString& path)
{
    ib2_msgs::LoadRosParamsRequest request;
    request.path = path.toStdString();
    return send(telecommand::NAME_LOAD_ROSPARAMS, request);
}

bool TelecommandClient::sendSetMaintenanceMode(const bool on)
{
    ib2_msgs::SetMaintenanceModeRequest request;
    request.maintenance_on = on;

    return send(telecommand::NAME_SET_MAINTENANCE_MODE, request);
}

bool TelecommandClient::sendExitDockingMode(const unsigned char mode)
{
    if(mode != ib2_msgs::Mode::OPERATION && mode != ib2_msgs::Mode::STANDBY)
    {
        LOG_WARNING() << "Invalid mode " << MODE_TYPE_LABEL.value(mode) << ". "
                      << telecommand::NAME_EXIT_DOCKING_MODE << " accept only"
                      << MODE_TYPE_LABEL.value(ib2_msgs::Mode::OPERATION) << " or " << MODE_TYPE_LABEL.value(ib2_msgs::Mode::STANDBY);
        return false;
    }

    ib2_msgs::ExitDockingModeRequest request;
    request.mode.mode = mode;

    return send(telecommand::NAME_EXIT_DOCKING_MODE, request);
}

bool TelecommandClient::sendExitDockingModeFinish()
{
    return sendExitDockingMode(ib2_msgs::Mode::STANDBY);
}

bool TelecommandClient::sendExitDockingModeCancel()
{
    return sendExitDockingMode(ib2_msgs::Mode::OPERATION);
}

bool TelecommandClient::sendDuty(const QList<double>& duty)
{
    std_msgs::Float64MultiArray request;
    for(auto i = duty.begin(); i != duty.end(); ++i)
    {
        request.data.push_back(*i);
    }

    return send(telecommand::NAME_CTL_DUTY, request);
}

bool TelecommandClient::sendMarkerCorrection()
{
    ib2_msgs::MarkerCorrectionRequest request;

    return send(telecommand::NAME_MARKER_CORRECTION, request);
}

bool TelecommandClient::sendLedLeftColors(const QList<QList<float>>& colors)
{
    ib2_msgs::LEDColors request;

    for(auto i = colors.begin(); i != colors.end(); ++i)
    {
        std_msgs::ColorRGBA color;
        if((*i).length() >= 3)
        {
            color.r = (*i).at(0);
            color.g = (*i).at(1);
            color.b = (*i).at(2);
        }
        else
        {
            LOG_CRITICAL() << __FUNCTION__ << " : There are too few elements for color specification.";
        }

        color.a = 0;
        request.colors.push_back(color);
    }

    return send(telecommand::NAME_LED_DISPLAY_LEFT_COLORS, request);
}

bool TelecommandClient::sendLedRightColors(const QList<QList<float>>& colors)
{
    ib2_msgs::LEDColors request;

    for(auto i = colors.begin(); i != colors.end(); ++i)
    {
        std_msgs::ColorRGBA color;
        if((*i).length() >= 3)
        {
            color.r = (*i).at(0);
            color.g = (*i).at(1);
            color.b = (*i).at(2);
        }
        else
        {
            LOG_CRITICAL() << __FUNCTION__ << " : There are too few elements for color specification.";
        }

        color.a = 0;
        request.colors.push_back(color);
    }

    return send(telecommand::NAME_LED_DISPLAY_RIGHT_COLORS, request);
}

bool TelecommandClient::sendDisplayManagerSwitch(const bool on)
{
    ib2_msgs::SwitchPowerRequest request;
    request.power.status = on ? ib2_msgs::PowerStatus::ON : ib2_msgs::PowerStatus::OFF;

    return send(telecommand::NAME_DISPLAY_MANAGER_SWITCH, request);
}

bool TelecommandClient::sendLighting(const bool on)
{
    ib2_msgs::SwitchPowerRequest request;
    request.power.status = on ? ib2_msgs::PowerStatus::ON : ib2_msgs::PowerStatus::OFF;

    return send(telecommand::NAME_DISPLAY_MANAGER_FLASH, request);
}

bool TelecommandClient::sendForcedRelease()
{
    return send(telecommand::NAME_FORCED_RELEASE, std_msgs::Empty());
}

bool TelecommandClient::sendReboot()
{
    return send(telecommand::NAME_REBOOT, std_msgs::Empty());
}

bool TelecommandClient::sendSetOperationType(platform_msgs::OperationType type)
{
    platform_msgs::SetOperationTypeRequest request;
    request.type = type;

    return send(telecommand::NAME_SET_OPERATION_TYPE, request);
}

bool TelecommandClient::sendUserNode(const bool on, const QString& user, const QString& launch, const QString& image)
{
    platform_msgs::UserNodeCommandRequest request;
    request.command = on;
    request.user = user.toStdString();
    request.launch = launch.toStdString();
    request.image = image.toStdString();

    return send(telecommand::NAME_USER_NODE, request);
}

bool TelecommandClient::sendUserLogic(const bool on, platform_msgs::UserLogic logic)
{
    platform_msgs::UserLogicCommandRequest request;
    request.command = on;
    request.logic = logic;

    return send(telecommand::NAME_USER_LOGIC, request);
}

communication_software::Message TelecommandClient::createMessageBaseForDock(const dock::telecommand::Index index)
{
    communication_software::Message command;
    command.msg_type = command.DOCK_ROW_BINARY_DATA;

    auto headerList = dock::telecommand::CODE.value(index);
    for(auto i = headerList.begin(); i != headerList.end(); ++i)
    {
        command.data.push_back(*i);
    }
    return command;
}

bool TelecommandClient::sendDockSetHostIPAddr(const QHostAddress& addr)
{
    auto command = createMessageBaseForDock(dock::telecommand::Index::SET_HOST_IP_ADDR);
    command.name = "Docking station: SET_HOST_IP_ADDR (OCS IP address)";
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 24) & 0xFF));
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 16) & 0xFF));
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 8) & 0xFF));
    command.data.push_back(static_cast<uint8_t>(addr.toIPv4Address() & 0xFF));

    return send(command);
}

bool TelecommandClient::sendDockSetCommandPort(const unsigned short port)
{
    auto command = createMessageBaseForDock(dock::telecommand::Index::SET_COMMAND_PORT);
    command.name = "Docking station: SET_COMMAND_PORT";
    command.data.push_back(static_cast<uint8_t>(port & 0xFF));
    command.data.push_back(static_cast<uint8_t>((port >> 8) & 0xFF));

    return send(command);
}

bool TelecommandClient::sendDockSetIBIPAddr(const QHostAddress& addr)
{
    auto command = createMessageBaseForDock(dock::telecommand::Index::SET_IB_ADDR);
    command.name = "Docking station: SET_IB_ADDR (Int-Ball2 IP address)";
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 24) & 0xFF));
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 16) & 0xFF));
    command.data.push_back(static_cast<uint8_t>((addr.toIPv4Address() >> 8) & 0xFF));
    command.data.push_back(static_cast<uint8_t>(addr.toIPv4Address() & 0xFF));

    return send(command);
}

bool TelecommandClient::sendDockMotorOnOff(const dock::telecommand::MOTOR_ON_OFF_TYPE type)
{
    auto command = createMessageBaseForDock(dock::telecommand::Index::MOTOR_ON_OFF);
    command.name = "Docking station: MOTOR_ON_OFF";
    command.data.push_back(static_cast<uint8_t>(type));

    return send(command);
}

bool TelecommandClient::sendDockChargeOnOff(const dock::telecommand::CHARGE_ON_OFF_TYPE type)
{
    auto command = createMessageBaseForDock(dock::telecommand::Index::CHARGE_ON_OFF);
    command.name = "Docking station: CHARGE_ON_OFF";
    command.data.push_back(static_cast<uint8_t>(type));

    return send(command);
}
