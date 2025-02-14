#ifndef TELECOMMAND_PUBLISHER_H
#define TELECOMMAND_PUBLISHER_H

#include <QHostAddress>
#include <QObject>
#include <QScopedPointer>
#include <QString>
#include <ros/ros.h>
#include "camera_config.h"
#include "communication_config.h"
#include "communication_software/Message.h"
#include "common_log_object.h"
#include "ib2_msgs.h"
#include "qdebug_custom.h"
#include "ros_related_type_definitions.h"
#include "telemetry_telecommand_config.h"

namespace intball
{

class TelecommandClient : public QObject
{
    Q_OBJECT
public:
    static const std::string SERVICE_NAME;

    TelecommandClient(ros::NodeHandle& nodeHandle, QObject* parent = nullptr);
    virtual ~TelecommandClient();

    bool send(const communication_software::Message& command);

    template<typename T>
    bool send(const std::string& name, const T& data)
    {
        INFO_START_FUNCTION();

        if(communicationConfig_->getDataClassByName(name) != std::string(ros::message_traits::DataType<T>::value()) &&
                communicationConfig_->getDataClassByName(name) + "Request" != std::string(ros::message_traits::DataType<T>::value()))
        {
            LOG_CRITICAL() << QString("The communication configuration (%1) probably wrong: name=%2 data=%3 data_class=%4")
                           .arg(communicationConfig_->getFilePath())
                           .arg(QString::fromStdString(name))
                           .arg(QString::fromStdString(ros::message_traits::DataType<T>::value()))
                           .arg(QString::fromStdString(communicationConfig_->getDataClassByName(name)) +
                                " or " + QString::fromStdString(communicationConfig_->getDataClassByName(name)) + "Request");
            return false;
        }

        communication_software::Message command;
        command.msg_type = communication_software::Message::INTBALL2_SERIALIZED_BINARY_DATA;
        command.name = name;

        uint32_t size = ros::serialization::serializationLength(data);
        boost::shared_array<uint8_t> buffer(new uint8_t[size]);
        ros::serialization::OStream stream(buffer.get(), size);
        ros::serialization::serialize(stream, data);
        command.data.assign(&buffer[0], &buffer[static_cast<long>(size)]);

        return send(command);
    }

    bool sendTargetGoalAbsolute(const QVector3D& position, const QQuaternion& attitude);

    bool sendTargetGoalRelative(const QVector3D& position, const QQuaternion& attitude);
    bool sendRelease();
    bool sendDockingWithMarkerCorrection();
    bool sendDockingWithoutMarkerCorrection();
    bool sendCtlCommand(const int type, const QVector3D& position, const QQuaternion& attitude);
    bool sendCtlCommandStop();
    bool sendCtlCommandCancel();
    bool sendUpdateParameter(const telecommand::UPDATE_PARAMETER_TARGET target);
    bool sendSwitchPower(const telecommand::SWITCH_POWER_TARGET target, const bool on);
    bool sendNavigationStartUp(const bool on);
    bool sendRecord(const bool on);
    bool sendSetRosParam(const RosParam& setParams);
    bool sendSetRosParams(const QList<RosParam>& setParams);
    bool getRosParam(const QString& key);
    bool getRosParams(const QList<QString>& keyList);
    bool sendDumpRosparams(const QString& path);
    bool sendLoadRosparams(const QString& path);
    bool sendSetMaintenanceMode(const bool on);
    bool sendExitDockingMode(const unsigned char mode);
    bool sendExitDockingModeFinish();
    bool sendExitDockingModeCancel();
    bool sendDuty(const QList<double>& duty);
    bool sendMarkerCorrection();
    bool sendLedLeftColors(const QList<QList<float>>& colors);
    bool sendLedRightColors(const QList<QList<float>>& colors);
    bool sendDisplayManagerSwitch(const bool on);
    bool sendLighting(const bool on);
    bool sendForcedRelease();
    bool sendReboot();
    bool sendSetOperationType(platform_msgs::OperationType type);
    bool sendUserNode(const bool on, const QString& user, const QString& launch, const QString& image);
    bool sendUserLogic(const bool on, platform_msgs::UserLogic);

    communication_software::Message createMessageBaseForDock(const dock::telecommand::Index index);
    bool sendDockSetHostIPAddr(const QHostAddress& addr);
    bool sendDockSetIBIPAddr(const QHostAddress& addr);
    bool sendDockSetCommandPort(const unsigned short port);
    bool sendDockMotorOnOff(const dock::telecommand::MOTOR_ON_OFF_TYPE type);
    bool sendDockChargeOnOff(const dock::telecommand::CHARGE_ON_OFF_TYPE type);

signals:
    void executed(CommandLog log);

private:
    ros::ServiceClient client_;
    QScopedPointer<CommunicationConfig> communicationConfig_;
};

}

Q_DECLARE_METATYPE(intball::RosParamType);

#endif // TELECOMMAND_PUBLISHER_H
