#ifndef TELECOMMAND_PUBLISHER_H
#define TELECOMMAND_PUBLISHER_H

#include <QHostAddress>
#include <QObject>
#include <QScopedPointer>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include "camera_config.h"
#include "communication_config.h"
#include "communication_software/msg/message.hpp"
#include "communication_software/srv/telecommand.hpp"
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

    TelecommandClient(rclcpp::Node::SharedPtr node, QObject* parent = nullptr);
    virtual ~TelecommandClient();

    bool send(const communication_software::msg::Message& command);

    template<typename T>
    bool send(const std::string& name, const T& data)
    {
        INFO_START_FUNCTION();

        communication_software::msg::Message command;
        command.msg_type = communication_software::msg::Message::INTBALL2_SERIALIZED_BINARY_DATA;
        command.name = name;

        // ROS 2 serialization
        rclcpp::Serialization<T> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(&data, &serialized_msg);
        auto& rcl_msg = serialized_msg.get_rcl_serialized_message();
        command.data.assign(rcl_msg.buffer, rcl_msg.buffer + rcl_msg.buffer_length);

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
    bool sendSetOperationType(platform_msgs::msg::OperationType type);
    bool sendUserNode(const bool on, const QString& user, const QString& launch, const QString& image);
    bool sendUserLogic(const bool on, platform_msgs::msg::UserLogic);

    communication_software::msg::Message createMessageBaseForDock(const dock::telecommand::Index index);
    bool sendDockSetHostIPAddr(const QHostAddress& addr);
    bool sendDockSetIBIPAddr(const QHostAddress& addr);
    bool sendDockSetCommandPort(const unsigned short port);
    bool sendDockMotorOnOff(const dock::telecommand::MOTOR_ON_OFF_TYPE type);
    bool sendDockChargeOnOff(const dock::telecommand::CHARGE_ON_OFF_TYPE type);

signals:
    void executed(CommandLog log);

private:
    rclcpp::Client<communication_software::srv::Telecommand>::SharedPtr client_;
    QScopedPointer<CommunicationConfig> communicationConfig_;
    rclcpp::Node::SharedPtr node_;
};

}

Q_DECLARE_METATYPE(intball::RosParamType);

#endif // TELECOMMAND_PUBLISHER_H
