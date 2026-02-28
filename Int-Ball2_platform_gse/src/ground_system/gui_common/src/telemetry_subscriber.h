#ifndef TELEMETRY_SUBSCRIBER_H
#define TELEMETRY_SUBSCRIBER_H

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include "communication_software/msg/telemetry.hpp"
#include "model/intball_telemetry.h"
#include "model/dock_telemetry.h"

Q_DECLARE_METATYPE(communication_software::msg::Telemetry::SharedPtr);

namespace intball
{

class TelemetrySubscriber : public QObject
{
    Q_OBJECT
public:
    static const std::string TOPIC_NAME_INTBALL2;
    static const std::string TOPIC_NAME_DOCK;

    explicit TelemetrySubscriber(QObject* parent = nullptr);
    virtual ~TelemetrySubscriber();

    void start(rclcpp::Node::SharedPtr node, IntBallTelemetry* telemetryIntball2, DockTelemetry* telemetryDock);

signals:
    void subscribedIntBall2(const communication_software::msg::Telemetry::SharedPtr msg);
    void subscribedDock(const communication_software::msg::Telemetry::SharedPtr msg);

private slots:
    void parseIntball2Telemetry(const communication_software::msg::Telemetry::SharedPtr msg);
    void parseDockTelemetry(const communication_software::msg::Telemetry::SharedPtr msg);

private:
    rclcpp::Subscription<communication_software::msg::Telemetry>::SharedPtr subscriberIntBall2_;
    IntBallTelemetry* telemetryIntBall2_;
    rclcpp::Subscription<communication_software::msg::Telemetry>::SharedPtr subscriberDock_;
    DockTelemetry* telemetryDock_;
};

}

#endif // TELEMETRY_SUBSCRIBER_H
