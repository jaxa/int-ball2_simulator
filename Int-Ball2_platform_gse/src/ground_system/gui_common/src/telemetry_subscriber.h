#ifndef TELEMETRY_SUBSCRIBER_H
#define TELEMETRY_SUBSCRIBER_H

#include <QObject>
#include <ros/ros.h>
#include "communication_software/Telemetry.h"
#include "model/intball_telemetry.h"
#include "model/dock_telemetry.h"

Q_DECLARE_METATYPE(communication_software::Telemetry::ConstPtr);

namespace intball
{

class TelemetrySubscriber : public QObject
{
    Q_OBJECT
public:
    /**
     * @brief IntBall2テレメトリのトピック名.
     */
    static const std::string TOPIC_NAME_INTBALL2;

    /**
     * @brief ドッキングステーションテレメトリのトピック名.
     */
    static const std::string TOPIC_NAME_DOCK;

    /**
     * @brief TelemetrySubscriberコンストラクタ.
     * @param parent 親ウィジット.
     */
    explicit TelemetrySubscriber(QObject* parent = nullptr);

    /**
     * @brief TelemetrySubscriberデストラクタ.
     */
    virtual ~TelemetrySubscriber();

    /**
     * @brief テレメトリの受信処理（Subscribe）開始.
     * @param nodeHandle Subscribe用ROS NodeHandle.
     * @param telemetryIntball2 受信したInt-Ball2テレメトリの格納先.
     * @param telemetryDock 受信したドッキングステーションテレメトリの格納先.
     */
    void start(ros::NodeHandle& nodeHandle, IntBallTelemetry* telemetryIntball2, DockTelemetry* telemetryDock);
    void intball2TelemetryCallback(const communication_software::Telemetry::ConstPtr& msg);
    void dockTelemetryCallback(const communication_software::Telemetry::ConstPtr& msg);

signals:
    void subscribedIntBall2(const communication_software::Telemetry::ConstPtr msg);
    void subscribedDock(const communication_software::Telemetry::ConstPtr msg);

private slots:
    void parseIntball2Telemetry(const communication_software::Telemetry::ConstPtr msg);
    void parseDockTelemetry(const communication_software::Telemetry::ConstPtr msg);


private:
    ros::Subscriber subscriberIntBall2_;
    IntBallTelemetry* telemetryIntBall2_;
    ros::Subscriber subscriberDock_;
    DockTelemetry* telemetryDock_;
};

}

#endif // TELEMETRY_SUBSCRIBER_H
