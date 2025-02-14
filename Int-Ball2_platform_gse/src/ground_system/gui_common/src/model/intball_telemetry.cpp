#include <QAbstractListModel>
#include <QDebug>
#include "ib2_msgs.h"
#include "model/intball_telemetry.h"
#include "qdebug_custom.h"
#include "utils.h"

using namespace intball;
using namespace ib2_msgs;

IntBallTelemetry::IntBallTelemetry(QObject* parent) : QAbstractListModel(parent)
{
    // データ領域の初期化.
    for(auto i = telemetry::Config.keyBegin(); i != telemetry::Config.keyEnd(); ++i)
    {
        data_.insert(createIndex(static_cast<int>(*i), 0),
                     telemetry::Config[*i].defaultValue);
        insertStatus_.insert(createIndex(static_cast<int>(*i), 0), false);
    }

    communicationConfig_.reset(new CommunicationConfig());
}

QModelIndex IntBallTelemetry::index(int row, int column, const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return createIndex(row, column);
}

QModelIndex IntBallTelemetry::parent(const QModelIndex &child) const
{
    Q_UNUSED(child);
    return QModelIndex();
}

int IntBallTelemetry::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return data_.size();
}

int IntBallTelemetry::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 1;
}

QVariant IntBallTelemetry::data(const QModelIndex &index, int role) const
{
    Q_UNUSED(role);
    return data_[index];
}

QVariant IntBallTelemetry::data(telemetry::Index item) const
{
    return data_[createIndex(static_cast<int>(item), 0)];
}

bool IntBallTelemetry::setData(const QModelIndex &index, const QVariant &value, int role)
{
    Q_UNUSED(role);
    bool valueChanged = false;
    if (index.isValid())
    {
        valueChanged = (value != data_[index]);
        data_[index] = value;
        // データ受信状況の更新.
        if(!insertStatus_[index]) {
            insertStatus_.insert(index, true);
        }
        // Qtのモデル用シグナル.
        emit dataChanged(index, index);

        /*
         * 独自シグナル.
         */
        QList<int> rowList;
        rowList.push_back(index.row());
        // 値の書き込みを実施.
        emit written(rowList);
        if(valueChanged)
        {
            // 値が変更された.
            emit rowsChanged(rowList);
        }
        return true;
    }
    return false;
}

bool IntBallTelemetry::setData(const QMap<telemetry::Index, QVariant>& data)
{
    QList<int> writtenList;
    QList<int> changedList;
    bool valueChanged = false;
    for(auto i = data.keyBegin(); i != data.keyEnd(); ++i)
    {
        if(data[*i].type() != data_[createIndex(static_cast<int>(*i), 0)].type())
        {
            // 引数で渡されたQVariantのデータの型が,想定するデータ型と異なっている場合は実装レベルのエラー.
            // iの値（telemetry::Index）が示す設定の誤り.
            QString logMessage = QString("telemetry::Index=%1 data[*i]=%2 data_[createIndex(static_cast<int>(*i), 0)]=%3")
                    .arg(static_cast<int>(*i))
                    .arg(data[*i].type())
                    .arg(data_[createIndex(static_cast<int>(*i), 0)].type());
            LOG_CRITICAL() << logMessage;
            Q_ASSERT_X(true, __FUNCTION__,
                       logMessage.toStdString().c_str());
        }

        valueChanged = (data_[createIndex(static_cast<int>(*i), 0)] != data[*i]);
        data_.insert(createIndex(static_cast<int>(*i), 0), data[*i]);
        // データ受信状況の更新.
        if(!insertStatus_[createIndex(static_cast<int>(*i), 0)]) {
            insertStatus_.insert(createIndex(static_cast<int>(*i), 0), true);
        }
        writtenList.push_back(static_cast<int>(*i));
        if(valueChanged)
        {
            changedList.push_back(static_cast<int>(*i));
        }
    }
    // Qtのモデル用シグナル.
    emit dataChanged(createIndex(0, 0), createIndex(data_.size()-1, 0));
    /*
     * 独自シグナル.
     */
    // 値の書き込みを実施.
    if(writtenList.size() > 0)
    {
        emit written(writtenList);
    }
    if(changedList.size() > 0)
    {
        // 値が変更された.
        emit rowsChanged(changedList);
    }
    return true;
}

void IntBallTelemetry::setReceivedTimestamp(const ros::Time& receivedTimestamp)
{
    receivedTimestamp_ = rosToQt(receivedTimestamp);
}

QString IntBallTelemetry::getReceivedTimestampString() const
{
    return dateTimeString(receivedTimestamp_);
}

QString IntBallTelemetry::getTimestampString() const
{
    return dateTimeString(data<ros::Time>(telemetry::Index::TIMESTAMP));
}

QString IntBallTelemetry::getLastExecutedCommandString() const
{
    auto msgId = data<unsigned short>(telemetry::Index::LAST_EXECUTED_COMMAND);
    return communicationConfig_->getTelecommandNameById(msgId);
}

unsigned char IntBallTelemetry::getSplitNumber() const
{
    return data<unsigned char>(telemetry::Index::SPLIT_NUMBER);
}

unsigned char IntBallTelemetry::getCurrentSplitIndex() const
{
    return data<unsigned char>(telemetry::Index::CURRENT_SPILIT_INDEX);
}

unsigned char IntBallTelemetry::getSendingPortIndex() const
{
    return data<unsigned char>(telemetry::Index::SENDING_PORT_INDEX);
}

QVector3D IntBallTelemetry::getPosition(const telemetry::Index item) const
{
    return geometryToQt(data<geometry_msgs::Point>(item));
}

void IntBallTelemetry::getOrientationAsRPY(const telemetry::Index item, qreal& roll, qreal& pitch, qreal& yaw) const
{
    auto orientation = data<geometry_msgs::Quaternion>(item);
    getRPY(geometryToQt(orientation), roll, pitch, yaw);
}

bool IntBallTelemetry::getMarkerStatus() const
{
    auto status = data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS);
    return status.marker;
}

int IntBallTelemetry::getCtlStatus() const
{
    return data<int>(telemetry::Index::CTL_STATUS_TYPE);
}

QString IntBallTelemetry::getCtlStatusAsString() const
{
    return intball::getCtlStatusAsString(data(telemetry::Index::CTL_STATUS_TYPE));
}

bool IntBallTelemetry::getInsertStatus(telemetry::Index index) const
{
    return insertStatus_[createIndex(static_cast<int>(index), 0)];
}

unsigned char IntBallTelemetry::getMode() const
{
    return data<unsigned char>(telemetry::Index::MODE);
}

QString IntBallTelemetry::getModeAsString() const
{
    return getModeString(data(telemetry::Index::MODE));
}

bool IntBallTelemetry::isGuidanceControlRunning() const
{
    auto lastResultTimestamp = data<ros::Time>(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP);
    auto lastFeedbackTimestamp = data<ros::Time>(telemetry::Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP);
    ros::Duration diff = lastFeedbackTimestamp - lastResultTimestamp;
    return (diff.toSec() > 0);
}

bool IntBallTelemetry::isGuidanceControlStopped() const
{
    auto lastResultTimestamp = data<ros::Time>(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP);
    auto lastFeedbackTimestamp = data<ros::Time>(telemetry::Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP);
    ros::Duration diff = lastResultTimestamp - lastFeedbackTimestamp;
    return (diff.toSec() > 0);
}

bool IntBallTelemetry::isFlightSoftwareStarted() const
{
    return data<bool>(telemetry::Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS);
}

bool IntBallTelemetry::isPlatformFlightSoftwareStarted() const
{
    return data<bool>(telemetry::Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS);
}

unsigned char IntBallTelemetry::getPlatformMode() const
{
    return data<unsigned char>(telemetry::Index::PLATFORM_MANAGER_MODE);
}

QString IntBallTelemetry::getPlatformModeAsString() const
{
    return getPlatformModeString(data(telemetry::Index::PLATFORM_MANAGER_MODE));
}

unsigned char IntBallTelemetry::getPlatformOperationType() const
{
    return data<unsigned char>(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE);
}

QDateTime IntBallTelemetry::getPlatformMonitorCheckTimeAsQDateTime() const
{
    auto checkTime = data<ros::Time>(telemetry::Index::PLATFORM_MONITOR_CHECK_TIME);
    return rosToQt(checkTime);
}

QVector<NodeStatus> IntBallTelemetry::getPlatformMonitorPublications() const
{
    return data<QVector<NodeStatus>>(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS);
}

QVector<NodeStatus> IntBallTelemetry::getPlatformMonitorSubscriptions() const
{
    return data<QVector<NodeStatus>>(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS);
}

QVector<NodeStatus> IntBallTelemetry::getPlatformMonitorServices() const
{
    return data<QVector<NodeStatus>>(telemetry::Index::PLATFORM_MONITOR_SERVICES);
}

QVector<ContainerStatus> IntBallTelemetry::getPlatformMonitorContainers() const
{
    return data<QVector<ContainerStatus>>(telemetry::Index::PLATFORM_MONITOR_CONTAINERS);
}

QString IntBallTelemetry::getUserNodeStatusTimestampString() const
{
    return dateTimeString(data<ros::Time>(telemetry::Index::USER_NODE_STATUS_TIMESTAMP));
}

QVector<char> IntBallTelemetry::getUserNodeStatusMessage() const
{
    return data<QVector<char>>(telemetry::Index::USER_NODE_STATUS_MESSAGE);
}

QString IntBallTelemetry::getPlatformOperationTypeAsString() const
{
    return getPlatformOperationTypeString(data(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE));
}

QString IntBallTelemetry::getLastUserAsString() const
{
    return QString::fromStdString(data<std::string>(telemetry::Index::PLATFORM_MANAGER_LAST_USER));
}

QString IntBallTelemetry::getLastLaunchAsString() const
{
    return QString::fromStdString(data<std::string>(telemetry::Index::PLATFORM_MANAGER_LAST_LAUNCH));
}

QString IntBallTelemetry::getLastImageAsString() const
{
    return QString::fromStdString(data<std::string>(telemetry::Index::PLATFORM_MANAGER_LAST_IMAGE));
}

unsigned short IntBallTelemetry::getLastLogic() const
{
    return data<unsigned short>(telemetry::Index::PLATFORM_MANAGER_LAST_LOGIC);
}
