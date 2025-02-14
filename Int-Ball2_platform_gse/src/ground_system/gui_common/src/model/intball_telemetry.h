#ifndef INTBALL_TELEMETRY_H
#define INTBALL_TELEMETRY_H

#include <QAbstractListModel>
#include <QDateTime>
#include <QString>
#include <ros/time.h>
#include "communication_config.h"
#include "ib2_msgs.h"
#include "telemetry_telecommand_config.h"

namespace intball
{

class IntBallTelemetry : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit IntBallTelemetry(QObject* parent = nullptr);
    virtual ~IntBallTelemetry()override = default;
    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = 0) const override;
    QVariant data(telemetry::Index item) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    bool setData(const QMap<telemetry::Index, QVariant>& data);
    void setReceivedTimestamp(const ros::Time& receivedTimestamp);

    template<typename T>
    T data(telemetry::Index item) const
    {
        Q_ASSERT_X(data_[createIndex(static_cast<int>(item), 0)].canConvert<T>(),
                   __FUNCTION__,
                   QString("Can't convert index %1").arg(static_cast<int>(item)).toStdString().c_str());
        return data_[createIndex(static_cast<int>(item), 0)].value<T>();
    }

    int getCtlStatus() const;
    QString getCtlStatusAsString() const;
    QString getReceivedTimestampString() const;
    QString getTimestampString() const;
    QString getLastExecutedCommandString() const;
    unsigned char getSplitNumber() const;
    unsigned char getCurrentSplitIndex() const;
    unsigned char getSendingPortIndex() const;

    unsigned char getMode() const;
    QString getModeAsString() const;
    QVector3D getPosition(const telemetry::Index item) const;
    void getOrientationAsRPY(const telemetry::Index item, qreal& roll, qreal& pitch, qreal& yaw) const;
    bool getMarkerStatus() const;
    bool isGuidanceControlRunning() const;
    bool isGuidanceControlStopped() const;
    bool isFlightSoftwareStarted() const;
    bool isPlatformFlightSoftwareStarted() const;
    unsigned char getPlatformMode() const;
    QString getPlatformModeAsString() const;
    unsigned char getPlatformOperationType() const;
    QString getPlatformOperationTypeAsString() const;
    QDateTime getPlatformMonitorCheckTimeAsQDateTime() const;
    QVector<NodeStatus> getPlatformMonitorPublications() const;
    QVector<NodeStatus> getPlatformMonitorSubscriptions() const;
    QVector<NodeStatus> getPlatformMonitorServices() const;
    QVector<ContainerStatus> getPlatformMonitorContainers() const;
    QString getUserNodeStatusTimestampString() const;
    QVector<char> getUserNodeStatusMessage() const;
    QString getLastUserAsString() const;
    QString getLastLaunchAsString() const;
    QString getLastImageAsString() const;
    unsigned short getLastLogic() const;

    bool getInsertStatus(telemetry::Index index) const;

signals:    
    /**
     * @brief データ（値）が変更された.
     * @param rows 変更されたデータのインデックス.
     */
    void rowsChanged(QList<int> rows);

    /**
     * @brief データの書き込みが実施された.
     * @param rows 書き込まれたデータのインデックス.
     */
    void written(QList<int> rows);

private:
    /**
     * @brief テレメトリ受信時のタイムスタンプ.
     */
    QDateTime receivedTimestamp_;

    /**
     * @brief 受信したテレメトリ内容.
     */
    QMap<QModelIndex, QVariant> data_;

    /**
     * @brief テレメトリの受信状況.
     */
    QMap<QModelIndex, bool> insertStatus_;

    /**
     * @brief 通信変換の設定情報.
     */
    QScopedPointer<CommunicationConfig> communicationConfig_;
};

}

#endif // INTBALL_TELEMETRY_H
