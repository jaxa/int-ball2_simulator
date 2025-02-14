#ifndef DOCK_TELEMETRY_H
#define DOCK_TELEMETRY_H

#include <ros/ros.h>
#include <QAbstractListModel>
#include <QDateTime>
#include "telemetry_telecommand_config.h"

namespace intball
{

class DockTelemetry : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit DockTelemetry(QObject* parent = nullptr);
    virtual ~DockTelemetry()override = default;
    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = 0) const override;
    QVariant data(dock::telemetry::Index item) const;
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    bool setData(const QMap<dock::telemetry::Index, QVariant>& data);

    template<typename T>
    T data(dock::telemetry::Index item) const
    {
        Q_ASSERT(data_[createIndex(static_cast<int>(item), 0)].canConvert<T>());
        return data_[createIndex(static_cast<int>(item), 0)].value<T>();
    }

    void setReceivedTimestamp(const ros::Time& receivedTimestamp);

    QString getReceivedTimestampString();
    QString getChargeStateAsString();
    QString getMotorStateAsString();
    QString getSwitchStateAsString();
    QString getCommandStateAsString();
    QString getTempAlertAsString();
    QString getChargeTimeAsString();
    bool isCharging();
private:
    QDateTime receivedTimestamp_;
    QMap<QModelIndex, QVariant> data_;
};

}

#endif // DOCK_TELEMETRY_H
