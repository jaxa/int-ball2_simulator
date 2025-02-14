#include <ros/ros.h>
#include "dock_telemetry.h"
#include "telemetry_telecommand_config.h"
#include "utils.h"

using namespace intball;
using namespace intball::dock::telemetry;

DockTelemetry::DockTelemetry(QObject* parent) : QAbstractListModel(parent)
{
    // データ領域の初期化.
    for(auto i = dock::telemetry::Config.keyBegin(); i != dock::telemetry::Config.keyEnd(); ++i)
    {
        data_.insert(createIndex(static_cast<int>(*i), 0),
                     dock::telemetry::Config[*i].defaultValue);
    }
}

QModelIndex DockTelemetry::index(int row, int column, const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return createIndex(row, column);
}

QModelIndex DockTelemetry::parent(const QModelIndex &child) const
{
    Q_UNUSED(child);
    return QModelIndex();
}

int DockTelemetry::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return data_.size();
}

int DockTelemetry::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 1;
}

QVariant DockTelemetry::data(const QModelIndex &index, int role) const
{
    Q_UNUSED(role);
    return data_[index];
}

QVariant DockTelemetry::data(dock::telemetry::Index item) const
{
    return data_[createIndex(static_cast<int>(item), 0)];
}

bool DockTelemetry::setData(const QModelIndex &index, const QVariant &value, int role)
{
    Q_UNUSED(role);
    if (index.isValid())
    {
        data_[index] = value;
        emit dataChanged(index, index);
        return true;
    }
    return false;
}

bool DockTelemetry::setData(const QMap<dock::telemetry::Index, QVariant>& data)
{
    for(auto i = data.keyBegin(); i != data.keyEnd(); ++i)
    {
        // 引数で渡されたQVariantのデータの型が,想定するデータ型と異なっている場合はエラー.
        Q_ASSERT_X(data[*i].type() == data_[createIndex(static_cast<int>(*i), 0)].type(),
                   __FUNCTION__,
                   QString("data[*i]=%1 data_[createIndex(static_cast<int>(*i), 0)]=%2")
                   .arg(data[*i].type())
                   .arg(data_[createIndex(static_cast<int>(*i), 0)].type())
                   .toStdString().c_str());
        data_.insert(createIndex(static_cast<int>(*i), 0), data[*i]);
    }
    emit dataChanged(createIndex(0, 0), createIndex(data_.size()-1, 0));
    return true;
}

void DockTelemetry::setReceivedTimestamp(const ros::Time& receivedTimestamp)
{
    receivedTimestamp_ = rosToQt(receivedTimestamp);
}

QString DockTelemetry::getReceivedTimestampString()
{
    return dateTimeString(receivedTimestamp_);
}

QString DockTelemetry::getChargeStateAsString()
{
    auto status = data<unsigned char>(Index::CHARGE_STATE);

    QString templateString;
    switch (status)
    {
    case CHARGE_STATE_TYPE::ON:
        templateString = "ON(%1)";
        break;
    case CHARGE_STATE_TYPE::OFF:
        templateString = "OFF(%1)";
        break;
    default:
        templateString = "(Invalid status)(%1)";
    }

    return templateString.arg(status);
}

QString DockTelemetry::getMotorStateAsString()
{
    auto status = data<unsigned char>(Index::MOTOR_STATE);

    QString templateString;
    switch (status)
    {
    case MOTOR_STATE_TYPE::STOP:
        templateString = "STOP(%1)";
        break;
    case MOTOR_STATE_TYPE::DOCKING:
        templateString = "DOCKING(%1)";
        break;
    case MOTOR_STATE_TYPE::RELEASE:
        templateString = "RELEASE(%1)";
        break;
    default:
        templateString = "(Invalid status)(%1)";
    }

    return templateString.arg(status);
}

QString DockTelemetry::getSwitchStateAsString()
{
    auto status = data<unsigned char>(Index::SWITCH_STATE);

    QString templateString;
    switch (status)
    {
    case SWITCH_STATE_TYPE::ALL_OFF:
        templateString = "ALL_OFF(%1)";
        break;
    case SWITCH_STATE_TYPE::DONE_DOCKING:
        templateString = "DONE_DOCKING(%1)";
        break;
    case SWITCH_STATE_TYPE::DONE_RELEASE:
        templateString = "DONE_RELEASE(%1)";
        break;
    case SWITCH_STATE_TYPE::ALL_ON:
        templateString = "ALL_ON(%1)";
        break;
    default:
        templateString = "(Invalid status)(%1)";
    }

    return templateString.arg(status);
}

QString DockTelemetry::getCommandStateAsString()
{
    auto status = data<unsigned char>(Index::COMMAND_STATE);

    QString templateString;
    switch (status)
    {
    case COMMAND_STATE_TYPE::SUCCESS:
        templateString = "SUCCESS(%1)";
        break;
    case COMMAND_STATE_TYPE::FAILED:
        templateString = "FAILED(%1)";
        break;
    case COMMAND_STATE_TYPE::UNKNOWN:
        templateString = "UNKNOWN(%1)";
        break;
    default:
        templateString = "(Invalid status)(%1)";
    }

    return templateString.arg(status);
}

QString DockTelemetry::getTempAlertAsString()
{
    auto status = data<unsigned char>(Index::TEMP_ALERT);

    QString templateString;
    switch (status)
    {
    case TEMP_ALERT_TYPE::NORMAL:
        templateString = "NORMAL(%1)";
        break;
    case TEMP_ALERT_TYPE::ALERT_MOTOR_TEMP:
        templateString = "ALERT_MOTOR_TEMP(%1)";
        break;
    case TEMP_ALERT_TYPE::ALERT_DCDC_TEMP:
        templateString = "ALERT_DCDC_TEMP(%1)";
        break;
    case TEMP_ALERT_TYPE::ALERT_ALL:
        templateString = "ALERT_ALL(%1)";
        break;
    default:
        templateString = "(Invalid status)(%1)";
    }

    return templateString.arg(status);
}

QString DockTelemetry::getChargeTimeAsString()
{
    auto time = data<unsigned short>(Index::CHARGE_TIME);
    return QString("%1:%2").arg(time / 60).arg(time % 60);
}

bool DockTelemetry::isCharging()
{
    return data<unsigned char>(Index::CHARGE_STATE) == CHARGE_STATE_TYPE::ON;
}
