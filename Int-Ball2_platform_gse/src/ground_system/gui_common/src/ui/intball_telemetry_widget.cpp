#include "intball_telemetry_widget.h"
#include "ui_intball_telemetry_widget.h"
#include <QFormLayout>
#include <QTableWidget>
#include <ros/ros.h>
#include "camera_config.h"
#include "model/intball_telemetry.h"
#include "qdebug_custom.h"
#include "utils.h"

using namespace intball;

namespace {
void updateRosParamTableWidget(const QString& key, const QMap<QString, RosParam>& map, QTableWidget* target)
{
    QTableWidgetItem* targetItemTimestamp;
    QTableWidgetItem* targetItemKey;
    QTableWidgetItem* targetItemValue;
    // 表の1列目はタイムスタンプ, 2列目はkey, 3列目はvalue.
    auto itemList = target->findItems(key, Qt::MatchFlag::MatchFixedString);
    if(itemList.size() > 0)
    {
        targetItemKey = itemList.at(0);
        targetItemTimestamp = target->item(targetItemKey->row(), 0);
        targetItemValue = target->item(targetItemKey->row(), 2);
    }
    else
    {
        target->insertRow(0);
        target->setItem(0, 0, new QTableWidgetItem());
        target->setItem(0, 1, new QTableWidgetItem());
        target->setItem(0, 2, new QTableWidgetItem());

        targetItemTimestamp = target->item(0, 0);
        targetItemKey = target->item(0, 1);
        targetItemValue = target->item(0, 2);
        targetItemKey->setText(key);
    }
    targetItemTimestamp->setText(dateTimeString(map.value(key).stamp));
    targetItemValue->setText(QString::fromStdString(map.value(key).value));
}

}

IntBallTelemetryWidget::IntBallTelemetryWidget(QWidget *parent) :
    QAbstractItemView(parent),
    ui(new Ui::IntBallTelemetryWidget)
{
    ui->setupUi(this);

    cameraConfig_.reset(new CameraConfig);

    ui->tableWidgetDiskSpaces->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    QStringList diskSpacesHeaderLabels;
    diskSpacesHeaderLabels.append("Path");
    diskSpacesHeaderLabels.append("Free space [%]");
    ui->tableWidgetDiskSpaces->setHorizontalHeaderLabels(diskSpacesHeaderLabels);

    ui->tableWidgetAliveStatuses->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetCtlParameters->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetNavigationParameters->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetLedLeftParameters->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetLedRightParameters->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    QStringList rosparamsLabel;
    rosparamsLabel.append("Timestamp");
    rosparamsLabel.append("Key");
    rosparamsLabel.append("Value");
    ui->tableWidgetGetRosparamsValue->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetGetRosparamsValue->setHorizontalHeaderLabels(rosparamsLabel);
    ui->tableWidgetSetRosparamsValue->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->tableWidgetSetRosparamsValue->setHorizontalHeaderLabels(rosparamsLabel);
}

IntBallTelemetryWidget::~IntBallTelemetryWidget()
{
    delete ui;
}

QRect IntBallTelemetryWidget::visualRect(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return QRect(0, 0, width(), height());
}

void IntBallTelemetryWidget::scrollTo(const QModelIndex &index, ScrollHint hint)
{
    Q_UNUSED(index);
    Q_UNUSED(hint);
}

QModelIndex IntBallTelemetryWidget::indexAt(const QPoint &point) const
{
    Q_UNUSED(point);
    return QModelIndex();
}

IntBallTelemetry* IntBallTelemetryWidget::thisModel()
{
    Q_ASSERT(dynamic_cast<IntBallTelemetry*>(model()) != nullptr);
    return dynamic_cast<IntBallTelemetry*>(model());
}

void IntBallTelemetryWidget::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    qreal roll, pitch, yaw;

    auto publishParameterMap = thisModel()->data<QMap<QString, QString>>(telemetry::Index::PUBLISHING_PARAMS_LIST);

    /*
     * Header.
     */
    if(thisModel()->getInsertStatus(telemetry::Index::TIMESTAMP))
    {
        char sendingPortIndex = static_cast<char>(thisModel()->getSendingPortIndex());
        if(!headerWidgets_.keys().contains(sendingPortIndex))
        {
            // 新規の送信元ポートインデックス値を受信したためWidgetを追加
            headerWidgets_.insert(
                        sendingPortIndex,
                        new IntBallTelemetryHeaderWidget(this)
                        );
            ui->groupHeader->layout()->addWidget(headerWidgets_[sendingPortIndex]);
        }

        headerWidgets_[sendingPortIndex]->setLabelValues(
                    thisModel()->getTimestampString(),
                    thisModel()->getLastExecutedCommandString(),
                    QString::number(thisModel()->getCurrentSplitIndex()),
                    QString::number(thisModel()->getSplitNumber()),
                    QString::number(thisModel()->getSendingPortIndex())
                    );
    }

    /*
     * Flight software
     */
    if(thisModel()->getInsertStatus(telemetry::Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS))
        ui->labelFlightSoftwareStatusValue->setText(thisModel()->isFlightSoftwareStarted() ? "ON" : "OFF");

    /*
     * Task manager
     */

    // Mode.
    if(thisModel()->getInsertStatus(telemetry::Index::MODE))
        ui->labelIntBallModeValue->setText(getModeString(thisModel()->data(telemetry::Index::MODE)));

    // Exit docking mode.
    if(thisModel()->getInsertStatus(telemetry::Index::EXIT_DOCKING_MODE_SUCCESS))
        ui->labelExitDockingModeSuccessValue->setText(getBoolAsString(thisModel()->data(telemetry::Index::EXIT_DOCKING_MODE_SUCCESS)));
    if(thisModel()->getInsertStatus(telemetry::Index::EXIT_DOCKING_MODE_RESULT_MODE))
     ui->labelExitDockingModeResultModeValue->setText(getIntBallModeAsString(thisModel()->data(telemetry::Index::EXIT_DOCKING_MODE_RESULT_MODE)));

    // Set maintenance mode.
    if(thisModel()->getInsertStatus(telemetry::Index::SET_MAINTENANCE_MODE_RESULT_MODE))
        ui->labelSetMaintenanceModeResultModeValue->setText(
                    getIntBallModeAsString(thisModel()->data(telemetry::Index::SET_MAINTENANCE_MODE_RESULT_MODE)));

    /*
     * System monitor.
     */
    if(thisModel()->getInsertStatus(telemetry::Index::SYSTEM_MONITOR_TIMESTAMP))
    {
        ui->labelSystemMonitorCheckTimeValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::SYSTEM_MONITOR_TIMESTAMP)));
        auto diskSpacesMap = thisModel()->data<QMap<QString, float>>(telemetry::Index::SYSTEM_MONITOR_DISK_SPACES);
        for(auto i = diskSpacesMap.keyBegin(); i != diskSpacesMap.keyEnd(); ++i)
        {
            auto itemList = ui->tableWidgetDiskSpaces->findItems(*i, Qt::MatchFlag::MatchFixedString);
            QTableWidgetItem* targetItemKey;
            QTableWidgetItem* targetItemValue;
            // 表の1列目はkey, 2列目はvalue.
            if(itemList.size() > 0)
            {
                targetItemKey = ui->tableWidgetDiskSpaces->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
                targetItemValue = ui->tableWidgetDiskSpaces->item(targetItemKey->row(), 1);
            }
            else
            {
                ui->tableWidgetDiskSpaces->insertRow(0);
                ui->tableWidgetDiskSpaces->setItem(0, 0, new QTableWidgetItem());
                ui->tableWidgetDiskSpaces->setItem(0, 1, new QTableWidgetItem());
                targetItemKey = ui->tableWidgetDiskSpaces->item(0, 0);
                targetItemValue = ui->tableWidgetDiskSpaces->item(0, 1);
                targetItemKey->setText(*i);
            }
            targetItemValue->setText(QString::number(static_cast<double>(diskSpacesMap.value(*i))));
        }
        ui->labelTemperatureValue->setNum(static_cast<double>(thisModel()->data<float>(telemetry::Index::SYSTEM_MONITOR_TEMPERATURE)));
        ui->labelWiFiConnectedValue->setText(getBoolAsString(thisModel()->data(telemetry::Index::SYSTEM_MONITOR_WIFI_CONNECTED)));
    }

    // バッテリー充電状態.
    if(thisModel()->getInsertStatus(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN))
    {
        auto batteryRemain = thisModel()->data<char>(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN);
        ui->labelBatteryRemainValue->setText(QString::number(batteryRemain));
    }

    /*
     * Alive status
     */

    // Topic.
    if(thisModel()->getInsertStatus(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC))
    {
        auto aliveStatusMapTopic = thisModel()->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC);
        for(auto i = aliveStatusMapTopic.keyBegin(); i != aliveStatusMapTopic.keyEnd(); ++i)
        {

            auto itemList = ui->tableWidgetAliveStatuses->findItems(*i, Qt::MatchFlag::MatchFixedString);
            QTableWidgetItem* targetItemName;
            QTableWidgetItem* targetItemResult;
            QTableWidgetItem* targetItemLatestValidTime;
            QTableWidgetItem* targetItemCheckTime;
            if(itemList.size() > 0)
            {
                targetItemName = ui->tableWidgetAliveStatuses->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
                targetItemResult = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 1);
                targetItemLatestValidTime = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 2);
                targetItemCheckTime = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 3);
            }
            else
            {
                ui->tableWidgetAliveStatuses->insertRow(0);
                ui->tableWidgetAliveStatuses->setItem(0, 0, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 1, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 2, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 3, new QTableWidgetItem());
                targetItemName = ui->tableWidgetAliveStatuses->item(0, 0);
                targetItemResult = ui->tableWidgetAliveStatuses->item(0, 1);
                targetItemLatestValidTime = ui->tableWidgetAliveStatuses->item(0, 2);
                targetItemCheckTime = ui->tableWidgetAliveStatuses->item(0, 3);
                targetItemName->setText(*i);
            }

            targetItemResult->setText(getAliveStatusResultString(aliveStatusMapTopic.value(*i).result));
            targetItemLatestValidTime->setText(dateTimeString(aliveStatusMapTopic.value(*i).latest_valid_time));
            targetItemCheckTime->setText(dateTimeString(aliveStatusMapTopic.value(*i).check_time));
        }
    }


    // Service.
    if(thisModel()->getInsertStatus(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE))
    {
        auto aliveStatusMapService = thisModel()->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE);
        for(auto i = aliveStatusMapService.keyBegin(); i != aliveStatusMapService.keyEnd(); ++i)
        {
            auto itemList = ui->tableWidgetAliveStatuses->findItems(*i, Qt::MatchFlag::MatchFixedString);
            QTableWidgetItem* targetItemName;
            QTableWidgetItem* targetItemResult;
            QTableWidgetItem* targetItemLatestValidTime;
            QTableWidgetItem* targetItemCheckTime;
            if(itemList.size() > 0)
            {
                targetItemName = ui->tableWidgetAliveStatuses->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
                targetItemResult = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 1);
                targetItemLatestValidTime = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 2);
                targetItemCheckTime = ui->tableWidgetAliveStatuses->item(targetItemName->row(), 3);
            }
            else
            {
                ui->tableWidgetAliveStatuses->insertRow(0);
                ui->tableWidgetAliveStatuses->setItem(0, 0, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 1, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 2, new QTableWidgetItem());
                ui->tableWidgetAliveStatuses->setItem(0, 3, new QTableWidgetItem());
                targetItemName = ui->tableWidgetAliveStatuses->item(0, 0);
                targetItemResult = ui->tableWidgetAliveStatuses->item(0, 1);
                targetItemLatestValidTime = ui->tableWidgetAliveStatuses->item(0, 2);
                targetItemCheckTime = ui->tableWidgetAliveStatuses->item(0, 3);
                targetItemName->setText(*i);
            }

            targetItemResult->setText(getAliveStatusResultString(aliveStatusMapService.value(*i).result));
            targetItemLatestValidTime->setText(dateTimeString(aliveStatusMapService.value(*i).latest_valid_time));
            targetItemCheckTime->setText(dateTimeString(aliveStatusMapService.value(*i).check_time));
        }
    }

    // テーブルにパラメータを追加した後にViewのリサイズ処理を呼び出す必要がある.
    ui->tableWidgetAliveStatuses->resizeColumnsToContents();

    /*
     * Guidance and control.
     */

    // Action result.
    if(thisModel()->getInsertStatus(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP))
    {
        ui->labelActionResultStampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP)));
        ui->labelActionResultTypeValue->setText(getCtlCommandResultAsString(thisModel()->data(telemetry::Index::CTL_ACTION_RESULT_TYPE)));
    }

    // Action feedback.
    if(thisModel()->getInsertStatus(telemetry::Index::CTL_ACTION_FEEDBACK_TIME_TO_GO))
    {
        ui->labelActionFeedbackTTGValue->setText(QString("%1 sec.").arg(thisModel()->data<ros::Duration>(telemetry::Index::CTL_ACTION_FEEDBACK_TIME_TO_GO).toSec()));

        auto feedbackPosition = thisModel()->getPosition(telemetry::Index::CTL_ACTION_FEEDBACK_POSE_POSITION);
        ui->labelActionFeedbackPositionXValue->setNum(static_cast<double>(feedbackPosition.x()));
        ui->labelActionFeedbackPositionYValue->setNum(static_cast<double>(feedbackPosition.y()));
        ui->labelActionFeedbackPositionZValue->setNum(static_cast<double>(feedbackPosition.z()));

        thisModel()->getOrientationAsRPY(telemetry::Index::CTL_ACTION_FEEDBACK_POSE_ORIENTATION, roll, pitch, yaw);
        auto ctlCommandFeedbackOrientation = geometryToQt(thisModel()->data<geometry_msgs::Quaternion>(telemetry::Index::CTL_ACTION_FEEDBACK_POSE_ORIENTATION));
        ui->labelActionFeedbackOrientationQuaternionXValue->setNum(static_cast<double>(ctlCommandFeedbackOrientation.x()));
        ui->labelActionFeedbackOrientationQuaternionYValue->setNum(static_cast<double>(ctlCommandFeedbackOrientation.y()));
        ui->labelActionFeedbackOrientationQuaternionZValue->setNum(static_cast<double>(ctlCommandFeedbackOrientation.x()));
        ui->labelActionFeedbackOrientationQuaternionWValue->setNum(static_cast<double>(ctlCommandFeedbackOrientation.scalar()));
        ui->labelActionFeedbackOrientationDegreeValue->setText(getDegreeAsString(ctlCommandFeedbackOrientation));
    }


    // Ctl status.
    if(thisModel()->getInsertStatus(telemetry::Index::CTL_STATUS_HEADER_SEQ))
    {
        ui->labelCtlStatusSequenceValue->setText(QString("%1").arg(thisModel()->data<unsigned int>(telemetry::Index::CTL_STATUS_HEADER_SEQ)));
        ui->labelCtlStatusStampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::CTL_STATUS_HEADER_STAMP)));
        ui->labelCtlStatusFrameIDValue->setText(QString::fromStdString(thisModel()->data<std::string>(telemetry::Index::CTL_STATUS_HEADER_FRAME_ID)));
        ui->labelCtlStatusStatusValue->setText(getCtlStatusAsString(thisModel()->data(telemetry::Index::CTL_STATUS_TYPE)));

        auto ctlStatusPosition = thisModel()->getPosition(telemetry::Index::CTL_STATUS_POSE_POSITION);
        ui->labelCtlStatusPositionValue->setText(QString("%1, %2, %3")
                .arg(static_cast<double>(ctlStatusPosition.x()))
                .arg(static_cast<double>(ctlStatusPosition.y()))
                .arg(static_cast<double>(ctlStatusPosition.z())));

        auto ctlStatusOrientation = geometryToQt(thisModel()->data<geometry_msgs::Quaternion>(telemetry::Index::CTL_STATUS_POSE_ORIENTATION));
        ui->labelCtlStatusOrientationQuaternionValue->setText(getQuaternionAsString(ctlStatusOrientation));
        ui->labelCtlStatusOrientationDegreeValue->setText(getDegreeAsString(ctlStatusOrientation));

        auto ctlStatusA = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::CTL_STATUS_A);
        ui->labelCtlStatusAValue->setText(QString("%1, %2, %3").arg(ctlStatusA.x).arg(ctlStatusA.y).arg(ctlStatusA.z));

        auto ctlStatusLinear = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::CTL_STATUS_TWIST_LINEAR);
        ui->labelCtlStatusLinearValue->setText(QString("%1, %2, %3").arg(ctlStatusLinear.x).arg(ctlStatusLinear.y).arg(ctlStatusLinear.z));

        auto ctlStatusAngular = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::CTL_STATUS_TWIST_ANGULAR);
        ui->labelCtlStatusAngularValue->setText(QString("%1, %2, %3").arg(ctlStatusAngular.x).arg(ctlStatusAngular.y).arg(ctlStatusAngular.z));
    }

    // Wrench.
    if(thisModel()->getInsertStatus(telemetry::Index::CTL_WRENCH_HEADER_SEQ))
    {
        ui->labelWrenchSequenceValue->setText(QString("%1").arg(thisModel()->data<unsigned int>(telemetry::Index::CTL_WRENCH_HEADER_SEQ)));
        ui->labelWrenchTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::CTL_WRENCH_HEADER_STAMP)));
        ui->labelWrenchFrameIDValue->setText(QString::fromStdString(thisModel()->data<std::string>(telemetry::Index::CTL_WRENCH_HEADER_FRAME_ID)));
        auto wrenchForce = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::CTL_WRENCH_FORCE);
        ui->labelWrenchForceValue->setText(QString("%1, %2, %3").arg(wrenchForce.x).arg(wrenchForce.y).arg(wrenchForce.z));
        auto wrenchTorque = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::CTL_WRENCH_TORQUE);
        ui->labelWrenchTorqueValue->setText(QString("%1, %2, %3").arg(wrenchTorque.x).arg(wrenchTorque.y).arg(wrenchTorque.z));
    }

    // Update parameter.
    if(thisModel()->getInsertStatus(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelCtlUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelCtlUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    // Parameters.
    for(auto i = publishParameterMap.keyBegin(); i != publishParameterMap.keyEnd(); ++i)
    {
        if(!(*i).startsWith(rosparam::PREFIX_CTL) && !(*i).startsWith(rosparam::PREFIX_POS_CTL) && !(*i).startsWith(rosparam::PREFIX_ATT_CTL))
        {
            continue;
        }

        auto itemList = ui->tableWidgetCtlParameters->findItems(*i, Qt::MatchFlag::MatchFixedString);
        QTableWidgetItem* targetItemKey;
        QTableWidgetItem* targetItemValue;
        // 表の1列目はkey, 2列目はvalue.
        if(itemList.size() > 0)
        {
            targetItemKey = ui->tableWidgetCtlParameters->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
            targetItemValue = ui->tableWidgetCtlParameters->item(targetItemKey->row(), 1);
        }
        else
        {
            ui->tableWidgetCtlParameters->insertRow(0);
            ui->tableWidgetCtlParameters->setItem(0, 0, new QTableWidgetItem());
            ui->tableWidgetCtlParameters->setItem(0, 1, new QTableWidgetItem());
            targetItemKey = ui->tableWidgetCtlParameters->item(0, 0);
            targetItemValue = ui->tableWidgetCtlParameters->item(0, 1);
            targetItemKey->setText(*i);
        }

        targetItemValue->setText(publishParameterMap.value(*i));
    }

    /*
     * Propulsion.
     */
    if(thisModel()->getInsertStatus(telemetry::Index::PROP_STATUS_HEADER_STAMP))
    {
        ui->labelPropStatusTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::PROP_STATUS_HEADER_STAMP)));
        ui->labelPropStatusPowerValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::PROP_STATUS_POWER)));
        auto dutyList = thisModel()->data<std::vector<double>>(telemetry::Index::PROP_STATUS_DUTY);
        QString dutyString = "";
        for(auto i = dutyList.begin(); i != dutyList.end(); ++i)
        {
            dutyString += QString("%1 ").arg(*i);
        }
        ui->labelPropStatusDutyValue->setText(dutyString);
    }

    if(thisModel()->getInsertStatus(telemetry::Index::PROP_SWITCH_POWER_RESPONSE))
    {
        ui->labelPropSwitchPowerValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::PROP_SWITCH_POWER_RESPONSE)));
    }

    // Update parameter.
    if(thisModel()->getInsertStatus(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelPropUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelPropUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    /*
     * Navigation.
     */
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_HEADER_STAMP))
    {
        ui->labelNavigationStampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::NAVIGATION_HEADER_STAMP)));
        ui->labelNavigationSequenceValue->setText(QString("%1").arg(thisModel()->data<unsigned int>(telemetry::Index::NAVIGATION_HEADER_SEQ)));
        ui->labelNavigationFrameIDValue->setText(QString::fromStdString(thisModel()->data<std::string>(telemetry::Index::NAVIGATION_HEADER_FRAME_ID)));

        auto navigationPosition = thisModel()->getPosition(telemetry::Index::NAVIGATION_POSE_POSITION);
        ui->labelNavigationPositionValue->setText(QString("%1, %2, %3")
                .arg(navigationPosition.x())
                .arg(navigationPosition.y())
                .arg(navigationPosition.z()));

        auto navigationOrientation = geometryToQt(thisModel()->data<geometry_msgs::Quaternion>(telemetry::Index::NAVIGATION_POSE_ORIENTATION));
        ui->labelNavigationOrientationQuaternionValue->setText(getQuaternionAsString(navigationOrientation));
        ui->labelNavigationOrientationDegreeValue->setText(getDegreeAsString(navigationOrientation));
        ui->labelNavigationStatusTypeValue->setText(getNavigationStatusAsString(thisModel()->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS).status));
        ui->labelNavigationStatusMarkerValue->setText(
                    getBoolAsString(thisModel()->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS).marker));

        auto navigationA = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::NAVIGATION_A);
        ui->labelNavigationAValue->setText(QString("%1, %2, %3").arg(navigationA.x).arg(navigationA.y).arg(navigationA.z));

        auto navigationLinear = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::NAVIGATION_TWIST_LINEAR);
        ui->labelNavigationLinearValue->setText(QString("%1, %2, %3").arg(navigationLinear.x).arg(navigationLinear.y).arg(navigationLinear.z));

        auto navigationAngular = thisModel()->data<geometry_msgs::Vector3>(telemetry::Index::NAVIGATION_TWIST_ANGULAR);
        ui->labelNavigationAngularValue->setText(QString("%1, %2, %3").arg(navigationAngular.x).arg(navigationAngular.y).arg(navigationAngular.z));
    }

    // StartUp action feedback.
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_STARTUP_FEEDBACK_DURATION))
    {
        ui->labelNavigationStartUpFeedbackDurationValue->setText(
                    QString("%1 sec.").arg(thisModel()->data<ros::Duration>(telemetry::Index::NAVIGATION_STARTUP_FEEDBACK_DURATION).toSec()));
    }

    // StartUp action result.
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_STARTUP_RESULT_TIMESTAMP))
    {
        ui->labelNavigationStartUpResultTimestampValue->setText(
                    dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::NAVIGATION_STARTUP_RESULT_TIMESTAMP)));
        ui->labelNavigationStartUpResultStatusValue->setText(
                    getNavigationStartUpResultAsString(thisModel()->data<unsigned char>(telemetry::Index::NAVIGATION_STARTUP_RESULT_TYPE)));
    }

    // Debug information.
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_DEBUG_POINT))
    {
        ui->labelNavigationDebugPointValue->setText(QString("%1").arg(thisModel()->data<unsigned int>(telemetry::Index::NAVIGATION_DEBUG_POINT)));
        auto posOffsets = thisModel()->data<QList<float>>(telemetry::Index::NAVIGATION_DEBUG_POS_OFFSETS);
        QString posOffsetsLabel = "";
        for(auto i = posOffsets.begin(); i != posOffsets.end(); ++i)
        {
            posOffsetsLabel += QString("%1 ").arg(*i);
        }
        ui->labelNavigationDebugPositionOffsetsValue->setText(posOffsetsLabel);
        auto attOffsets = thisModel()->data<QList<float>>(telemetry::Index::NAVIGATION_DEBUG_ATT_OFFSETS);
        QString attOffsetsLabel = "";
        for(auto i = attOffsets.begin(); i != attOffsets.end(); ++i)
        {
            attOffsetsLabel += QString("%1 ").arg(*i);
        }
        ui->labelNavigationDebugAttitudeOffsetsValue->setText(attOffsetsLabel);
        ui->labelNavigationDebugIMUTemperatureValue->setNum(thisModel()->data<float>(telemetry::Index::NAVIGATION_DEBUG_IMU_TEMPERATURE));
    }

    // Status.
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS))
    {
        ui->labelNavigationStatusTopicStatusValue->setText(
                    getNavigationStatusAsString(thisModel()->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS).status));
        ui->labelNavigationStatusTopicMarkerValue->setText(
                    getBoolAsString(thisModel()->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS).marker));
    }

    // MarkerCorrection
    if(thisModel()->getInsertStatus(telemetry::Index::MARKER_CORRECTION_TIMESTAMP))
    {
        ui->labelMarkerCorrectionTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::MARKER_CORRECTION_TIMESTAMP)));
        ui->labelMarkerCorrectionResultValue->setText(getMarkerCorrectionResultString(thisModel()->data(telemetry::Index::MARKER_CORRECTION_STATUS)));
    }

    // Update parameter.
    if(thisModel()->getInsertStatus(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelNavigationUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelNavigationUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    if(thisModel()->getInsertStatus(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelImuUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelImuUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    if(thisModel()->getInsertStatus(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelUpdateParameterSlamWrapperTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelUpdateParameterSlamWrapperResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT)));

    }


    // Parameters.
    for(auto i = publishParameterMap.keyBegin(); i != publishParameterMap.keyEnd(); ++i)
    {
        if(!(*i).startsWith(rosparam::PREFIX_SENSOR_FUSION))
        {
            continue;
        }

        auto itemList = ui->tableWidgetNavigationParameters->findItems(*i, Qt::MatchFlag::MatchFixedString);
        QTableWidgetItem* targetItemKey;
        QTableWidgetItem* targetItemValue;
        // 表の1列目はkey, 2列目はvalue.
        if(itemList.size() > 0)
        {
            targetItemKey = ui->tableWidgetNavigationParameters->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
            targetItemValue = ui->tableWidgetNavigationParameters->item(targetItemKey->row(), 1);
        }
        else
        {
            ui->tableWidgetNavigationParameters->insertRow(0);
            ui->tableWidgetNavigationParameters->setItem(0, 0, new QTableWidgetItem());
            ui->tableWidgetNavigationParameters->setItem(0, 1, new QTableWidgetItem());
            targetItemKey = ui->tableWidgetNavigationParameters->item(0, 0);
            targetItemValue = ui->tableWidgetNavigationParameters->item(0, 1);
            targetItemKey->setText(*i);
        }
        targetItemValue->setText(publishParameterMap.value(*i));
    }

    /*
     * IMU
     */
    if(thisModel()->getInsertStatus(telemetry::Index::IMU_TIMESTAMP))
    {
        ui->labelImuTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::IMU_TIMESTAMP)));
        ui->labelImuAccValue->setText(QString("%1, %2, %3")
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_ACC_X)))
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_ACC_Y)))
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_ACC_Z))));
        ui->labelImuGyroValue->setText(QString("%1, %2, %3")
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_GYRO_X)))
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_GYRO_Y)))
                                         .arg(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_GYRO_Z))));
        ui->labelImuTemperatureValue->setNum(static_cast<double>(thisModel()->data<float>(telemetry::Index::IMU_TEMPERATURE)));
    }

    /*
     * Camera and microphone.
     */
    if(thisModel()->getInsertStatus(telemetry::Index::CAMERA_MIC_STREAMING_STATUS))
    {
        ui->labelCameraStreamingValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_STREAMING_STATUS)));
        ui->labelCameraRecordingValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_RECORDING_STATUS)));
        ui->labelMainCameraOnOffValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_CAMERA_POWER)));
        ui->labelMicrophoneOnOffValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_MICROPHONE_POWER)));

        // Main Camera settings.
        ui->labelZoomValue->setNum(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_ZOOM));
        ui->labelEVValue->setNum(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_EV));
        ui->labelCameraGainValue->setNum(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_CAMERA_GAIN));
        ui->labelWhiteBalanceValue->setText(getWhiteBalanceModeAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_WHITE_BALANCE_MODE)));
        ui->labelResolutionValue->setText(
                    cameraConfig_->getQualitySettingAsStringById(
                        thisModel()->data<ib2_msgs::MainCameraResolutionType>(telemetry::Index::CAMERA_MIC_RESOLUTION_TYPE).type));
        ui->labelBitRateValue->setNum(static_cast<int>(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_SENDING_BIT_RATE)));
        ui->labelFrameRateValue->setNum(static_cast<int>(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_FRAME_RATE)));

        // Microphone settings.
        ui->labelMicGainValue->setNum(thisModel()->data<float>(telemetry::Index::CAMERA_MIC_MICROPHONE_GAIN));
    }

    // Update parameter.
    if(thisModel()->getInsertStatus(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelCameraUpdateParameterTimestampValue->setText(
                    dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelCameraUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    /*
     * LED, DisplayManager.
     */

    // Display Manager.
    if(thisModel()->getInsertStatus(telemetry::Index::DISPLAY_MANAGER_STATUS_MODE))
    {
        ui->labelDisplayManagerStatusModeValue->setText(
                    getIntBallModeAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_STATUS_MODE)));
        ui->labelDisplayManagerStatusCtlStatusTypeValue->setText(
                    getCtlStatusAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_STATUS_CTL_STATUS_TYPE)));
        ui->labelDisplayManagerStatusColorValue->setText(getColorRGBAAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_STATUS_COLOR)));
        ui->labelDisplayManagerStatusPowerValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_STATUS_POWER)));
        ui->labelDisplayManagerStatusLightingValue->setText(getPowerStatusAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH)));
    }

    if(thisModel()->getInsertStatus(telemetry::Index::DISPLAY_MANAGER_SWITCH_POWER_RESPONSE))
    {
        ui->labelDisplayManagerSwitchPowerResponseValue->setText(
                    getPowerStatusAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_SWITCH_POWER_RESPONSE)));
    }

    if(thisModel()->getInsertStatus(telemetry::Index::DISPLAY_MANAGER_SWITCH_FLASH_RESPONSE))
    {
        ui->labelDisplayManagerSwitchFlashResponseValue->setText(
                    getPowerStatusAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_SWITCH_FLASH_RESPONSE)));
    }

    if(thisModel()->getInsertStatus(telemetry::Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelDisplayManagerUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelDisplayManagerUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_RESULT)));
    }

    // LED left.
    if(thisModel()->getInsertStatus(telemetry::Index::LED_LEFT_LED_COLORS))
    {
        auto colors = thisModel()->data<QVector<std_msgs::ColorRGBA>>(telemetry::Index::LED_LEFT_LED_COLORS);
        ui->labelLedLeftColorsValue->setText(getColorRGBAAsString(QVariant::fromValue(colors[0])));
        ui->labelLedLeftColorsValue_2->setText(getColorRGBAAsString(QVariant::fromValue(colors[1])));
        ui->labelLedLeftColorsValue_3->setText(getColorRGBAAsString(QVariant::fromValue(colors[2])));
        ui->labelLedLeftColorsValue_4->setText(getColorRGBAAsString(QVariant::fromValue(colors[3])));
        ui->labelLedLeftColorsValue_5->setText(getColorRGBAAsString(QVariant::fromValue(colors[4])));
        ui->labelLedLeftColorsValue_6->setText(getColorRGBAAsString(QVariant::fromValue(colors[5])));
        ui->labelLedLeftColorsValue_7->setText(getColorRGBAAsString(QVariant::fromValue(colors[6])));
        ui->labelLedLeftColorsValue_8->setText(getColorRGBAAsString(QVariant::fromValue(colors[7])));
    }
    if(thisModel()->getInsertStatus(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelLedLeftUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelLedLeftUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT)));

    }
    // LED left Parameters.
    auto maxValueLengthLedLeft = 0;
    for(auto i = publishParameterMap.keyBegin(); i != publishParameterMap.keyEnd(); ++i)
    {
        if(!(*i).startsWith(rosparam::PREFIX_LED_LEFT))
        {
            continue;
        }

        auto itemList = ui->tableWidgetLedLeftParameters->findItems(*i, Qt::MatchFlag::MatchFixedString);
        QTableWidgetItem* targetItemKey;
        QTableWidgetItem* targetItemValue;
        // 表の1列目はkey, 2列目はvalue.
        if(itemList.size() > 0)
        {
            targetItemKey = ui->tableWidgetLedLeftParameters->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
            targetItemValue = ui->tableWidgetLedLeftParameters->item(targetItemKey->row(), 1);
        }
        else
        {
            ui->tableWidgetLedLeftParameters->insertRow(0);
            ui->tableWidgetLedLeftParameters->setItem(0, 0, new QTableWidgetItem());
            ui->tableWidgetLedLeftParameters->setItem(0, 1, new QTableWidgetItem());
            targetItemKey = ui->tableWidgetLedLeftParameters->item(0, 0);
            targetItemValue = ui->tableWidgetLedLeftParameters->item(0, 1);
            targetItemKey->setText(*i);
        }
        targetItemValue->setText(publishParameterMap.value(*i));
        auto valueLength = fontMetrics().horizontalAdvance(publishParameterMap.value(*i));
        if(valueLength > maxValueLengthLedLeft)
        {
            maxValueLengthLedLeft = valueLength;
        }
    }
    // テーブルにパラメータを追加した後にViewのリサイズ処理を呼び出す必要がある.
    ui->tableWidgetLedLeftParameters->resizeColumnsToContents();
    // ヘッダー未表示だと列幅再設定が動作しないケースがあるため、文字列長から列幅を算出する.
    ui->tableWidgetLedLeftParameters->horizontalHeader()->resizeSection(1, maxValueLengthLedLeft + 10);

    // LED right.
    if(thisModel()->getInsertStatus(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP))
    {
        ui->labelLedRightUpdateParameterTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)));
        ui->labelLedRightUpdateParameterResultValue->setText(
                    getUpdateParameterResponseResultAsString(thisModel()->data(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT)));

    }
    if(thisModel()->getInsertStatus(telemetry::Index::LED_RIGHT_LED_COLORS))
    {
        auto colors = thisModel()->data<QVector<std_msgs::ColorRGBA>>(telemetry::Index::LED_RIGHT_LED_COLORS);
        ui->labelLedRightColorsValue->setText(getColorRGBAAsString(QVariant::fromValue(colors[0])));
        ui->labelLedRightColorsValue_2->setText(getColorRGBAAsString(QVariant::fromValue(colors[1])));
        ui->labelLedRightColorsValue_3->setText(getColorRGBAAsString(QVariant::fromValue(colors[2])));
        ui->labelLedRightColorsValue_4->setText(getColorRGBAAsString(QVariant::fromValue(colors[3])));
        ui->labelLedRightColorsValue_5->setText(getColorRGBAAsString(QVariant::fromValue(colors[4])));
        ui->labelLedRightColorsValue_6->setText(getColorRGBAAsString(QVariant::fromValue(colors[5])));
        ui->labelLedRightColorsValue_7->setText(getColorRGBAAsString(QVariant::fromValue(colors[6])));
        ui->labelLedRightColorsValue_8->setText(getColorRGBAAsString(QVariant::fromValue(colors[7])));
    }
    // LED right parameters.
    auto maxValueLengthLedRight = 0;
    for(auto i = publishParameterMap.keyBegin(); i != publishParameterMap.keyEnd(); ++i)
    {
        if(!(*i).startsWith(rosparam::PREFIX_LED_RIGHT))
        {
            continue;
        }

        auto itemList = ui->tableWidgetLedRightParameters->findItems(*i, Qt::MatchFlag::MatchFixedString);
        QTableWidgetItem* targetItemKey;
        QTableWidgetItem* targetItemValue;
        // 表の1列目はkey, 2列目はvalue.
        if(itemList.size() > 0)
        {
            targetItemKey = ui->tableWidgetLedRightParameters->findItems(*i, Qt::MatchFlag::MatchFixedString).at(0);
            targetItemValue = ui->tableWidgetLedRightParameters->item(targetItemKey->row(), 1);
        }
        else
        {
            ui->tableWidgetLedRightParameters->insertRow(0);
            ui->tableWidgetLedRightParameters->setItem(0, 0, new QTableWidgetItem());
            ui->tableWidgetLedRightParameters->setItem(0, 1, new QTableWidgetItem());
            targetItemKey = ui->tableWidgetLedRightParameters->item(0, 0);
            targetItemValue = ui->tableWidgetLedRightParameters->item(0, 1);
            targetItemKey->setText(*i);
        }
        targetItemValue->setText(publishParameterMap.value(*i));
        auto valueLength = fontMetrics().horizontalAdvance(publishParameterMap.value(*i));
        if(valueLength > maxValueLengthLedRight)
        {
            maxValueLengthLedRight = valueLength;
        }
    }
    // テーブルにパラメータを追加した後にViewのリサイズ処理を呼び出す必要がある.
    ui->tableWidgetLedRightParameters->resizeColumnsToContents();
    // ヘッダー未表示だと列幅再設定が動作しないケースがあるため、文字列長から列幅を算出する.
    ui->tableWidgetLedRightParameters->horizontalHeader()->resizeSection(1, maxValueLengthLedRight + 10);

    /*
     * FileMonitor
     */
    if(thisModel()->getInsertStatus(telemetry::Index::FILE_MONITOR_CHECK_TIME))
    {
        ui->labelFileMonitorCheckTimeValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::FILE_MONITOR_CHECK_TIME)));
        QString timeSyncLog = QString::fromStdString(thisModel()->data<std::string>(telemetry::Index::FILE_MONITOR_TIMESYNC_LOG));
        auto timeSyncLogUnixTime = static_cast<unsigned int>(timeSyncLog.toDouble());
        QDateTime timeSyncLogDateTime;
        timeSyncLogDateTime.setTime_t(timeSyncLogUnixTime);
        ui->labelFileMonitorTimeSynchronizationValue->setText(dateTimeString(timeSyncLogDateTime));
    }

    /*
     * SetRosparam / SetRosParams
     */
    if(thisModel()->getInsertStatus(telemetry::Index::SET_ROS_PARAM_SUCCESS))
    {
        auto setParamResult = thisModel()->data<bool>(telemetry::Index::SET_ROS_PARAM_SUCCESS) ? "SUCCESS" : "FAIL";
        ui->labelSetRosparamResultValue->setText(setParamResult);
    }

    if(thisModel()->getInsertStatus(telemetry::Index::SET_ROS_PARAM_SUCCESS_TIMESTAMP))
        ui->labelSetRosparamTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::SET_ROS_PARAM_SUCCESS_TIMESTAMP)));

    if(thisModel()->getInsertStatus(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL))
    {
        auto setParamResult = thisModel()->data<bool>(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL) ? "SUCCESS" : "FAIL";
        ui->labelSetRosparamsSuccessAllResultValue->setText(setParamResult);
    }

    if(thisModel()->getInsertStatus(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL_TIMESTAMP))
        ui->labelSetRosparamsSuccessAllTimestampValue->setText(dateTimeString(thisModel()->data<ros::Time>(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL_TIMESTAMP)));

    auto setRosParamsList = thisModel()->data<QMap<QString, RosParam>>(telemetry::Index::SET_ROS_PARAMS_PARAMS_LIST);
    for(auto i = setRosParamsList.keyBegin(); i != setRosParamsList.keyEnd(); ++i)
    {
        updateRosParamTableWidget(*i, setRosParamsList, ui->tableWidgetSetRosparamsValue);
    }

    /*
     * GetRosparam / GetRosparams
     */

    auto getRosParamsList = thisModel()->data<QMap<QString, RosParam>>(telemetry::Index::GET_ROS_PARAMS_PARAMS_LIST);
    if(thisModel()->getInsertStatus(telemetry::Index::GET_ROS_PARAM_VALUES))
    {
        auto getParam = thisModel()->data<RosParam>(telemetry::Index::GET_ROS_PARAM_VALUES);
        if(!getRosParamsList.contains(QString::fromStdString(getParam.id)) ||
                getParam.stamp.toSec() > getRosParamsList.value(QString::fromStdString(getParam.id)).stamp.toSec())
        {
            // GetParam結果とGetParams結果のうち、新しい方を表示データとして採用する.
            getRosParamsList.insert(QString::fromStdString(getParam.id), getParam);
        }
    }

    for(auto i = getRosParamsList.keyBegin(); i != getRosParamsList.keyEnd(); ++i)
    {
        updateRosParamTableWidget(*i, getRosParamsList, ui->tableWidgetGetRosparamsValue);
    }
}

QModelIndex IntBallTelemetryWidget::moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers)
{
    Q_UNUSED(cursorAction);
    Q_UNUSED(modifiers);
    return QModelIndex();
}

int IntBallTelemetryWidget::horizontalOffset() const
{
    return 0;
}

int IntBallTelemetryWidget::verticalOffset() const
{
    return 0;
}

bool IntBallTelemetryWidget::isIndexHidden(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return false;
}

void IntBallTelemetryWidget::setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags command)
{
    Q_UNUSED(rect);
    Q_UNUSED(command);
}

QRegion IntBallTelemetryWidget::visualRegionForSelection(const QItemSelection &selection) const
{
    Q_UNUSED(selection);
    return QRegion();
}

void IntBallTelemetryWidget::rowsAboutToBeRemoved(const QModelIndex &parent, int start, int end)
{
    Q_UNUSED(parent);
    Q_UNUSED(start);
    Q_UNUSED(end);
}

void IntBallTelemetryWidget::rowsInserted(const QModelIndex &parent, int start, int end)
{
    Q_UNUSED(parent);
    Q_UNUSED(start);
    Q_UNUSED(end);
}

void IntBallTelemetryWidget::selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    Q_UNUSED(selected);
    Q_UNUSED(deselected);
}

void IntBallTelemetryWidget::currentChanged(const QModelIndex &current, const QModelIndex &previous)
{
    Q_UNUSED(current);
    Q_UNUSED(previous);
}

void IntBallTelemetryWidget::setVisibility(const TelemetryWidgetGroup &group, const bool on)
{
    switch(group)
    {
    case TelemetryWidgetGroup::HEADER:
        ui->groupHeader->setVisible(on);
        break;
    case TelemetryWidgetGroup::FLIGHT_SOFTWARE:
        ui->groupFlightSoftware->setVisible(on);
        break;
    case TelemetryWidgetGroup::ALIVE_STATUS:
        ui->groupAliveStatus->setVisible(on);
        break;
    case TelemetryWidgetGroup::SYSTEM_MONITOR:
        ui->groupSystemMonitor->setVisible(on);
        break;
    case TelemetryWidgetGroup::TASK_MANAGER:
        ui->groupTaskManager->setVisible(on);
        break;
    case TelemetryWidgetGroup::NAVIGATION:
        ui->groupNavigation->setVisible(on);
        break;
    case TelemetryWidgetGroup::IMU:
        ui->groupImu->setVisible(on);
        break;
    case TelemetryWidgetGroup::GUIDANCE_AND_CONTROL:
        ui->groupGuidanceAndControl->setVisible(on);
        break;
    case TelemetryWidgetGroup::FILE_MONITOR:
        ui->groupFileMonitor->setVisible(on);
        break;
    case TelemetryWidgetGroup::PROPULSION:
        ui->groupProp->setVisible(on);
        break;
    case TelemetryWidgetGroup::WRENCH:
        ui->groupWrench->setVisible(on);
        break;
    case TelemetryWidgetGroup::CAMERA_AND_MICROPHONE:
        ui->groupCameraMicrophone->setVisible(on);
        break;
    case TelemetryWidgetGroup::LED_DISPLAY:
        ui->groupLedDisplay->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_CAMERA_AND_MICROPHONE:
        ui->groupCameraMicrophoneResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_PROPULSION:
        ui->groupPropResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_GUIDANCE_AND_CONTROL:
        ui->groupGuidanceAndControlResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_TASK_MANAGER:
        ui->groupTaskManagerResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_NAVIGATION:
        ui->groupNavigationResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_IMU:
        ui->groupImuResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_LED_DISPLAY:
        ui->groupLedDisplayResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_DISPLAY_MANAGER:
        ui->groupDisplayManagerResponse->setVisible(on);
        break;
    case TelemetryWidgetGroup::RESPONSE_ROSPARAMS:
        ui->groupRosparamResponse->setVisible(on);
        break;
    }

}
