#include <QDateTime>
#include <QMediaPlayer>
#include <QMediaPlaylist>
#include <QPropertyAnimation>
#include "status_widget.h"
#include "ui_status_widget.h"
#include "common_log_object.h"
#include "gui_color.h"
#include "ib2_msgs.h"
#include "operator_gui_config.h"
#include "qdebug_custom.h"
#include "utils.h"

using namespace intball;
using namespace qsettings;
using namespace qsettings::key;

const QColor StatusWidget::LED_COLOR_CAMERA_AND_MICROPHONE = QColor(0, 0, 255);
const QColor StatusWidget::LED_COLOR_CAMERA = QColor(0, 255, 0);
const QColor StatusWidget::LED_COLOR_MICROPHONE = QColor(255, 13, 102);
const QColor StatusWidget::LED_COLOR_WITHOUT_ALL = QColor(255, 51, 0);
const QColor StatusWidget::LED_COLOR_NONE = QColor(10, 10, 10);
const QColor StatusWidget::DEFAULT_FRAME_COLOR = Color::U2;

StatusWidget::StatusWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StatusWidget),
    maxLogNumber_(Config::valueAsInt(KEY_STATUS_LOG_NUMBER_MAX)),
    isTooltipEnabled_(false),
    soundPlayerAlertRingingTime_(0),
    soundPlayerWarningRingingTime_(0),
    borderColor_(DEFAULT_FRAME_COLOR)
{
    ui->setupUi(this);

    // 点滅表示の設定.
    blinkAnimation_ = new QPropertyAnimation(this, "borderColor");
    blinkAnimation_->setLoopCount(-1);

    // 点滅停止ボタン.
    // 初期は非表示.
    ui->stopBlinkingButton->setToolTip("Stop blinking");
    ui->stopBlinkingButton->hide();

    ui->tableIntBallEvents->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
    ui->tableIntBallEvents->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
    ui->tableIntBallEvents->horizontalHeader()->setSectionResizeMode(2, QHeaderView::Fixed);
    ui->tableIntBallEvents->horizontalHeader()->resizeSection(2, COLUMN_WIDTH_LOG_MESSAGE);
    ui->tableIntBallEvents->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    setLabelColor(ui->labelIntBallLastUpdateTitle, Color::S1);
    setLabelColor(ui->labelIntBallLastUpdateValue, Color::S1);
    setLabelColor(ui->labelDockLastUpdateTitle, Color::S1);
    setLabelColor(ui->labelDockLastUpdateValue, Color::S1);

    setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S1);
    setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S1, BATTERY_MARGIN_RIGHT);
    setProgressBarStyleSheet(ui->progressIntBallStorage1, Color::S1, STORAGE_MARGIN_RIGHT);
    setProgressBarStyleSheet(ui->progressIntBallStorage2, Color::S1, STORAGE_MARGIN_RIGHT);

    ui->progressIntballTemperature->setFormat(QString("%v") + QChar(0x2103));

    soundPlayerAlert_ = new QMediaPlayer;
    soundPlayerAlert_->setVolume(Config::valueAsInt(KEY_ALERT_EVENT_VOLUME));
    soundPlayerAlertRingingTime_ = Config::valueAsInt(KEY_ALERT_EVENT_RINGING_TIME);
    if(soundPlayerAlertRingingTime_ > 0)
    {
        // 鳴動時間が定義されている場合は, ループ再生.

        QMediaPlaylist* mediaList = new QMediaPlaylist(soundPlayerAlert_);
        mediaList->addMedia(QUrl::fromLocalFile(Config::valueAsString(KEY_ALERT_EVENT_FILE)));
        mediaList->setPlaybackMode(QMediaPlaylist::Loop);
        soundPlayerAlert_->setPlaylist(mediaList);

        // 停止用タイマーの設定
        soundPlayerAlertStopTimer_ = new QTimer(this);
        soundPlayerAlertStopTimer_->setSingleShot(true);
        soundPlayerAlertStopTimer_->setInterval(soundPlayerAlertRingingTime_ * 1000);
        connect(soundPlayerAlertStopTimer_, &QTimer::timeout, this, &StatusWidget::stopPlayingAlertSound);
    }
    else
    {
        // 鳴動時間が0の場合, ループ再生はしない.
        soundPlayerAlert_->setMedia(QUrl::fromLocalFile(Config::valueAsString(KEY_ALERT_EVENT_FILE)));
    }

    soundPlayerWarning_ = new QMediaPlayer;
    soundPlayerWarning_->setVolume(Config::valueAsInt(KEY_WARNING_EVENT_VOLUME));
    soundPlayerWarningRingingTime_ = Config::valueAsInt(KEY_WARNING_EVENT_RINGING_TIME);
    if(soundPlayerWarningRingingTime_ > 0)
    {
        // 鳴動時間が定義されている場合は, ループ再生.

        QMediaPlaylist* mediaList = new QMediaPlaylist(soundPlayerWarning_);
        mediaList->addMedia(QUrl::fromLocalFile(Config::valueAsString(KEY_ALERT_EVENT_FILE)));
        mediaList->setPlaybackMode(QMediaPlaylist::Loop);
        soundPlayerWarning_->setPlaylist(mediaList);

        // 停止用タイマーの設定
        soundPlayerWarningStopTimer_ = new QTimer(this);
        soundPlayerWarningStopTimer_->setSingleShot(true);
        soundPlayerWarningStopTimer_->setInterval(soundPlayerWarningRingingTime_ * 1000);
        connect(soundPlayerWarningStopTimer_, &QTimer::timeout, this, &StatusWidget::stopPlayingWarningSound);
    }
    else
    {
        // 鳴動時間が0の場合, ループ再生はしない.
        soundPlayerWarning_->setMedia(QUrl::fromLocalFile(Config::valueAsString(KEY_WARNING_EVENT_FILE)));
    }
}

StatusWidget::~StatusWidget()
{
    delete ui;
}

void StatusWidget::setLabelColor(QWidget* label, const QColor& color)
{
    label->setStyleSheet(QString("color: %1;").arg(Color::styleSheetRGB(color)));
}

void StatusWidget::setProgressBarStyleSheet(QWidget* progressBar, const QColor& color, const int marginRight)
{
    progressBar->setStyleSheet(QString("QProgressBar {"
                                       "background-color: %1;"
                                       "height: 20px;"
                                       "border: 0;"
                                       "margin-right: %2px;"
                                       "text-align: right;"
                                       "}"
                                       "QProgressBar::chunk {"
                                       "background-color: %3;"
                                       "margin: 0;"
                                       "}"
                                       "* {"
                                       "color: %3"
                                       "}")
                               .arg(Color::styleSheetRGB(Color::U3))
                               .arg(marginRight)
                               .arg(Color::styleSheetRGB(color)));
}

void StatusWidget::initialize(IntBallTelemetry* intballTelemetry, DockTelemetry* dockTelemetry, intball::TelemetryMonitor* telemetryMonitor)
{
    intballTelemetry_ = intballTelemetry;
    connect(intballTelemetry_, &QAbstractListModel::dataChanged, this, &StatusWidget::IntBall2Telemetry_dataChanged);

    dockTelemetry_ = dockTelemetry;
    connect(dockTelemetry_, &QAbstractListModel::dataChanged, this, &StatusWidget::DockTelemetry_dataChanged);

    telemetryMonitor_ = telemetryMonitor;
    connect(telemetryMonitor_, &TelemetryMonitor::detected, this, &StatusWidget::TelemetryMonitor_detected);
}

void StatusWidget::setOrientation(Qt::Orientation o)
{
    auto targetLayout = dynamic_cast<QGridLayout*>(ui->statusIntBallValues->layout());
    auto targetDockLayout = dynamic_cast<QGridLayout*>(ui->statusDockValues->layout());
    Q_ASSERT(targetLayout != nullptr);
    Q_ASSERT(targetDockLayout != nullptr);

    if(o == Qt::Vertical)
    {
        targetLayout->removeWidget(ui->statusIntBallValues1);
        targetLayout->addWidget(ui->statusIntBallValues1, 1, 0);
        targetDockLayout->removeWidget(ui->widgetDockBattery);
        targetDockLayout->addWidget(ui->widgetDockBattery, 1, 0);
        // 縦表示の場合はログ文言はツールチップ表示.
        isTooltipEnabled_ = true;

        // 全てのログ文言でツールチップ表示を有効化する.
        for(int i = 0; i < ui->tableIntBallEvents->rowCount(); ++i)
        {
            QColor color;
            auto level = ui->tableIntBallEvents->item(i, 1)->text();
            if(level == LABEL_LOGLEVEL_WARN)
            {
                color = Color::S2;
            }
            else if(level == LABEL_LOGLEVEL_ALERT)
            {
                color = Color::S3;
            }
            enableTooltip(*(ui->tableIntBallEvents->item(i, 2)), color);
        }
    }

    if(o == Qt::Horizontal)
    {
        targetLayout->removeWidget(ui->statusIntBallValues1);
        targetLayout->addWidget(ui->statusIntBallValues1, 0, 1);
        targetDockLayout->removeWidget(ui->widgetDockBattery);
        targetDockLayout->addWidget(ui->widgetDockBattery, 0, 1);
        isTooltipEnabled_ = false;

        // 全てのログ文言でツールチップ表示を無効化する.
        for(int i = 0; i < ui->tableIntBallEvents->rowCount(); ++i)
        {
            disableTooltip(*(ui->tableIntBallEvents->item(i, 2)));
        }
    }
}


void StatusWidget::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    // 受信日時.
    ui->labelIntBallLastUpdateValue->setText(intballTelemetry_->getReceivedTimestampString());

    // FlightSoftware 起動有無
    ui->labelFlightSoftwareValue->setText(intballTelemetry_->isFlightSoftwareStarted() ? "ON" : "OFF");

    // テレメトリデータ内タイムスタンプ.
    ui->labelIntBallTimestampValue->setText(intballTelemetry_->getTimestampString());

    // 動作モード.
    ui->labelIntBallModeValue->setText(intballTelemetry_->getModeAsString());

    // 誘導制御ステータス.
    ui->labelControlStateValue->setText(getCtlStatusAsString(intballTelemetry_->data(telemetry::Index::CTL_STATUS_TYPE)));

    // LEDの色.
    if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::DISPLAY_MANAGER_STATUS_POWER).status == ib2_msgs::PowerStatus::OFF)
    {
        // 表示管理機能がOFFの場合は色表記無し.
        ui->widgetIntBallLedValue->setStyleSheet("background-color: rgb(0, 0, 0);");
    }
    if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH).status == ib2_msgs::PowerStatus::ON)
    {
        // 撮影用フラッシュ転倒時は白色表示.
        ui->widgetIntBallLedValue->setStyleSheet("background-color: rgb(255, 255, 255);");
    }
    else
    {
        // フラッシュ未点灯時は表示管理の色を採用.
        auto color = intballTelemetry_->data<std_msgs::ColorRGBA>(telemetry::Index::DISPLAY_MANAGER_STATUS_COLOR);
        ui->widgetIntBallLedValue->setStyleSheet(
                    QString("background-color: rgb(%1, %2, %3);")
                    .arg(static_cast<int>(color.r * 255))
                    .arg(static_cast<int>(color.g * 255))
                    .arg(static_cast<int>(color.b * 255)));
    }

    // マーカー捕捉有無.
    ui->labelIntBallMarkerValue->setText(intballTelemetry_->getMarkerStatus() ? "Detecting" : "Not detected");

    // 温度.
    ui->progressIntballTemperature->setValue(static_cast<int>(intballTelemetry_->data<float>(telemetry::Index::SYSTEM_MONITOR_TEMPERATURE)));

    // ストレージ1
    auto storageValues = intballTelemetry_->data<QMap<QString, float>>(telemetry::Index::SYSTEM_MONITOR_DISK_SPACES);
    auto keys = storageValues.keys();
    if(keys.count() > 0)
    {
        ui->labelStorage1Path->setText(keys.at(0));
        ui->progressIntBallStorage1->setValue(static_cast<int>(storageValues.value(keys.at(0))));
    }

    // ストレージ2.
    if(keys.count() > 1)
    {
        ui->labelStorage2Path->setVisible(true);
        ui->progressIntBallStorage2->setVisible(true);
        ui->labelStorage2Path->setText(keys.at(1));
        ui->progressIntBallStorage2->setValue(static_cast<int>(storageValues.value(keys.at(1))));
    }
    else
    {
        ui->labelStorage2Path->setVisible(false);
        ui->progressIntBallStorage2->setVisible(false);
    }

    // バッテリー充電状態.
    auto batteryRemain = intballTelemetry_->data<char>(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN);
    ui->labelIntBallBatteryRemainValue->setText(QString("%1%").arg(static_cast<int>(batteryRemain)));
    char batteryRemainProgressValue;
    if(batteryRemain > 100)
    {
        batteryRemainProgressValue = 100;
    }
    else if(batteryRemain < 0)
    {
        batteryRemainProgressValue = 0;
    }
    else
    {
        batteryRemainProgressValue = batteryRemain;
    }
    ui->progressIntBallBattery->setValue(batteryRemainProgressValue);
}

void StatusWidget::TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value)
{   
    switch(event)
    {
    case TelemetryMonitor::Event::MODE_CHANGED:
        eventOccurred(CommandLog(
                          QString("The operation mode of Int-Ball2 has been changed: Current=%1")
                          .arg(getModeString(intballTelemetry_->data(telemetry::Index::MODE)))));
        break;
    case TelemetryMonitor::Event::START_CTL_COMMAND:
        eventOccurred(CommandLog("The guidance control process has started."));
        break;
    case TelemetryMonitor::Event::FINISH_CTL_COMMAND:
        eventOccurred(CommandLog(
                          QString("The guidance control process has been completed. Result=%1")
                          .arg(getCtlCommandResultAsString(intballTelemetry_->data(telemetry::Index::CTL_ACTION_RESULT_TYPE)))));
        break;
    case TelemetryMonitor::Event::MOVE_NOT_YET_COMPLETE:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "The guidance control process is still not completed."));
        break;
    case TelemetryMonitor::Event::INTBALL_TELEMETRY_RECEIVED:
        setLabelColor(ui->labelIntBallLastUpdateTitle, Color::S1);
        setLabelColor(ui->labelIntBallLastUpdateValue, Color::S1);
        eventOccurred(CommandLog("Started receiving Int-Ball2's telemetry."));
        break;
    case TelemetryMonitor::Event::TIMEOUT_TELEMETRY_INTBALL:
        setLabelColor(ui->labelIntBallLastUpdateTitle, Color::S2);
        setLabelColor(ui->labelIntBallLastUpdateValue, Color::S2);
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Int-Ball2's telemetry is not received."));
        break;
    case TelemetryMonitor::Event::DOCK_TELEMETRY_RECEIVED:
        setLabelColor(ui->labelDockLastUpdateTitle, Color::S1);
        setLabelColor(ui->labelDockLastUpdateValue, Color::S1);
        eventOccurred(CommandLog("Started receiving docking station's telemetry."));
        break;
    case TelemetryMonitor::Event::TIMEOUT_TELEMETRY_DOCK:
        setLabelColor(ui->labelDockLastUpdateTitle, Color::S2);
        setLabelColor(ui->labelDockLastUpdateValue, Color::S2);
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Docking-station's telemetry is not received."));
        break;
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_WARNING:
        setProgressBarStyleSheet(ui->progressIntBallStorage1, Color::S2, STORAGE_MARGIN_RIGHT);
        setProgressBarStyleSheet(ui->progressIntBallStorage2, Color::S2, STORAGE_MARGIN_RIGHT);
        eventOccurred(CommandLog(CommandLogLevel::WARN, "There is not enough free storage space."));
        break;
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_CRITICAL:
        setProgressBarStyleSheet(ui->progressIntBallStorage1, Color::S3, STORAGE_MARGIN_RIGHT);
        setProgressBarStyleSheet(ui->progressIntBallStorage2, Color::S3, STORAGE_MARGIN_RIGHT);
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "There is almost no free storage space."));
        break;
    case TelemetryMonitor::Event::STORAGE_USAGE_COME_UNDER_WARNING:
        setProgressBarStyleSheet(ui->progressIntBallStorage1, Color::S1, STORAGE_MARGIN_RIGHT);
        setProgressBarStyleSheet(ui->progressIntBallStorage2, Color::S1, STORAGE_MARGIN_RIGHT);
        eventOccurred(CommandLog("The storage capacity has been restored."));
        break;
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_WARNING:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "The temperature has reached a warning value."));
        // 温度に関してはMeterウィジット内で自動判定して表示を切り替える.
        break;
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_CRITICAL:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "The temperature has reached a critical value."));
        // 温度に関してはMeterウィジット内で自動判定して表示を切り替える.
        break;
    case TelemetryMonitor::Event::TEMPERATURE_COME_UNDER_WARNING:
        eventOccurred(CommandLog("The temperature is normal."));
        // 温度に関してはMeterウィジット内で自動判定して表示を切り替える.
        break;
    case TelemetryMonitor::Event::BATTERY_LOW:
        setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S2);
        setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S2, BATTERY_MARGIN_RIGHT);
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Battery is low."));
        break;
    case TelemetryMonitor::Event::BATTERY_LOW_CRITICAL:
        setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S3);
        setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S3, BATTERY_MARGIN_RIGHT);
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Battery is extremely low."));
        break;
    case TelemetryMonitor::Event::BATTERY_OUT_OF_RANGE:
        setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S3);
        setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S3, BATTERY_MARGIN_RIGHT);
        eventOccurred(CommandLog(CommandLogLevel::WARN, "The battery remaining value is out of range."));
        break;
    case TelemetryMonitor::Event::BATTERY_NORMAL:
        setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S1);
        setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S1, BATTERY_MARGIN_RIGHT);
        eventOccurred(CommandLog("The battery remaining value has returned to normal."));
        break;
    case TelemetryMonitor::Event::BATTERY_FULLY_CHARGED:
        setLabelColor(ui->labelIntBallBatteryRemainValue, Color::S1);
        setProgressBarStyleSheet(ui->progressIntBallBattery, Color::S1, BATTERY_MARGIN_RIGHT);
        eventOccurred(CommandLog("Int-Ball2 is fully charged."));
        break;
    case TelemetryMonitor::Event::MODE_IDLING:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Transitioned to IDLING mode."));
        break;
    case TelemetryMonitor::Event::MODE_OFF_NOMINAL:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Transitioned to OFF_NOMINAL mode."));
        break;
    case TelemetryMonitor::Event::RELEASE_CAPTURE:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Int-Ball2 was released after capture."));
        break;
    case TelemetryMonitor::Event::START_SCAN:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Scanning operation has started."));
        break;
    case TelemetryMonitor::Event::CAPTURED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Int-Ball2 has been captured."));
        break;
    case TelemetryMonitor::Event::STOPPED_BY_COLLISION:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Int-Ball2 detects a collision and stops moving."));
        break;
    case TelemetryMonitor::Event::CTL_STANDBY:
        eventOccurred(CommandLog(CommandLogLevel::INFO, "The guidance control has stopped."));
        break;
    case TelemetryMonitor::Event::CTL_ABNORMAL_SHUTDOWN:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "The guidance control has stopped during operation."));
        break;
    case TelemetryMonitor::Event::KEEP_POSE:
        eventOccurred(CommandLog(CommandLogLevel::INFO, "The guidance control is running KEEP_POSE(hold position)."));
        break;
    case TelemetryMonitor::Event::FAILED_CTL_COMMAND:
        if(value.isValid() && value.toUInt() == ib2_msgs::CtlCommandResult::TERMINATE_ABORTED)
        {
            eventOccurred(CommandLog(CommandLogLevel::WARN, "The guidance control process has aborted."));
        }
        else
        {
            eventOccurred(CommandLog(CommandLogLevel::ALERT, "The guidance control process has failed."));
        }
        break;
    case TelemetryMonitor::Event::CAMERA_ON:
        eventOccurred(CommandLog("Camera turned on."));
        break;
    case TelemetryMonitor::Event::CAMERA_OFF:
        eventOccurred(CommandLog("Camera turned off."));
        break;
    case TelemetryMonitor::Event::MICROPHONE_ON:
        eventOccurred(CommandLog("Microphone turned on."));
        break;
    case TelemetryMonitor::Event::MICROPHONE_OFF:
        eventOccurred(CommandLog("Microphone turned off."));
        break;
    case TelemetryMonitor::Event::STREAMING_ON:
        eventOccurred(CommandLog("Video streaming has started."));
        break;
    case TelemetryMonitor::Event::STREAMING_OFF:
        eventOccurred(CommandLog("Video streaming has stopped."));
        break;
    case TelemetryMonitor::Event::RECORDING_ON:
        eventOccurred(CommandLog("Video recording has started."));
        break;
    case TelemetryMonitor::Event::RECORDING_OFF:
        eventOccurred(CommandLog("Video recording has stopped."));
        break;
    case TelemetryMonitor::Event::CAMERA_FLASH_ON:
        eventOccurred(CommandLog("Camera lighting turned on."));
        break;
    case TelemetryMonitor::Event::CAMERA_FLASH_OFF:
        eventOccurred(CommandLog("Camera lighting turned off."));
        break;
    case TelemetryMonitor::Event::NAVIGATION_ACTION_TIMEOUT:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Start (or return) of control failed.Navigation function startup timed out."));
        break;
    case TelemetryMonitor::Event::NAVIGATION_ACTION_ABORTED:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Start (or return) of control failed.Navigation function activation has been aborted."));
        break;
    case TelemetryMonitor::Event::NAVIGATION_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Navigation"));
        break;
    case TelemetryMonitor::Event::NAVIGATION_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Navigation"));
        break;
    case TelemetryMonitor::Event::IMU_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: IMU"));
        break;
    case TelemetryMonitor::Event::IMU_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: IMU"));
        break;
    case TelemetryMonitor::Event::SLAM_WRAPPER_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Slam"));
        break;
    case TelemetryMonitor::Event::SLAM_WRAPPER_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Slam"));
        break;
    case TelemetryMonitor::Event::CTL_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Guidance control"));
        break;
    case TelemetryMonitor::Event::CTL_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Guidance control"));
        break;
    case TelemetryMonitor::Event::PROP_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Propulsion"));
        break;
    case TelemetryMonitor::Event::PROP_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Propulsion"));
        break;
    case TelemetryMonitor::Event::CAMERA_MIC_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Camera and microphone"));
        break;
    case TelemetryMonitor::Event::CAMERA_MIC_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Camera and microphone"));
        break;
    case TelemetryMonitor::Event::LED_LEFT_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Left LED"));
        break;
    case TelemetryMonitor::Event::LED_LEFT_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Left LED"));
        break;
    case TelemetryMonitor::Event::LED_RIGHT_UPDATE_PARAMETER_SUCCESS:
        eventOccurred(CommandLog("Parameter updated successfully: Right LED"));
        break;
    case TelemetryMonitor::Event::LED_RIGHT_UPDATE_PARAMETER_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Parameter update failed: Right LED"));
        break;
    case TelemetryMonitor::Event::MARKER_CORRECTION_SUCCESS:
        eventOccurred(CommandLog("Marker correction succeeded."));
        break;
    case TelemetryMonitor::Event::MARKER_CORRECTION_FAILED:
        eventOccurred(CommandLog(CommandLogLevel::ALERT, "Marker correction failed."));
        break;
    case TelemetryMonitor::Event::DOCK_CHARGING_ON:
        eventOccurred(CommandLog("Docking station has started charging."));
        break;
    case TelemetryMonitor::Event::DOCK_CHARGING_OFF:
        eventOccurred(CommandLog("Docking station has stopped charging."));
        break;
    case TelemetryMonitor::Event::FLIGHT_SOFTWARE_ON:
        eventOccurred(CommandLog("Flight software has started."));
        break;
    case TelemetryMonitor::Event::FLIGHT_SOFTWARE_OFF:
        eventOccurred(CommandLog(CommandLogLevel::WARN, "Flight software has stopped."));
        break;
    };
}

void StatusWidget::DockTelemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    // 受信日時.
    ui->labelDockLastUpdateValue->setText(dockTelemetry_->getReceivedTimestampString());

    // モーター動作状態.
    ui->labelDockModeValue->setText(dockTelemetry_->getMotorStateAsString());

    // 給電状態.
    ui->labelDockBatteryValue->setText(dockTelemetry_->getChargeStateAsString());

    // スイッチ状態.
    ui->labelDockSwitchValue->setText(dockTelemetry_->getSwitchStateAsString());

    // 最終充電時間.
    ui->labelDockChargeTimeValue->setText(dockTelemetry_->getChargeTimeAsString());
}

void StatusWidget::eventOccurred(CommandLog log)
{
    ui->tableIntBallEvents->insertRow(0);
    if(log.timestamp == QDateTime())
    {
        ui->tableIntBallEvents->setItem(0, 0, new QTableWidgetItem(dateTimeStringWithoutYear(QDateTime::currentDateTime())));
    }
    else
    {
        ui->tableIntBallEvents->setItem(0, 0, new QTableWidgetItem(dateTimeStringWithoutYear(log.timestamp)));
    }

    QString messageToDisplay = automaticLineBreak(log.message, COLUMN_WIDTH_LOG_MESSAGE - 20, ui->tableIntBallEvents->fontMetrics());
    ui->tableIntBallEvents->setItem(0, 2, new QTableWidgetItem(messageToDisplay));

    if(log.level == CommandLogLevel::ALERT)
    {
        ui->tableIntBallEvents->setItem(0, 1, new QTableWidgetItem(LABEL_LOGLEVEL_ALERT));
        LOG_WARNING() <<  "Event occurred: " << log.message;

        // 表示色変更.
        changeTextColor(0, Color::S3);

        // イベント音
        playAlertSound();
    }
    else if(log.level == CommandLogLevel::WARN)
    {
        ui->tableIntBallEvents->setItem(0, 1, new QTableWidgetItem(LABEL_LOGLEVEL_WARN));
        LOG_WARNING() <<  "Event occurred: " << log.message;

        // 表示色変更.
        changeTextColor(0, Color::S2);

        // イベント音
        playWarningSound();
    }
    else
    {
        ui->tableIntBallEvents->setItem(0, 1, new QTableWidgetItem(LABEL_LOGLEVEL_INFO));
        LOG_INFO() <<  "Event occurred: " << log.message;
    }

    if(isTooltipEnabled_)
    {
        // ツールチップ表示
        QColor color;
        if(log.level == CommandLogLevel::WARN)
        {
            color = Color::S2;
        }
        else if(log.level == CommandLogLevel::ALERT)
        {
            color = Color::S3;
        }
        enableTooltip(*(ui->tableIntBallEvents->item(0, 2)), color);
    }

    //最大数を超過した場合ログ行を削除.
    int removeRowCount = ui->tableIntBallEvents->rowCount() - maxLogNumber_;
    if(removeRowCount > 0)
    {
        for(int i = 0; i < removeRowCount; ++i)
        {
            ui->tableIntBallEvents->removeRow(ui->tableIntBallEvents->rowCount() - 1);
        }
    }

    // 最新のイベント内容に応じて点滅を設定.
    if(ui->tableIntBallEvents->item(0, 1)->text() == LABEL_LOGLEVEL_ALERT)
    {
        startBlinking(Color::S3);
    }
    else if(ui->tableIntBallEvents->item(0, 1)->text() == LABEL_LOGLEVEL_WARN)
    {
        startBlinking(Color::S2);
    }
}

void StatusWidget::enableTooltip(QTableWidgetItem& item, const QColor& color)
{
    auto text = item.text();
    if(color != QColor())
    {
        //ツールチップの文言に色を設定する.
        auto tooltipString = QString("<p style='white-space:pre;color:%1;'>%2<font color=</p>")
                                     .arg(Color::htmlRGB(color))
                                     .arg(text);
        item.setToolTip(tooltipString);
    }
    else
    {
        item.setToolTip(text);
    }
    item.setText(text.mid(0, 10) + "...");
}

void StatusWidget::disableTooltip(QTableWidgetItem& item)
{
    // ツールチップ文字列からHTMLタグを除いたメッセージ本文のみを抽出.
    item.setText(item.toolTip().remove(QRegExp("<[^>]*>")));
    item.setToolTip("");
}

void StatusWidget::changeTextColor(const int rowIndex, const QColor& color)
{
    ui->tableIntBallEvents->item(rowIndex, 0)->setTextColor(color);
    ui->tableIntBallEvents->item(rowIndex, 1)->setTextColor(color);
    ui->tableIntBallEvents->item(rowIndex, 2)->setTextColor(color);
}

void StatusWidget::playAlertSound()
{
    soundPlayerAlert_->play();
    if(soundPlayerAlertRingingTime_ > 0)
    {
        soundPlayerAlertStopTimer_->start();
    }
}

void StatusWidget::playWarningSound()
{
    soundPlayerWarning_->play();
    if(soundPlayerWarningRingingTime_ > 0)
    {
        soundPlayerWarningStopTimer_->start();
    }
}

void StatusWidget::stopPlayingAlertSound()
{
    soundPlayerAlert_->stop();
}

void StatusWidget::stopPlayingWarningSound()
{
    soundPlayerWarning_->stop();
}

QColor StatusWidget::borderColor() const
{
    return borderColor_;
}

void StatusWidget::setBorderColor(const QColor color)
{
    borderColor_ = color;
    ui->frame->setStyleSheet(QString("#frame {border: 5px solid %1;}").arg(Color::styleSheetRGB(color)));
}

void StatusWidget::startBlinking(const QColor& color)
{
    if(blinkAnimation_->state() != QPropertyAnimation::State::Stopped)
    {
        blinkAnimation_->stop();
    }
    blinkAnimation_->setDuration(BLINK_ANIMATION_DURATION_MSECS);
    blinkAnimation_->setKeyValueAt(0, DEFAULT_FRAME_COLOR);
    blinkAnimation_->setKeyValueAt(0.5, color);
    blinkAnimation_->setKeyValueAt(1, DEFAULT_FRAME_COLOR);
    blinkAnimation_->start();

    // 点滅停止ボタンの表示.
    ui->stopBlinkingButton->setStyleSheet(QString("QPushButton {"
                                          "background-color: %1;"
                                          "border-radius: 2px;"
                                          "border: 1px outset %2;"
                                          "}"
                                          "QToolTip {"
                                          "background-color: %1;"
                                          "padding: 4px;"
                                          "border: 1px solid %2;"
                                          "}").arg(Color::styleSheetRGB(Color::B1)).arg(Color::styleSheetRGB(color)));
    ui->stopBlinkingButton->show();
}

void StatusWidget::stopBlinking()
{
    if(blinkAnimation_->state() != QPropertyAnimation::State::Stopped)
    {
        blinkAnimation_->stop();
        setBorderColor(DEFAULT_FRAME_COLOR);

        // 点滅停止ボタンを非表示.
        ui->stopBlinkingButton->hide();
    }
}

void intball::StatusWidget::on_stopBlinkingButton_clicked()
{
    stopBlinking();
}
