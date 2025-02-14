#include "telemetry_monitor.h"
#include <QTimer>
#include "model/intball_telemetry.h"
#include "model/dock_telemetry.h"
#include "gui_config_base.h"
#include "qdebug_custom.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

TelemetryMonitor::TelemetryMonitor(intball::IntBallTelemetry* intballTelemetry,
                                   intball::DockTelemetry* dockTelemetry,
                                   QObject *parent)
    : QObject(parent), intballTelemetry_(intballTelemetry), dockTelemetry_(dockTelemetry),
      followingIntBallMode_(false),
      followingPlatformMode_(false),
      temperatureWarningThreshold_(Config::valueAsInt(key::KEY_WARNING_TEMPERATURE)),
      temperatureAlertThreshold_(Config::valueAsInt(key::KEY_CRITICAL_TEMPERATURE)),
      temperatureNormalThreshold_(Config::valueAsInt(key::KEY_NORMAL_TEMPERATURE)),
      diskSpaceWarningThreshold_(Config::valueAsFloat(key::KEY_WARNING_STORAGE)),
      diskSpaceCriticalThreshold_(Config::valueAsFloat(key::KEY_CRITICAL_STORAGE)),
      batteryWarningThreshold_(Config::valueAsInt(key::KEY_WARNING_BATTERY_REMAIN)),
      batteryCriticalThreshold_(Config::valueAsInt(key::KEY_CRITICAL_BATTERY_REMAIN)),
      checkTimeToGoThreshold_(Config::valueAsFloat(key::KEY_TIME_TO_GO_THRESHOLD)),
      lastTemperature_(0.0), temperatureWarning_(false), temperatureCritical_(false), batteryWarning_(false), batteryCritical_(false),
      healthCheckIntBallTelemetry_(false), healthCheckDockTelemetry_(false),
      captured_(false), batteryFullyCharged_(false), batteryOutOfRange_(false), dockCharging_(false), moveNotYetComplete_(false),
      navigationStatus_(false), ctlStatus_(false), lastPlatformManagerMode_(false), userNodeStatus_(false), userLogicStatus_(false)
{
    qRegisterMetaType<intball::TelemetryMonitor::Event>("intball::TelemetryMonitor::Event");
    connect(intballTelemetry_, &IntBallTelemetry::written,
            this, &TelemetryMonitor::IntBall2Telemetry_written);
    connect(intballTelemetry_, &IntBallTelemetry::dataChanged,
            this, &TelemetryMonitor::IntBall2Telemetry_dataChanged);
    connect(intballTelemetry_, &IntBallTelemetry::rowsChanged,
            this, &TelemetryMonitor::IntBall2Telemetry_rowsChanged);
    connect(dockTelemetry_, &DockTelemetry::dataChanged,
            this, &TelemetryMonitor::DockTelemetry_dataChanged);

    // タイムアウト検知用タイマー(Int-Ball2).
    timerIntballTelemetry_ = new QTimer(this);
    connect(timerIntballTelemetry_, &QTimer::timeout, this, &TelemetryMonitor::timeoutIntballTelemetry);
    timerIntballTelemetry_->setSingleShot(true);
    timerIntballTelemetry_->setInterval(Config::valueAsInt(KEY_TIMEOUT_SECOND_INTBALL_TELEMETRY) * 1000);
    timerIntballTelemetry_->start();
    if(!timerIntballTelemetry_->isActive())
    {
        throw std::runtime_error("TelemetryMonitor timer (timerIntballTelemetry_) fails to start.");
    }

    // タイムアウト検知用タイマー(ドッキングステーション).
    timerDockTelemetry_ = new QTimer(this);
    connect(timerDockTelemetry_, &QTimer::timeout, this, &TelemetryMonitor::timeoutDockTelemetry);
    timerDockTelemetry_->setSingleShot(true);
    timerDockTelemetry_->setInterval(Config::valueAsInt(KEY_TIMEOUT_SECOND_DOCK_TELEMETRY) * 1000);
    timerDockTelemetry_->start();
    if(!timerDockTelemetry_->isActive())
    {
        throw std::runtime_error("TelemetryMonitor timer (timerDockTelemetry_) fails to start.");
    }
}

TelemetryMonitor::~TelemetryMonitor()
{
    timerIntballTelemetry_->stop();
    timerDockTelemetry_->stop();
}

void TelemetryMonitor::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    timerIntballTelemetry_->start();
    if(!timerIntballTelemetry_->isActive())
    {
        throw std::runtime_error("TelemetryMonitor timer (timerIntballTelemetry_) fails to start "
                                 "when calling IntBall2Telemetry_dataChanged.");
    }

    // Int-Ballのテレメトリを受信.
    if(!healthCheckIntBallTelemetry_)
    {
        emitEventSignal(Event::INTBALL_TELEMETRY_RECEIVED, QVariant());
        healthCheckIntBallTelemetry_ = true;
    }
}

void TelemetryMonitor::IntBall2Telemetry_written(QList<int> rowList)
{
    if(!followingIntBallMode_ && rowList.contains(static_cast<int>(telemetry::Index::MODE)))
    {
        // Int-Ball2の動作モード値を受信した＝地上システム側で動作モードの追従が可能になった　と判定する
        emit modeTrackingStarted();
        followingIntBallMode_ = true;
    }

    if(!followingPlatformMode_ && rowList.contains(static_cast<int>(telemetry::Index::PLATFORM_MANAGER_MODE)))
    {
        // Int-Ball2(技術実証プラットフォーム)の動作モード値を受信した＝地上システム側で動作モードの追従が可能になった　と判定する
        emit platformModeTrackingStarted();
        followingPlatformMode_ = true;
    }
}

void TelemetryMonitor::IntBall2Telemetry_rowsChanged(QList<int> rowList)
{
    // タスク管理のモード更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::MODE)))
    {
        emitEventSignal(Event::MODE_CHANGED, intballTelemetry_->data(telemetry::Index::MODE));

        switch(intballTelemetry_->data<unsigned char>(telemetry::Index::MODE))
        {
        case ib2_msgs::Mode::OPERATION:
            // 運用モード遷移.
            if(intballTelemetry_->data<int>(telemetry::Index::CTL_STATUS_TYPE) == ib2_msgs::CtlStatusType::STAND_BY)
            {
                // 誘導制御スタンバイのまま運用モードに遷移した場合はイベントを通知する.
                emitEventSignal(Event::CTL_ABNORMAL_SHUTDOWN, intballTelemetry_->data(telemetry::Index::MODE));
            }
            break;
        case ib2_msgs::Mode::IDLING:
            // アイドリングモード遷移.
            emitEventSignal(Event::MODE_IDLING, intballTelemetry_->data(telemetry::Index::MODE));
            break;
        case ib2_msgs::Mode::OFF_NOMINAL:
            // オフノミナル遷移.
            emitEventSignal(Event::MODE_OFF_NOMINAL, intballTelemetry_->data(telemetry::Index::MODE));
            break;
        }
    }

    // 新しい誘導制御アクションの実行.
    if(rowList.contains(static_cast<int>(telemetry::Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP)))
    {
        emitEventSignal(Event::START_CTL_COMMAND, QVariant());
        moveNotYetComplete_ = false;
    }

    // 誘導制御アクションフィードバックの更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::CTL_ACTION_FEEDBACK_TIME_TO_GO)))
    {
        if(!moveNotYetComplete_ &&
                (intballTelemetry_->data<ros::Duration>(
                     telemetry::Index::CTL_ACTION_FEEDBACK_TIME_TO_GO).toSec() <= static_cast<double>(checkTimeToGoThreshold_)))
        {
            emitEventSignal(Event::MOVE_NOT_YET_COMPLETE, QVariant());
            moveNotYetComplete_ = true;
        }
    }

    // 誘導制御アクション結果の更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP)))
    {
        emitEventSignal(Event::FINISH_CTL_COMMAND, intballTelemetry_->data(telemetry::Index::CTL_ACTION_RESULT_TYPE));

        auto status = intballTelemetry_->data<unsigned char>(telemetry::Index::CTL_ACTION_RESULT_TYPE);
        if(status != ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS)
        {
            // アクション失敗.
            emitEventSignal(Event::FAILED_CTL_COMMAND, intballTelemetry_->data(telemetry::Index::CTL_ACTION_RESULT_TYPE));
        }
    }

    // 誘導制御ステータスの更新
    if(rowList.contains(static_cast<int>(telemetry::Index::CTL_STATUS_TYPE)))
    {
        emitEventSignal(Event::CTL_STATUS_CHANGED, intballTelemetry_->data(telemetry::Index::CTL_STATUS_TYPE));
        auto status = intballTelemetry_->data<int>(telemetry::Index::CTL_STATUS_TYPE);

        if(captured_ && status != ib2_msgs::CtlStatusType::CAPTURED)
        {
            // キャプチャ状態から解除.
            captured_ = false;

            if(status == ib2_msgs::CtlStatusType::KEEP_POSE)
            {
                // キャプチャからのリリース.
                emitEventSignal(Event::RELEASE_CAPTURE, QVariant());
            }
        }

        if(!ctlStatus_ && status != ib2_msgs::CtlStatusType::STAND_BY)
        {
            emitEventSignal(Event::CTL_ACTIVE, QVariant());
            ctlStatus_ = true;
        }

        switch(status)
        {
        case ib2_msgs::CtlStatusType::SCAN:
            // スキャン動作開始.
            emitEventSignal(Event::START_SCAN, QVariant());
            break;
        case ib2_msgs::CtlStatusType::CAPTURED:
            // キャプチャ.
            emitEventSignal(Event::CAPTURED, QVariant());
            captured_ = true;
            break;
        case ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION:
            // 衝突による停止.
            emitEventSignal(Event::STOPPED_BY_COLLISION, QVariant());
            break;
        case ib2_msgs::CtlStatusType::STAND_BY:
            // スタンバイ.
            if(intballTelemetry_->data<unsigned char>(telemetry::Index::MODE) != INTBALL_MODE_UNKNOWN &&
                    intballTelemetry_->data<unsigned char>(telemetry::Index::MODE) != ib2_msgs::Mode::MAINTENANCE &&
                    intballTelemetry_->data<unsigned char>(telemetry::Index::MODE) != ib2_msgs::Mode::STANDBY &&
                    intballTelemetry_->data<unsigned char>(telemetry::Index::MODE) != ib2_msgs::Mode::IDLING)
            {
                // 運用中に誘導制御が異常終了した（STAND_BYに遷移した）と判定する.
                emitEventSignal(Event::CTL_ABNORMAL_SHUTDOWN, QVariant());
            }
            else
            {
                // 通常のSTAND_BY遷移.
                emitEventSignal(Event::CTL_STANDBY, QVariant());
                ctlStatus_ = false;
            }
            break;
        case ib2_msgs::CtlStatusType::KEEP_POSE:
            emitEventSignal(Event::KEEP_POSE, QVariant());
            break;
        }
    }

    // 航法アクション結果の更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::NAVIGATION_STARTUP_RESULT_TIMESTAMP)))
    {
        switch(intballTelemetry_->data<unsigned char>(telemetry::Index::NAVIGATION_STARTUP_RESULT_TYPE))
        {
        case ib2_msgs::NavigationStartUpResult::TIME_OUT:
            emitEventSignal(Event::NAVIGATION_ACTION_TIMEOUT, QVariant());
            break;
        case ib2_msgs::NavigationStartUpResult::ABORTED:
            emitEventSignal(Event::NAVIGATION_ACTION_ABORTED, QVariant());
            break;
        }
    }

    // 温度情報の更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::SYSTEM_MONITOR_TEMPERATURE)))
    {
        // 温度.
        auto value = intballTelemetry_->data<float>(telemetry::Index::SYSTEM_MONITOR_TEMPERATURE);

        if(!temperatureCritical_ && lastTemperature_ < temperatureAlertThreshold_ && temperatureAlertThreshold_ <= value)
        {
            // 危険温度以上に達した.
            temperatureCritical_ = true;
            emitEventSignal(Event::TEMPERATURE_IS_OVER_CRITICAL, QVariant());
        }
        else if(!temperatureWarning_ && lastTemperature_ < temperatureWarningThreshold_ && temperatureWarningThreshold_ <= value)
        {
            // 警告温度以上に達した.
            temperatureWarning_ = true;
            emitEventSignal(Event::TEMPERATURE_IS_OVER_WARNING, QVariant());
        }
        else if((temperatureCritical_ || temperatureWarning_) && temperatureNormalThreshold_ < lastTemperature_ && value <= temperatureNormalThreshold_)
        {
            // 正常値になった.
            temperatureCritical_ = false;
            temperatureWarning_ = false;
            emitEventSignal(Event::TEMPERATURE_COME_UNDER_WARNING, QVariant());
        }

        lastTemperature_ = value;
    }

    // ディスク容量値の更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::SYSTEM_MONITOR_DISK_SPACES)))
    {
        // ディスク容量.
        auto values = intballTelemetry_->data<QMap<QString, float>>(telemetry::Index::SYSTEM_MONITOR_DISK_SPACES);

        // 全ディスクの状態チェック.
        for(auto i = values.keyBegin(); i != values.keyEnd(); ++i)
        {
            if(((lastDiskSpaces_.count(*i) == 0) || (lastDiskSpaces_.value(*i) > diskSpaceCriticalThreshold_)) &&
                    (values.value(*i) <= diskSpaceCriticalThreshold_))
            {
                // 空きストレージ容量が危険値以下となった.
                emitEventSignal(Event::STORAGE_USAGE_IS_OVER_CRITICAL, QVariant(*i));
            }
            else if(((lastDiskSpaces_.count(*i) == 0) || (lastDiskSpaces_.value(*i) > diskSpaceWarningThreshold_)) &&
                    (values.value(*i) <= diskSpaceWarningThreshold_))
            {
                // 空きストレージ容量が警告値以下となった.
                emitEventSignal(Event::STORAGE_USAGE_IS_OVER_WARNING, QVariant(*i));
            }
            else if((lastDiskSpaces_.count(*i) > 0) &&
                    (lastDiskSpaces_.value(*i) <= diskSpaceWarningThreshold_) &&
                    (values.value(*i) > diskSpaceWarningThreshold_))
            {
                // 空きストレージ容量が警告値から復帰した.
                emitEventSignal(Event::STORAGE_USAGE_COME_UNDER_WARNING, QVariant(*i));
            }
            lastDiskSpaces_.insert(*i, values.value(*i));
        }
    }

    // バッテリー残量の更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN)))
    {
        auto batteryRemain = intballTelemetry_->data<char>(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN);
        if(static_cast<int>(batteryRemain) > 100 || static_cast<int>(batteryRemain) < 0)
        {
            // 現在値が範囲外

            if(!batteryOutOfRange_)
            {
                // フラグON、イベント通知
                emitEventSignal(Event::BATTERY_OUT_OF_RANGE, QVariant());
                batteryOutOfRange_ = true;

                // フラグOFF
                batteryWarning_ = false;
                batteryFullyCharged_ = false;
            }
        }
        else
        {
            // 現在値が範囲内
            // 範囲外フラグ以外のフラグをすべて処理した後、範囲外フラグをOFFにする

            if(static_cast<int>(batteryRemain) <= batteryCriticalThreshold_)
            {
                // バッテリー残量が危険値に達した
                if(!batteryCritical_)
                {
                    // フラグON、イベント通知
                    emitEventSignal(Event::BATTERY_LOW_CRITICAL, QVariant());
                    batteryCritical_ = true;

                    // フラグOFF
                    batteryWarning_ = false;
                    batteryFullyCharged_ = false;
                }
            }
            else if(static_cast<int>(batteryRemain) <= batteryWarningThreshold_)
            {
                // バッテリー残量が警告値に達した
                if(!batteryWarning_)
                {
                    // フラグON、イベント通知
                    emitEventSignal(Event::BATTERY_LOW, QVariant());
                    batteryWarning_ = true;

                    // フラグOFF
                    batteryFullyCharged_ = false;
                    batteryCritical_ = false;
                }
            }
            else if(static_cast<int>(batteryRemain) >= 100)
            {
                // バッテリー残量が100%になった
                if(!batteryFullyCharged_)
                {
                    // フラグON、イベント通知
                    emitEventSignal(Event::BATTERY_FULLY_CHARGED, intballTelemetry_->data(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN));
                    batteryFullyCharged_ = true;

                    // フラグOFF
                    batteryWarning_ = false;
                    batteryCritical_ = false;
                }
            }
            else
            {
                // バッテリー残量が正常値(21~99%)に戻った
                if (batteryWarning_ || batteryOutOfRange_ || batteryCritical_)
                {
                    // イベント通知
                    emitEventSignal(Event::BATTERY_NORMAL, QVariant());

                    // フラグOFF
                    batteryWarning_ = false;
                    batteryFullyCharged_ = false;
                    batteryCritical_ = false;
                }
            }

            // 範囲外フラグをOFF
            batteryOutOfRange_ = false;
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MIC_CAMERA_POWER)))
    {
        if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_CAMERA_POWER).status == ib2_msgs::PowerStatus::ON)
        {
            // カメラON.
            emitEventSignal(Event::CAMERA_ON, QVariant());
        }
        else
        {
            // カメラOFF.
            emitEventSignal(Event::CAMERA_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MIC_MICROPHONE_POWER)))
    {
        if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_MICROPHONE_POWER).status == ib2_msgs::PowerStatus::ON)
        {
            // マイクON.
            emitEventSignal(Event::MICROPHONE_ON, QVariant());
        }
        else
        {
            // マイクOFF.
            emitEventSignal(Event::MICROPHONE_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MIC_STREAMING_STATUS)))
    {
        if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_STREAMING_STATUS).status == ib2_msgs::PowerStatus::ON)
        {
            // ストリーミングON.
            emitEventSignal(Event::STREAMING_ON, QVariant());
        }
        else
        {
            // ストリーミングOFF.
            emitEventSignal(Event::STREAMING_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MIC_RECORDING_STATUS)))
    {
        if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_RECORDING_STATUS).status == ib2_msgs::PowerStatus::ON)
        {
            // 録画ON.
            emitEventSignal(Event::RECORDING_ON, QVariant());
        }
        else
        {
            // 録画OFF.
            emitEventSignal(Event::RECORDING_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH)))
    {
        if(intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH).status == ib2_msgs::PowerStatus::ON)
        {
            // カメラ用照明ON.
            emitEventSignal(Event::CAMERA_FLASH_ON, QVariant());
        }
        else
        {
            // カメラ用照明OFF.
            emitEventSignal(Event::CAMERA_FLASH_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み 航法機能.
        if(intballTelemetry_->data<unsigned char>(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::NAVIGATION_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::NAVIGATION_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み SLAMノード.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::SLAM_WRAPPER_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::SLAM_WRAPPER_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み 誘導制御.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::CTL_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::CTL_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み 推力.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::PROP_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::PROP_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み IMU.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::IMU_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::IMU_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み カメラ・マイク.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::CAMERA_MIC_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::CAMERA_MIC_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み LED左.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::LED_LEFT_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::LED_LEFT_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP)))
    {
        // パラメータの再読み込み LED右.

        if(intballTelemetry_->data<unsigned char>(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT)
                == ib2_msgs::UpdateParameterResponse::SUCCESS)
        {
            // 再読み込み成功.
            emitEventSignal(Event::LED_RIGHT_UPDATE_PARAMETER_SUCCESS, QVariant());
        }
        else
        {
            // 再読み込み失敗.
            emitEventSignal(Event::LED_RIGHT_UPDATE_PARAMETER_FAILED,
                          intballTelemetry_->data(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT));
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::MARKER_CORRECTION_TIMESTAMP)))
    {
        if(intballTelemetry_->data<short>(telemetry::Index::MARKER_CORRECTION_STATUS) == ib2_msgs::MarkerCorrectionResponse::SUCCESS)
        {
            // マーカー補正成功.
            emitEventSignal(Event::MARKER_CORRECTION_SUCCESS, QVariant());
        }
        else
        {
            // マーカー補正失敗.
            emitEventSignal(Event::MARKER_CORRECTION_FAILED, intballTelemetry_->data(telemetry::Index::MARKER_CORRECTION_STATUS));
        }
    }

    // Flight Softwareの起動状態
    if(rowList.contains(static_cast<int>(telemetry::Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS)))
    {
        if(intballTelemetry_->data<bool>(telemetry::Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS))
        {
            emitEventSignal(Event::FLIGHT_SOFTWARE_ON, QVariant());
        }else{
            /*
             * Flight Softwareが起動していない時点で
             * Int-Ball2の動作モードを追跡できていない状態とする.
             *
             * 動作モードの追跡が復旧できたかどうかの判定は
             * テレメトリで動作モード値を取得できたタイミングとする.
             */
            followingIntBallMode_ = false;
            followingPlatformMode_ = false;

            emitEventSignal(Event::FLIGHT_SOFTWARE_OFF, QVariant());
        }
    }

    /*** Platform GUI ***/
    // PlatformFlight Softwareの起動状態
    if(rowList.contains(static_cast<int>(telemetry::Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS)))
    {
        if(intballTelemetry_->data<bool>(telemetry::Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS))
        {
            emitEventSignal(Event::PLATFORM_FLIGHT_SOFTWARE_ON, QVariant());
        }else{
            emitEventSignal(Event::PLATFORM_FLIGHT_SOFTWARE_OFF, QVariant());
        }
    }

    // タスク管理のモード更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::PLATFORM_MANAGER_MODE)))
    {
        emitEventSignal(Event::PLATFORM_MODE_CHANGED, intballTelemetry_->data(telemetry::Index::PLATFORM_MANAGER_MODE));
        auto mode = intballTelemetry_->data<unsigned char>(telemetry::Index::PLATFORM_MANAGER_MODE);

        switch(mode)
        {
        case platform_msgs::Mode::USER_OFF:
            // ユーザプログラムOFF.
            if (lastPlatformManagerMode_ == platform_msgs::Mode::USER_IN_PROGRESS)
            {
                // ユーザ実装ロジックの実行中からユーザプログラムOFFへ遷移した場合、ユーザノードが異常終了したと判断する
                emitEventSignal(Event::USER_NODE_ABNORMAL_SHUTDOWN, QVariant());
            }
            else
            {
                if(userNodeStatus_)
                {
                    // ユーザノード終了
                    emitEventSignal(Event::USER_NODE_OFF, QVariant());
                }
                if(userLogicStatus_)
                {
                    // ユーザロジック終了
                    emitEventSignal(Event::USER_LOGIC_OFF, QVariant());
                }
            }
            userNodeStatus_ = false;
            userLogicStatus_ = false;
            break;
        case platform_msgs::Mode::USER_READY:
            // ユーザ実装ロジック開始待ち.
            if(!userNodeStatus_)
            {
                // ユーザノード開始
                emitEventSignal(Event::USER_NODE_ON, QVariant());
                userNodeStatus_ = true;
            }
            if(userLogicStatus_)
            {
                // ユーザロジック終了
                emitEventSignal(Event::USER_LOGIC_OFF, QVariant());
                userLogicStatus_ = false;
            }
            break;
        case platform_msgs::Mode::USER_IN_PROGRESS:
            // ユーザ実装ロジック実行中.
            if(!userNodeStatus_)
            {
                // ユーザノード開始
                emitEventSignal(Event::USER_NODE_ON, QVariant());
                userNodeStatus_ = true;
            }
            if(!userLogicStatus_)
            {
                // ユーザロジック開始
                emitEventSignal(Event::USER_LOGIC_ON, QVariant());
                userLogicStatus_ = true;
            }
            break;
        }

        lastPlatformManagerMode_ = mode;
    }

    // Operation typeの更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE)))
    {
        emitEventSignal(Event::OPERATION_TYPE_CHANGED, intballTelemetry_->data(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE));
    }

    // 航法ステータスの更新.
    if(rowList.contains(static_cast<int>(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS)))
    {
        emitEventSignal(Event::NAVIGATION_STATUS_CHANGED, intballTelemetry_->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS).status);

        auto status = intballTelemetry_->data<ib2_msgs::NavigationStatus>(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS).status;
        if(status == ib2_msgs::NavigationStatus::NAV_OFF)
        {
            emitEventSignal(Event::NAVIGATION_OFF, QVariant());
            navigationStatus_ = false;
        }
        if(!navigationStatus_ && status != ib2_msgs::NavigationStatus::NAV_OFF)
        {
            emitEventSignal(Event::NAVIGATION_ON, QVariant());
            navigationStatus_ = true;
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_MAIN_STREAMING_STATUS)))
    {
        if(intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_MAIN_STREAMING_STATUS).status == platform_msgs::PowerStatus::ON)
        {
            // メインカメラ ストリーミングON.
            emitEventSignal(Event::CAMERA_MAIN_STREAMING_ON, QVariant());
        }
        else
        {
            // メインカメラ ストリーミングOFF.
            emitEventSignal(Event::CAMERA_MAIN_STREAMING_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_LEFT_STREAMING_STATUS)))
    {
        if(intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_LEFT_STREAMING_STATUS).status == platform_msgs::PowerStatus::ON)
        {
            // 航法カメラ左 ストリーミングON.
            emitEventSignal(Event::CAMERA_LEFT_STREAMING_ON, QVariant());
        }
        else
        {
            // 航法カメラ左 ストリーミングOFF.
            emitEventSignal(Event::CAMERA_LEFT_STREAMING_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::CAMERA_RIGHT_STREAMING_STATUS)))
    {
        if(intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_RIGHT_STREAMING_STATUS).status == platform_msgs::PowerStatus::ON)
        {
            // 航法カメラ右 ストリーミングON.
            emitEventSignal(Event::CAMERA_RIGHT_STREAMING_ON, QVariant());
        }
        else
        {
            // 航法カメラ右 ストリーミングOFF.
            emitEventSignal(Event::CAMERA_RIGHT_STREAMING_OFF, QVariant());
        }
    }

    if(rowList.contains(static_cast<int>(telemetry::Index::MICROPHONE_STREAMING_STATUS)))
    {
        if(intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::MICROPHONE_STREAMING_STATUS).status == platform_msgs::PowerStatus::ON)
        {
            // マイク ストリーミングON.
            emitEventSignal(Event::MICROPHONE_STREAMING_ON, QVariant());
        }
        else
        {
            // マイク ストリーミングOFF.
            emitEventSignal(Event::MICROPHONE_STREAMING_OFF, QVariant());
        }
    }
}

void TelemetryMonitor::timeoutIntballTelemetry()
{
    INFO_START_FUNCTION();
    emitEventSignal(Event::TIMEOUT_TELEMETRY_INTBALL, QVariant());
    healthCheckIntBallTelemetry_ = false;

    /*
     * Int-Ball2テレメトリのタイムアウトが発生した時点で,
     * Int-Ball2の動作モードを追跡できていない状態となる.
     */
    followingIntBallMode_ = false;
    followingPlatformMode_ = false;
}

void TelemetryMonitor::DockTelemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    timerDockTelemetry_->start();
    if(!timerDockTelemetry_->isActive())
    {
        throw std::runtime_error("TelemetryMonitor timer (timerDockTelemetry_) fails to start "
                                 "when calling DockTelemetry_dataChanged.");
    }

    // Docking stationのテレメトリを受信.
    if(!healthCheckDockTelemetry_)
    {
        emitEventSignal(Event::DOCK_TELEMETRY_RECEIVED, QVariant());
        healthCheckDockTelemetry_ = true;
    }

    // 充電 開始/終了
    if(dockCharging_ != dockTelemetry_->isCharging())
    {
        // 保持している充電状態とテレメトリ値が異なる場合, イベント発生.

        if(dockTelemetry_->isCharging())
        {
            // 充電開始.
            emitEventSignal(Event::DOCK_CHARGING_ON, QVariant());
        }
        else
        {
            // 充電停止.
            emitEventSignal(Event::DOCK_CHARGING_OFF, QVariant());
        }

        dockCharging_ = dockTelemetry_->isCharging();
    }
}

void TelemetryMonitor::timeoutDockTelemetry()
{
    INFO_START_FUNCTION();
    emitEventSignal(Event::TIMEOUT_TELEMETRY_DOCK, QVariant());
    healthCheckDockTelemetry_ = false;
}

void TelemetryMonitor::emitEventSignal(Event event, QVariant value)
{
    emit detected(event, value);

    bool resetOffNominal = false; // オフノミナル解除の発生有無
    for(auto it = offNominalDefinitionMap.keyValueBegin(); it != offNominalDefinitionMap.keyValueEnd(); ++it)
    {
        // first = QMapのキー(OffNominalType), second = QMapの値(struct OffNominal)
        auto offNominalType = (*it).first;
        auto offNominalValues = (*it).second;

        // オフノミナル事象発生の確認
        if(offNominalValues.eventOffnominalOn.contains(event) && currentOffNominalStatus_[offNominalType] == false)
        {
            LOG_WARNING() << "Off-nominal event detected: " << OffNominalTypeNames.at(static_cast<int>(offNominalType));
            currentOffNominalStatus_[offNominalType] = true;
            emit offNominalDetected(offNominalType);
        }

        // オフノミナル事象の解除確認
        if(offNominalValues.eventOffnominalOff.contains(event) && currentOffNominalStatus_[offNominalType] == true)
        {
            LOG_INFO() << "Off-nominal event released: " << OffNominalTypeNames.at(static_cast<int>(offNominalType));
            currentOffNominalStatus_[offNominalType] = false;
            resetOffNominal = true;
        }
    }

    // 全オフノミナル事象の解除確認とシグナル通知
    if(resetOffNominal && !currentOffNominalStatus_.values().contains(true))
    {
        LOG_INFO() << "All off-nominal events have been released";
        emit releaseAllOffNominal();
    }
}
