#ifndef TELEMETRY_MONITOR_H
#define TELEMETRY_MONITOR_H

#include <QObject>
#include <QTimer>
#include "model/intball_telemetry.h"
#include "model/dock_telemetry.h"

namespace intball
{

class TelemetryMonitor : public QObject
{
    Q_OBJECT
public:

    /**
     * @brief 検知するイベントの定義.
     */
    enum class Event {
        MODE_CHANGED,
        MODE_IDLING,
        MODE_OFF_NOMINAL,
        MOVE_NOT_YET_COMPLETE,
        RELEASE_CAPTURE,
        START_SCAN,
        CAPTURED,
        STOPPED_BY_COLLISION,
        CTL_STANDBY,
        CTL_ABNORMAL_SHUTDOWN,
        KEEP_POSE,
        INTBALL_TELEMETRY_RECEIVED,
        TIMEOUT_TELEMETRY_INTBALL,
        DOCK_TELEMETRY_RECEIVED,
        TIMEOUT_TELEMETRY_DOCK,
        START_CTL_COMMAND,
        FINISH_CTL_COMMAND,
        FAILED_CTL_COMMAND,
        TEMPERATURE_IS_OVER_WARNING,
        TEMPERATURE_IS_OVER_CRITICAL,
        TEMPERATURE_COME_UNDER_WARNING,
        STORAGE_USAGE_IS_OVER_WARNING,
        STORAGE_USAGE_IS_OVER_CRITICAL,
        STORAGE_USAGE_COME_UNDER_WARNING,
        BATTERY_LOW,
        BATTERY_LOW_CRITICAL,
        BATTERY_FULLY_CHARGED,
        BATTERY_OUT_OF_RANGE,
        BATTERY_NORMAL,
        CAMERA_ON,
        CAMERA_OFF,
        MICROPHONE_ON,
        MICROPHONE_OFF,
        STREAMING_ON,
        STREAMING_OFF,
        RECORDING_ON,
        RECORDING_OFF,
        CAMERA_FLASH_ON,
        CAMERA_FLASH_OFF,
        NAVIGATION_ACTION_TIMEOUT,
        NAVIGATION_ACTION_ABORTED,
        NAVIGATION_UPDATE_PARAMETER_SUCCESS,
        NAVIGATION_UPDATE_PARAMETER_FAILED,
        IMU_UPDATE_PARAMETER_SUCCESS,
        IMU_UPDATE_PARAMETER_FAILED,
        SLAM_WRAPPER_UPDATE_PARAMETER_SUCCESS,
        SLAM_WRAPPER_UPDATE_PARAMETER_FAILED,
        CTL_UPDATE_PARAMETER_SUCCESS,
        CTL_UPDATE_PARAMETER_FAILED,
        PROP_UPDATE_PARAMETER_SUCCESS,
        PROP_UPDATE_PARAMETER_FAILED,
        CAMERA_MIC_UPDATE_PARAMETER_SUCCESS,
        CAMERA_MIC_UPDATE_PARAMETER_FAILED,
        LED_LEFT_UPDATE_PARAMETER_SUCCESS,
        LED_LEFT_UPDATE_PARAMETER_FAILED,
        LED_RIGHT_UPDATE_PARAMETER_SUCCESS,
        LED_RIGHT_UPDATE_PARAMETER_FAILED,
        MARKER_CORRECTION_SUCCESS,
        MARKER_CORRECTION_FAILED,
        DOCK_CHARGING_ON,
        DOCK_CHARGING_OFF,
        FLIGHT_SOFTWARE_ON,
        FLIGHT_SOFTWARE_OFF,
        PLATFORM_FLIGHT_SOFTWARE_ON,
        PLATFORM_FLIGHT_SOFTWARE_OFF,
        PLATFORM_MODE_CHANGED,
        OPERATION_TYPE_CHANGED,
        NAVIGATION_STATUS_CHANGED,
        NAVIGATION_ON,
        NAVIGATION_OFF,
        CTL_STATUS_CHANGED,
        CTL_ACTIVE,
        CAMERA_MAIN_STREAMING_ON,
        CAMERA_MAIN_STREAMING_OFF,
        CAMERA_LEFT_STREAMING_ON,
        CAMERA_LEFT_STREAMING_OFF,
        CAMERA_RIGHT_STREAMING_ON,
        CAMERA_RIGHT_STREAMING_OFF,
        MICROPHONE_STREAMING_ON,
        MICROPHONE_STREAMING_OFF,
        USER_NODE_ON,
        USER_NODE_OFF,
        USER_NODE_ABNORMAL_SHUTDOWN,
        USER_LOGIC_ON,
        USER_LOGIC_OFF,
    };


    /**
     * @brief オフノミナル事象の定義.
     */
    enum class OffNominalType {
        Temperature,
        Storage,
        Battery,
        TelemetryTimeOut
    };

    QList<QString> OffNominalTypeNames = {
        "Temperature",
        "Storage",
        "Battery",
        "TelemetryTimeOut",
    };

    TelemetryMonitor(intball::IntBallTelemetry* intballTelemetry,
                     intball::DockTelemetry* dockTelemetry,
                     QObject *parent = nullptr);
    virtual ~TelemetryMonitor();

signals:
    /**
     * @brief イベントの発生を検知した.
     * @param event イベント内容.
     * @param value イベント発生時のパラメータ.
     * @note TelemetryMonitorの内部実装では必ずemitEventSignal経由で呼び出す.
     */
    void detected(Event event, QVariant value);

    /**
     * @brief オフノミナル事象の検知.
     * @param type 検知したオフノミナル事象の種別.
     * @note detectedイベントと重複して発生する.
     */
    void offNominalDetected(OffNominalType type);

    /**
     * @brief 全てのオフノミナル事象が解除された.
     * @note detectedイベントと重複して発生する.
     */
    void releaseAllOffNominal();

    /**
     * @brief Int-Ball2のモード遷移の追従を開始した.
     */
    void modeTrackingStarted();

    /**
     * @brief Int-Ball2(技術実証プラットフォーム)のモード遷移の追従を開始した.
     */
    void platformModeTrackingStarted();

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void IntBall2Telemetry_written(QList<int> rowList);
    void IntBall2Telemetry_rowsChanged(QList<int> rowList);
    void timeoutIntballTelemetry();
    void DockTelemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void timeoutDockTelemetry();

private:
    intball::IntBallTelemetry* intballTelemetry_;
    intball::DockTelemetry* dockTelemetry_;
    QTimer *timerIntballTelemetry_;
    QTimer *timerDockTelemetry_;
    bool followingIntBallMode_;
    bool followingPlatformMode_;
    float temperatureWarningThreshold_;
    float temperatureAlertThreshold_;
    float temperatureNormalThreshold_;
    float diskSpaceWarningThreshold_;
    float diskSpaceCriticalThreshold_;
    int batteryWarningThreshold_;
    int batteryCriticalThreshold_;
    float checkTimeToGoThreshold_;
    float lastTemperature_;
    QMap<QString, float> lastDiskSpaces_;
    bool temperatureWarning_;
    bool temperatureCritical_;
    bool batteryWarning_;
    bool batteryCritical_;
    bool healthCheckIntBallTelemetry_;
    bool healthCheckDockTelemetry_;
    bool captured_;
    bool batteryFullyCharged_;
    bool batteryOutOfRange_;
    bool dockCharging_;
    bool moveNotYetComplete_;
    bool navigationStatus_;
    bool ctlStatus_;
    unsigned char lastPlatformManagerMode_;
    bool userNodeStatus_;
    bool userLogicStatus_;

    /**
     * @brief オフノミナル事象の発生状況.
     */
    QMap<OffNominalType, bool> currentOffNominalStatus_ = {
        {OffNominalType::Temperature,      false},
        {OffNominalType::Storage,          false},
        {OffNominalType::Battery,          false},
        {OffNominalType::TelemetryTimeOut, false},
    };

    /**
     * @brief オフノミナル事象の設定用構造体.
     */
    struct OffNominalDefinition {
        /**
         * @brief オフノミナル発生（ON）と判定するイベントの定義.
         */
        QList<intball::TelemetryMonitor::Event> eventOffnominalOn;

        /**
         * @brief オフノミナル解除（OFF）と判定するイベントの定義.
         */
        QList<intball::TelemetryMonitor::Event> eventOffnominalOff;
    };

    /**
     * @brief オフノミナル事象を検知および解除するイベントの定義.
     */
    const QMap<OffNominalType, OffNominalDefinition> offNominalDefinitionMap = {
        {OffNominalType::Temperature,      { {Event::TEMPERATURE_IS_OVER_WARNING, Event::TEMPERATURE_IS_OVER_CRITICAL},
                                             {Event::TEMPERATURE_COME_UNDER_WARNING} }},
        {OffNominalType::Storage,          { {Event::STORAGE_USAGE_IS_OVER_WARNING, Event::STORAGE_USAGE_IS_OVER_CRITICAL},
                                             {Event::STORAGE_USAGE_COME_UNDER_WARNING} }},
        {OffNominalType::Battery,          { {Event::BATTERY_LOW, Event::BATTERY_LOW_CRITICAL, Event::BATTERY_OUT_OF_RANGE},
                                             {Event::BATTERY_NORMAL, Event::BATTERY_FULLY_CHARGED} }},
        {OffNominalType::TelemetryTimeOut, { {Event::TIMEOUT_TELEMETRY_INTBALL},
                                             {Event::INTBALL_TELEMETRY_RECEIVED} }},
    };

    /**
     * @brief イベント関連のシグナルをemitする.
     * @param event イベント内容.
     * @param value イベント発生時のパラメータ.
     */
    void emitEventSignal(Event event, QVariant value);
};

} // namespace intball

Q_DECLARE_METATYPE(intball::TelemetryMonitor::Event);
Q_DECLARE_METATYPE(intball::TelemetryMonitor::OffNominalType);

#endif // TELEMETRY_MONITOR_H
