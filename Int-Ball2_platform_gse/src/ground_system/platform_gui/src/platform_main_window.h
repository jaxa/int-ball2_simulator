#ifndef INTBALL_PLATFORM_MAIN_WINDOW_H
#define INTBALL_PLATFORM_MAIN_WINDOW_H

#include <QMainWindow>
#include <QDateTime>
#include <QTableWidgetItem>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <ros/ros.h>
#include "common_log_object.h"
#include "monitor_status_model.h"
#include "model/dock_telemetry.h"
#include "model/intball_telemetry.h"
#include "telemetry_monitor.h"

namespace intball {

namespace Ui
{
class PlatformMainWindow;
}

class DockTelemetry;
class IntBallTelemetry;
class TelemetrySubscriber;
class TelecommandClient;
class StatusWidget;

class PlatformMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    static const int BATTERY_MARGIN_RIGHT = 0;
    static const int STORAGE_MARGIN_RIGHT = 70;
    static const int COLUMN_WIDTH_LOG_MESSAGE = 400;
    static const int BLINK_ANIMATION_DURATION_MSECS = 4000;
    static const QColor DEFAULT_FRAME_COLOR;
    static const QString SUFFIX_ALIVE_STATUS_MAIN_CAMERA;
    static const QString SUFFIX_ALIVE_STATUS_LEFT_CAMERA;
    static const QString SUFFIX_ALIVE_STATUS_RIGHT_CAMERA;
    static const QString SUFFIX_ALIVE_STATUS_MICROPHONE;

    explicit PlatformMainWindow(QWidget *parent = nullptr);
    ~PlatformMainWindow();

protected:
    void closeEvent(QCloseEvent *event);

public slots:

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void on_controlMainCameraStreamingButton_clicked();
    void on_controlLeftCameraStreamingButton_clicked();
    void on_controlRightCameraStreamingButton_clicked();
    void on_controlMicrophoneStreamingButton_clicked();
    void on_pushButtonUserNodeStart_clicked();
    void on_pushButtonUserNodeStop_clicked();
    void on_pushButtonUserLogicStart_clicked();
    void on_pushButtonUserLogicStop_clicked();
    void on_pushButtonOperationTypeOn_clicked();
    void on_pushButtonOperationTypeOff_clicked();
    void switchingUserLaunchFileComboBox();

    void on_controlGoButton_clicked();
    void on_controlPositionUp_clicked();
    void on_controlPositionRight_clicked();
    void on_controlPositionDown_clicked();
    void on_controlPositionLeft_clicked();
    void on_controlPositionFront_clicked();
    void on_controlPositionBack_clicked();
    void on_controlAttitudeRollCounterClock_clicked();
    void on_controlAttitudeRollClock_clicked();
    void on_controlAttitudePitchCounterClock_clicked();
    void on_controlAttitudePitchClock_clicked();
    void on_controlAttitudeYawCounterClock_clicked();
    void on_controlAttitudeYawClock_clicked();

    void TelemetryMonitor_offNominalDetected(TelemetryMonitor::OffNominalType type);
    void TelemetryMonitor_releaseAllOffNominal();

private:
    Ui::PlatformMainWindow *ui;
    ros::Subscriber telemetrySubscriber;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::DockTelemetry* dockTelemetry_;
    intball::TelemetryMonitor* telemetryMonitor_;
    intball::TelecommandClient* telecommandClient_;
    intball::TelemetrySubscriber* telemetrySubscriber_;

    /**
     * @brief 利用可能なユーザプログラムのリスト.設定ファイルから読み出す.
     */
    QJsonArray userPackageList_;

    /**
     * @brief 選択可能なコンテナイメージのリスト.設定ファイルから読み出す.
     */
    QJsonArray containerImageList_;

    /**
     * @brief 監視結果をツリー表示するためのデータモデル.
     */
    MonitorStatusModel monitorStatusModel_;

    /**
     * @brief 古い監視結果の表示を消去する際の閾値.
     */
    int monitorStatusClearInterval_;

    /**
     * @brief 最後に読み出した監視結果タイムスタンプ.
     */
    QDateTime lastMonitorStatusCheckTime_;

    /**
     * @brief オフノミナル状態の検知有無.
     * @note TelemetryMonitorからのシグナル契機で更新する.
     */
    bool isOffNominalDetected_;

    bool isNormalMoveCommandEnabled();
    bool isSetOperationTypeEnabled();
    bool isLaunchEnabled();
    bool isLogicEnabled();
    bool isMainCameraEnabled();
    bool isLeftCameraEnabled();
    bool isRightCameraEnabled();
    bool isMicrophoneEnabled();
    bool isPlatformFlightSoftwareStarted();
    void getUserPackageList();
    void getContainerImageList();
    void displayUserNodeMessage(std::vector<char> userNodeStatusData);
    void setProgressBarStyleSheet(QWidget* progressBar, const QColor& color, const int marginRight);
    void switchingControlOperations();

    void sendMainCameraStreaming();
    void sendLeftCameraStreaming();
    void sendRightCameraStreaming();
    void sendMicrophoneStreaming();
    void sendOperationType(const unsigned char type);
    void sendTargetGoalAbsolute();
    void sendTargetGoalRelative(const QVector3D& position, const QQuaternion& orientation);
    void sendUserNode(const bool on, const QString& user, const QString& launch, const QString& container);
    void sendUserLogic(const bool on, const platform_msgs::UserLogic& userLogic);

};


} // namespace intball
#endif // INTBALL_PLATFORM_MAIN_WINDOW_H
