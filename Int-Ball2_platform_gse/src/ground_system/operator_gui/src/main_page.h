#ifndef MAIN_PAGE_H
#define MAIN_PAGE_H

#include <QItemSelectionModel>
#include <QMainWindow>
#include <QMessageBox>
#include <boost/shared_ptr.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "camera_microphone_settings_dialog.h"
#include "led_settings_dialog.h"
#include "communication_software/Telemetry.h"
#include "model/intball_telemetry.h"
#include "model/route_information.h"
#include "telemetry_monitor.h"

namespace rviz
{
class Display;
class VisualizationManager;
} // namespace rviz

namespace tf
{
class StampedTransform;
class TransformListener;
class TransformBroadcaster;
} // namespace tf

namespace intball
{

namespace Ui
{
class MainPage;
}

class VideoController;
class IntBallTelemetry;
class TelemetrySubscriber;
class TelecommandClient;

class MainPage : public QWidget
{
    Q_OBJECT

public:
    enum class CONTROL_STATUS {
        NONE,
        MOVE
    };

    explicit MainPage(QWidget *parent = nullptr);
    ~MainPage();

    void initialize(const QString& pathRvizConfig,
                    intball::RouteInformation* routeInformation,
                    QItemSelectionModel* routeInformationSelectionModel,
                    intball::VideoController* videoController,
                    intball::IntBallTelemetry* intballTelemetry,
                    intball::TelecommandClient* telecommandClient,
                    intball::DockTelemetry* dockTelemetry);

    void setVideoArea(QWidget* video);

    void setStatusArea(QWidget* status);

public slots:
    void TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value);
    void TelemetryMonitor_modeTrackingStarted();

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void IntBall2Telemetry_rowsChanged(QList<int> rowList);
    void DockTelemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last);
    void RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last);
    void RouteInformation_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

    void on_videoRotateSlider_valueChanged(int value);

    void on_loadRouteButton_clicked();

    void on_videoRotateResetButton_clicked();

    void on_controlGoButton_clicked();

    void on_controlReadButton_clicked();

    void on_dockingButton_clicked();

    void on_emergencyButton_clicked();

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

    void on_takeSnapshotButton_clicked();

    void on_releaseButton_clicked();

    void on_executeAdditionalCommandButton_clicked();

    /**
     * @note on_transferGoButton_clickedに関しては画面切り替えを要するため
     *       GroudSystemMainWindowクラスにて定義する.
     */

    void on_controlAdditionalCommandComboBox_currentIndexChanged(int index);

private:
    intball::Ui::MainPage *ui;

    ros::Subscriber telemetrySubscriber;
    tf::TransformBroadcaster tfBroadcaster_;
    QScopedPointer<rviz::VisualizationManager> rvizVisualizationManager_;

    intball::VideoController* videoController_;

    intball::RouteInformation* routeInformation_;
    QItemSelectionModel* routeInformationSelectionModel_;

    intball::IntBallTelemetry* intballTelemetry_;
    intball::TelecommandClient* telecommandClient_;
    intball::DockTelemetry* dockTelemetry_;

    CONTROL_STATUS controlStatus_;


    bool isEnableMarkerCorrectionOnDock_;

    bool isOffNominalBattery_;
    bool isOffNominalDiskSpace_;
    bool isOffNominalTemperature_;

    bool isDockSwitchDocking_;

    bool isReleaseEnabled();
    bool isDockingEnabled();
    bool isNormalMoveCommandEnabled();
    bool isMovableModeAndCtlStatus();
    void switchingControlOperations();
    void switchingAdditionalCommands();
    void updateRoute(const int lastToBeRemoved = -1);
    void checkControlTelecommandResultCheck(const bool result, const CONTROL_STATUS &status);
};

} // namespace intball

#endif // MAIN_PAGE_H
