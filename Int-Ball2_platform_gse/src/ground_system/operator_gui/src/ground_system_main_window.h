#ifndef GROUND_SYSTEM_MAIN_WINDOW_H
#define GROUND_SYSTEM_MAIN_WINDOW_H

#include <QItemSelectionModel>
#include <QMainWindow>
#include <memory>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include "communication_software/msg/telemetry.hpp"
#include "common_log_object.h"
#include "model/intball_telemetry.h"
#include "model/route_information.h"
#include "operator_gui_common.h"
#include "telemetry_monitor.h"

namespace rviz_common
{
class Display;
class VisualizationManager;
} // namespace rviz_common

namespace intball
{

namespace Ui
{
class GroundSystemMainWindow;
}

class DockTelemetry;
class IntBallTelemetry;
class StatusWidget;
class TelemetrySubscriber;
class TelecommandClient;
class VideoController;
class GroundSystemMainWindow : public QMainWindow
{
    Q_OBJECT

public:

    /**
     * @brief メイン画面（MainPage）のインデックス値.
     */
    const static int PAGE_MAIN = 0;

    /**
     * @brief 経路編集画面（EditingPage）のインデックス値.
     */
    const static int PAGE_EDIT = 1;

    /**
     * @brief 経路移動実行画面（ExecutionPage）のインデックス値.
     */
    const static int PAGE_EXECUTION = 2;

    /**
     * @brief rviz画面上に描画するマーカー情報のインデックス値:経路の線.
     */
    const static int MARKER_INDEX_LINE = 0;

    /**
     * @brief rviz画面上に描画するマーカー情報のインデックス値:経路の点.
     */
    const static int MARKER_INDEX_POINT = 1;

    /**
     * @brief GroundSystemMainWindowコンストラクタ.
     * @param parent 親のQWidget.
     */
    explicit GroundSystemMainWindow(QWidget *parent = nullptr);

    /**
     * @brief rviz2パネルを含む各ページの初期化.
     * ウィンドウ表示後に呼び出す必要がある（Ogre SceneManagerの遅延初期化のため）.
     */
    void initializePages();

    /**
     * @brief ~GroundSystemMainWindowデストラクタ.
     */
    virtual ~GroundSystemMainWindow();

signals:
    void statusWidgetEvent(CommandLog log);

protected:
    void closeEvent(QCloseEvent *event);

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void switchPageEvent(intball::SwitchPageEvent event);

    // RouteInformation.
    void RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last);
    void RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last);

    // MainPage.
    void on_editGoalButton_clicked();
    void on_transferGoButton_clicked();

    // TelemetryMonitor.
    void TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value);

    // TelemetryClient.
    void clientLog(CommandLog log);

private:
    intball::Ui::GroundSystemMainWindow *ui;
    int videoAreaDefaultFixedWidth_;
    int statusDefaultFixedWidth_;
    qreal videoAreaAspectRatio_;

    tf2_ros::TransformBroadcaster tfBroadcaster_;
    QScopedPointer<rviz_common::VisualizationManager> rvizVisualizationManager_;

    intball::VideoController* videoController_;

    intball::RouteInformation* routeInformation_;
    QItemSelectionModel* routeInformationSelectionModel_;

    QList<RoutePoint> beforeEditRoutePointList_;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::TelemetrySubscriber* telemetrySubscriber_;
    intball::TelecommandClient* telecommandClient_;
    intball::TelemetryMonitor* telemetryMonitor_;

    intball::DockTelemetry* dockTelemetry_;

    QWidget* videoAreaWidget_;

    intball::StatusWidget* statusAreaWidget_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisherRoute_;
    visualization_msgs::msg::MarkerArray markerArrayView_;
    visualization_msgs::msg::MarkerArray markerArrayDelete_;
    QVector3D intballPositionBeforeMoving_;

    // rviz遅延初期化用.
    QString pathRvizConfig_;
    QString pathRvizConfigCamera_;
    bool editPageRvizInitialized_ = false;
    bool execPageRvizInitialized_ = false;

    void updateRoute(const int lastToBeRemoved = -1);
    void switchToMainPage();
    void switchToEditPage();
    void switchToExecutionPage();
    void stopCurrentPageRendering();
    void publishRouteMarkerArray(const int firstToBeRemoved = -1, const int lastToBeRemoved = -1);

};

} // namespace intball

#endif // GROUND_SYSTEM_MAIN_WINDOW_H
