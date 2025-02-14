#include "ground_system_main_window.h"
#include "ui_ground_system_main_window.h"
#include <string>
#include <vector>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QSettings>
#include <QTextCodec>
#include <QTimer>
#include <QUrl>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/yaml_config_reader.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>
#include "amount_slider_widget.h"
#include "communication_software/Telemetry.h"
#include "common_log_object.h"
#include "dialog_factory.h"
#include "editing_page.h"
#include "gui_common.h"
#include "operator_gui_config.h"
#include "operator_gui_common.h"
#include "ib2_msgs.h"
#include "main_page.h"
#include "model/dock_telemetry.h"
#include "model/intball_telemetry.h"
#include "model/route_information.h"
#include "qdebug_custom.h"
#include "ros_common.h"
#include "route_setting.h"
#include "status_widget.h"
#include "video_controller.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include "telemetry_telecommand_config.h"
#include "telecommand_client.h"
#include "telemetry_monitor.h"
#include "telemetry_subscriber.h"


using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;
using namespace intball::telemetry;
using namespace intball::message;

GroundSystemMainWindow::GroundSystemMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::GroundSystemMainWindow),
    videoAreaAspectRatio_(9.0 / 16.0),
    intballPositionBeforeMoving_(QVector3D())
{
    qRegisterMetaType<intball::RoutePoint>("intball::RoutePoint");
    qRegisterMetaType<QList<intball::RoutePoint>>("QList<intball::RoutePoint>");
    qRegisterMetaTypeStreamOperators<intball::RoutePoint>("intball::RoutePoint");
    qRegisterMetaTypeStreamOperators<QList<intball::RoutePoint>>("QList<intball::RoutePoint>");

    ui->setupUi(this);
    ui->stackedWidget->setCurrentIndex(PAGE_MAIN);

    // Video.
    videoAreaWidget_ = new QWidget(this);
    videoController_ = new VideoController(videoAreaWidget_->winId(), this);
    auto videoInput = Config::valueAsEnum(VideoController::InputType(), KEY_VIDEO_TYPE);
    auto videoPath = Config::value(KEY_VIDEO_INPUT);
    videoController_->start(videoInput, videoPath.toString().toStdString().c_str());

    // 経路のデータモデル登録.
    routeInformation_ = new RouteInformation(this);

    /*
     * QtのViewクラスでは,setModelが呼ばれた際に内部でSelectionModelが自動生成されるため
     * 必ずsetModelを呼び出した後にsetSelectionModelを呼び出す必要がある.
     */
    routeInformationSelectionModel_ = new QItemSelectionModel(routeInformation_);
    routeInformation_->insertData(0, QVector3D(), QQuaternion());

    connect(routeInformation_, &RouteInformation::rowsInserted, this, &GroundSystemMainWindow::RouteInformation_rowsInserted);
    connect(routeInformation_, &RouteInformation::rowsAboutToBeRemoved, this, &GroundSystemMainWindow::RouteInformation_rowsAboutToBeRemoved);
    connect(routeInformation_, &RouteInformation::dataChanged, this, &GroundSystemMainWindow::RouteInformation_dataChanged);

    // テレメトリ.
    intballTelemetry_ = new IntBallTelemetry(this);
    connect(intballTelemetry_, &IntBallTelemetry::dataChanged,
            this, &GroundSystemMainWindow::IntBall2Telemetry_dataChanged);
    dockTelemetry_ = new DockTelemetry(this);

    // テレメトリからのイベント検知（モニター）.
    telemetryMonitor_ = new TelemetryMonitor(intballTelemetry_, dockTelemetry_, this);
    connect(telemetryMonitor_, &TelemetryMonitor::detected,
            this, &GroundSystemMainWindow::TelemetryMonitor_detected);
    connect(telemetryMonitor_, &TelemetryMonitor::detected,
            ui->mainPageWidget, &MainPage::TelemetryMonitor_detected);
    connect(telemetryMonitor_, &TelemetryMonitor::modeTrackingStarted,
            ui->mainPageWidget, &MainPage::TelemetryMonitor_modeTrackingStarted);
    connect(telemetryMonitor_, &TelemetryMonitor::detected,
            ui->execPageWidget, &ExecutionPage::TelemetryMonitor_detected);


    // テレコマンド送信用クライアント.
    telecommandClient_ = new TelecommandClient(*getNodeHandle(), this);
    connect(telecommandClient_, &TelecommandClient::executed, this, &GroundSystemMainWindow::clientLog);

    // 各種ページの初期化.
    QString pathRvizConfig = Config::packagePath() +
                             Config::valueAsString(KEY_RVIZ_CONFIG_ROUTE);
    ui->mainPageWidget->initialize(pathRvizConfig, routeInformation_, routeInformationSelectionModel_, videoController_,
                                   intballTelemetry_, telecommandClient_, dockTelemetry_);
    ui->mainPageWidget->setVideoArea(videoAreaWidget_);

    ui->editPageWidget->initialize(pathRvizConfig, routeInformation_, routeInformationSelectionModel_, telecommandClient_);
    ui->execPageWidget->initialize(pathRvizConfig, routeInformation_, telecommandClient_, intballTelemetry_);

    // テレメトリ情報表示用ウィジット.
    statusAreaWidget_ = new StatusWidget(this);
    statusAreaWidget_->initialize(intballTelemetry_, dockTelemetry_, telemetryMonitor_);
    // 一部ログはテレメトリ情報表示用ウィジットにも表示する.
    connect(telecommandClient_, &TelecommandClient::executed, this, &GroundSystemMainWindow::statusWidgetEvent);
    ui->mainPageWidget->setStatusArea(statusAreaWidget_);

    // rviz画面の経路用マーカーのPublish.
    publisherRoute_ = getNodeHandle()->advertise<visualization_msgs::MarkerArray>("marker_route", 100);
    //rvizパネルにマーカーを配置するためのメッセージの初期化.
    markerArrayView_.markers.resize(2);

    // 経路の線を配置する.
    markerArrayView_.markers[MARKER_INDEX_LINE].header.frame_id = rosframe::ISS_BODY;
    markerArrayView_.markers[MARKER_INDEX_LINE].header.stamp = ros::Time(0);
    markerArrayView_.markers[MARKER_INDEX_LINE].ns = "route";
    markerArrayView_.markers[MARKER_INDEX_LINE].id = 0;
    markerArrayView_.markers[MARKER_INDEX_LINE].lifetime = ros::Duration(0);

    markerArrayView_.markers[MARKER_INDEX_LINE].type = visualization_msgs::Marker::LINE_STRIP;
    markerArrayView_.markers[MARKER_INDEX_LINE].action = visualization_msgs::Marker::MODIFY;
    markerArrayView_.markers[MARKER_INDEX_LINE].scale.x = 0.1;
    markerArrayView_.markers[MARKER_INDEX_LINE].pose.orientation.w = 1.0;

    markerArrayView_.markers[MARKER_INDEX_LINE].color.r = 0.0f;
    markerArrayView_.markers[MARKER_INDEX_LINE].color.g = 1.0f;
    markerArrayView_.markers[MARKER_INDEX_LINE].color.b = 0.0f;
    markerArrayView_.markers[MARKER_INDEX_LINE].color.a = 1.0f;

    // 経路の点を配置する.
    markerArrayView_.markers[MARKER_INDEX_POINT].header.frame_id = rosframe::ISS_BODY;
    markerArrayView_.markers[MARKER_INDEX_POINT].header.stamp = ros::Time(0);
    markerArrayView_.markers[MARKER_INDEX_POINT].ns = "route";
    markerArrayView_.markers[MARKER_INDEX_POINT].id = 1;
    markerArrayView_.markers[MARKER_INDEX_POINT].lifetime = ros::Duration(0);

    markerArrayView_.markers[MARKER_INDEX_POINT].type = visualization_msgs::Marker::SPHERE_LIST;
    markerArrayView_.markers[MARKER_INDEX_POINT].action = visualization_msgs::Marker::MODIFY;
    markerArrayView_.markers[MARKER_INDEX_POINT].scale.x = 0.2;
    markerArrayView_.markers[MARKER_INDEX_POINT].pose.orientation.w = 1.0;

    markerArrayView_.markers[MARKER_INDEX_POINT].color.r = 1.0f;
    markerArrayView_.markers[MARKER_INDEX_POINT].color.g = 0.0f;
    markerArrayView_.markers[MARKER_INDEX_POINT].color.b = 0.0f;
    markerArrayView_.markers[MARKER_INDEX_POINT].color.a = 1.0f;

    //rvizパネルからマーカーを削除するためのメッセージの初期化.
    markerArrayDelete_.markers.resize(2);

    // 経路の点を削除する.
    markerArrayDelete_.markers[MARKER_INDEX_LINE].header.frame_id = rosframe::ISS_BODY;
    markerArrayDelete_.markers[MARKER_INDEX_LINE].header.stamp = ros::Time(0);
    markerArrayDelete_.markers[MARKER_INDEX_LINE].ns = "route";
    markerArrayDelete_.markers[MARKER_INDEX_LINE].id = 0;
    markerArrayDelete_.markers[MARKER_INDEX_LINE].lifetime = ros::Duration(0);
    markerArrayDelete_.markers[MARKER_INDEX_LINE].action = visualization_msgs::Marker::DELETE;

    // 経路の線を削除する.
    markerArrayDelete_.markers[MARKER_INDEX_POINT].header.frame_id = rosframe::ISS_BODY;
    markerArrayDelete_.markers[MARKER_INDEX_POINT].header.stamp = ros::Time(0);
    markerArrayDelete_.markers[MARKER_INDEX_POINT].ns = "route";
    markerArrayDelete_.markers[MARKER_INDEX_POINT].id = 1;
    markerArrayDelete_.markers[MARKER_INDEX_POINT].lifetime = ros::Duration(0);
    markerArrayDelete_.markers[MARKER_INDEX_POINT].action = visualization_msgs::Marker::DELETE;

    // ページ切り替えイベント
    connect(ui->execPageWidget, &ExecutionPage::readyToSwitch, this, &GroundSystemMainWindow::switchPageEvent);
    connect(ui->editPageWidget, &EditingPage::readyToSwitch, this, &GroundSystemMainWindow::switchPageEvent);

    // その他初期化が完了してからテレメトリの受信処理を開始する.
    telemetrySubscriber_ = new TelemetrySubscriber(this);
    telemetrySubscriber_->start(*getNodeHandle(), intballTelemetry_, dockTelemetry_);
}

GroundSystemMainWindow::~GroundSystemMainWindow()
{
    publisherRoute_.shutdown();
    delete ui;
}

void GroundSystemMainWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    ros::shutdown();
}

void GroundSystemMainWindow::clientLog(CommandLog log)
{
    switch(log.level)
    {
    case CommandLogLevel::INFO:
        /* DO NOTHING */
        break;
    case CommandLogLevel::WARN:
    case CommandLogLevel::ALERT:
        // 警告ログはイベントとして通知する.
        statusWidgetEvent(log);
        break;
    }
}

void GroundSystemMainWindow::switchToMainPage()
{
    LOG_INFO() << "Switching screen to the MainPage";
    ui->mainPageWidget->setVideoArea(videoAreaWidget_);
    statusAreaWidget_->setOrientation(Qt::Horizontal);
    ui->mainPageWidget->setStatusArea(statusAreaWidget_);
    ui->stackedWidget->setCurrentIndex(PAGE_MAIN);
}

void GroundSystemMainWindow::switchToEditPage()
{
    LOG_INFO() << "Switching screen to the EditPage";

    // 編集前状態を保存する
    beforeEditRoutePointList_ = routeInformation_->routeWithoutStartPointAsList();

    statusAreaWidget_->setOrientation(Qt::Vertical);
    ui->editPageWidget->setVideoArea(videoAreaWidget_);
    ui->editPageWidget->setStatusArea(statusAreaWidget_);
    ui->editPageWidget->setCamera();
    ui->stackedWidget->setCurrentIndex(PAGE_EDIT);
}

void GroundSystemMainWindow::switchToExecutionPage()
{
    LOG_INFO() << "Switching screen to the ExecutionPage";

    //Int-Ball2現在位置を保持する.
    intballPositionBeforeMoving_ = routeInformation_->currentIntBallPose().position();

    ui->execPageWidget->setVideoArea(videoAreaWidget_);
    statusAreaWidget_->setOrientation(Qt::Vertical);
    ui->execPageWidget->setStatusArea(statusAreaWidget_);
    ui->execPageWidget->start();
    ui->stackedWidget->setCurrentIndex(PAGE_EXECUTION);
}

void GroundSystemMainWindow::on_editGoalButton_clicked()
{
    switchToEditPage();
}

void GroundSystemMainWindow::on_transferGoButton_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_GO)) {
        switchToExecutionPage();
    }
}

void GroundSystemMainWindow::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    // 有効な座標値の場合はPublish.
    auto quaternion = intballTelemetry_->data<geometry_msgs::Quaternion>(Index::NAVIGATION_POSE_ORIENTATION);
    if(quaternion.w != 0.0)
    {
        // rviz表示用のtfをPublish.
        tf::StampedTransform transform;
        transform.frame_id_ = rosframe::ISS_BODY;
        transform.child_frame_id_ = rosframe::BODY;
        transform.stamp_ = ros::Time::now();
        transform.setOrigin(geometryToTf(intballTelemetry_->data<geometry_msgs::Point>(Index::NAVIGATION_POSE_POSITION)));
        transform.setRotation(geometryToTf(intballTelemetry_->data<geometry_msgs::Quaternion>(Index::NAVIGATION_POSE_ORIENTATION)));
        tfBroadcaster_.sendTransform(transform);

        // Int-Ball2現在位置姿勢の更新.
        routeInformation_->setData(0, geometryToQt(intballTelemetry_->data<geometry_msgs::Point>(Index::NAVIGATION_POSE_POSITION)),
                                   geometryToQt(intballTelemetry_->data<geometry_msgs::Quaternion>(Index::NAVIGATION_POSE_ORIENTATION)));
    }
}

void GroundSystemMainWindow::TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value)
{
    switch(event)
    {
    case TelemetryMonitor::Event::CAPTURED:
    case TelemetryMonitor::Event::RELEASE_CAPTURE:
    case TelemetryMonitor::Event::START_SCAN:
    case TelemetryMonitor::Event::MODE_IDLING:
    case TelemetryMonitor::Event::MODE_OFF_NOMINAL:
    case TelemetryMonitor::Event::STOPPED_BY_COLLISION:
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_WARNING:
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_CRITICAL:
    case TelemetryMonitor::Event::TIMEOUT_TELEMETRY_INTBALL:
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_WARNING:
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_CRITICAL:
    case TelemetryMonitor::Event::BATTERY_LOW:
    case TelemetryMonitor::Event::BATTERY_LOW_CRITICAL:
    case TelemetryMonitor::Event::BATTERY_OUT_OF_RANGE:
    case TelemetryMonitor::Event::FLIGHT_SOFTWARE_OFF:
        /*
         * テレメトリの受信タイムアウトや移動が中断されるイベントを検知した場合は,
         * 移動中画面に対してエラーを通知する.
         * 誘導制御のキャンセル（ABORT）に関しては、PAUSEボタン押下時の停止で発生するため、エラーとはしない。
         */
        switch(ui->stackedWidget->currentIndex())
        {
        case PAGE_MAIN:
            break;
        case PAGE_EDIT:
            break;
        case PAGE_EXECUTION:
            ui->execPageWidget->anomalyDetected();
            break;
        default:
            Q_ASSERT(false);
            break;
        }
        break;
    case TelemetryMonitor::Event::CAMERA_ON:
    case TelemetryMonitor::Event::CAMERA_OFF:
    case TelemetryMonitor::Event::MICROPHONE_ON:
    case TelemetryMonitor::Event::MICROPHONE_OFF:
    case TelemetryMonitor::Event::STREAMING_ON:
    case TelemetryMonitor::Event::STREAMING_OFF:
    case TelemetryMonitor::Event::RECORDING_ON:
    case TelemetryMonitor::Event::RECORDING_OFF:
    case TelemetryMonitor::Event::CAMERA_MIC_UPDATE_PARAMETER_SUCCESS:
        // カメラ関連の操作が実行された後は,VLC（ビデオプレイヤー）の再起動を行う.
        videoController_->restart();
        break;
    }
}

void GroundSystemMainWindow::switchPageEvent(SwitchPageEvent event)
{
    INFO_START_FUNCTION();
    switch(event)
    {
    case SwitchPageEvent::DONE:
        switchToMainPage();
        if(sender() == ui->editPageWidget)
        {
            if(routeInformation_->count() > 1 && DialogFactory::executeConfirm("Do you want to save the route?"))
            {
                QFileDialog saveDialog(this,
                                       "Save the route",
                                       Config::valueAsString(KEY_EDITED_ROUTE_SAVE_DIRECTORY));
                saveDialog.setAcceptMode(QFileDialog::AcceptSave);
                saveDialog.setNameFilter("INI file(*.ini)");
                saveDialog.setDefaultSuffix("ini");
                // OSのNativeDialogを利用すると
                // 別ウィジットのアニメーション処理がダイアログの呼び出しをブロックしてしまい、固まる。
                // QFileDialog::DontUseNativeDialogを指定して、NativeDialogの利用は避ける。
                saveDialog.setOptions(QFileDialog::DontUseCustomDirectoryIcons | QFileDialog::DontUseNativeDialog);

                if(saveDialog.exec())
                {
                    QStringList fileNames = saveDialog.selectedFiles();
                    QSettings routeSettings(fileNames[0], QSettings::IniFormat);
                    routeSettings.setValue(KEY_ROUTE_LIST, QVariant::fromValue(routeInformation_->routeWithoutStartPointAsList()));
                    routeSettings.sync();
                }
            }
        }
        break;
    case SwitchPageEvent::CANCEL:
        switchToMainPage();
        if(sender() == ui->editPageWidget)
        {
            // 編集キャンセルの場合、経路を元に戻す.
            // Int-Ball2現在位置（beforeEditRoutePointList_に含まれない）は上書きしない.
            routeInformation_->removeRows(1, routeInformation_->rowCount() - 1);
            for(auto i = beforeEditRoutePointList_.begin(); i != beforeEditRoutePointList_.end(); ++i)
            {
                routeInformation_->pushBack(*i);
            }
        }
        break;
    case SwitchPageEvent::ERROR:
        switchToMainPage();
        break;

    }
}

void GroundSystemMainWindow::RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);
    publishRouteMarkerArray();
}
void GroundSystemMainWindow::RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    Q_UNUSED(last);
    publishRouteMarkerArray();
}
void GroundSystemMainWindow::RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    publishRouteMarkerArray(first, last);
}

void GroundSystemMainWindow::publishRouteMarkerArray(const int firstToBeRemoved, const int lastToBeRemoved)
{
    QVector<int> targetIndex;
    for(int i = 0; i < routeInformation_->rowCount(); ++i)
    {
        if((i >= firstToBeRemoved) && (i <= lastToBeRemoved))
        {
            continue;
        }
        targetIndex << i;
    }

    if(targetIndex.size() > 1)
    {
        // 経路の点と線を配置する.
        markerArrayView_.markers[MARKER_INDEX_LINE].points.resize(static_cast<unsigned long>(targetIndex.size()));

        int point_index_offset = -1;
        if(!(ui->stackedWidget->currentIndex() == PAGE_EXECUTION)) {
            /*
             * 移動中画面以外.
             */

            // 移動中でない場合,経路の点に関しては,Int-Ball2現在位置は表示しない（Int-Ball2本体のロボットモデル表示と重複するため）
            markerArrayView_.markers[MARKER_INDEX_POINT].points.resize(static_cast<unsigned long>(targetIndex.size() - 1));

            // 移動中画面以外を表示中の場合,経路線の先頭（開始位置）にInt-Ball2現在位置を表示する.
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].x = static_cast<double>(routeInformation_->data(targetIndex.at(0)).position().x());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].y = static_cast<double>(routeInformation_->data(targetIndex.at(0)).position().y());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].z = static_cast<double>(routeInformation_->data(targetIndex.at(0)).position().z());
        } else {
            /*
             * 移動中画面.
             */

            // 移動中画面の場合,Int-Ball2初期位置を含めて表示する.
            markerArrayView_.markers[MARKER_INDEX_POINT].points.resize(static_cast<unsigned long>(targetIndex.size()));
            markerArrayView_.markers[MARKER_INDEX_POINT].points[0].x = intballPositionBeforeMoving_.x();
            markerArrayView_.markers[MARKER_INDEX_POINT].points[0].y = intballPositionBeforeMoving_.y();
            markerArrayView_.markers[MARKER_INDEX_POINT].points[0].z = intballPositionBeforeMoving_.z();


            // 移動中画面を表示中は,経路線の先頭（開始位置）はInt-Ball2現在位置ではなく
            // 移動開始直前のInt-Ball2位置を固定表示する.
            point_index_offset = 0;
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].x = static_cast<double>(intballPositionBeforeMoving_.x());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].y = static_cast<double>(intballPositionBeforeMoving_.y());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[0].z = static_cast<double>(intballPositionBeforeMoving_.z());
        }

        for(int i = 1; i < targetIndex.size(); ++i)
        {
            markerArrayView_.markers[MARKER_INDEX_LINE].points[i].x = static_cast<double>(routeInformation_->data(targetIndex.at(i)).position().x());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[i].y = static_cast<double>(routeInformation_->data(targetIndex.at(i)).position().y());
            markerArrayView_.markers[MARKER_INDEX_LINE].points[i].z = static_cast<double>(routeInformation_->data(targetIndex.at(i)).position().z());
            markerArrayView_.markers[MARKER_INDEX_POINT].points[i + point_index_offset].x = routeInformation_->data(targetIndex.at(i)).position().x();
            markerArrayView_.markers[MARKER_INDEX_POINT].points[i + point_index_offset].y = routeInformation_->data(targetIndex.at(i)).position().y();
            markerArrayView_.markers[MARKER_INDEX_POINT].points[i + point_index_offset].z = routeInformation_->data(targetIndex.at(i)).position().z();

        }
        publisherRoute_.publish(markerArrayView_);
    }
    else
    {
        // 経路の点と線を削除する.
        publisherRoute_.publish(markerArrayDelete_);
    }
}
