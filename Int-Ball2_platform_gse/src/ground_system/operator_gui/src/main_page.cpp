#include "main_page.h"
#include "ui_main_page.h"
#include <string>
#include <vector>
#include <QDebug>
#include <QFileDialog>
#include <QSettings>
#include <QString>
#include <QTextCodec>
#include <QUrl>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/yaml_config_reader.h>
#include <std_msgs/Time.h>
#include <tf/transform_listener.h>
#include "amount_slider_widget.h"
#include "dialog_factory.h"
#include "editing_page.h"
#include "gui_color.h"
#include "gui_common.h"
#include "operator_gui_config.h"
#include "ib2_msgs.h"
#include "model/intball_telemetry.h"
#include "model/dock_telemetry.h"
#include "model/route_information.h"
#include "communication_software/Telemetry.h"
#include "qdebug_custom.h"
#include "ros_common.h"
#include "route_setting.h"
#include "video_controller.h"
#include "utils.h"
#include "yaml-cpp/yaml.h"
#include "telemetry_telecommand_config.h"
#include "telecommand_client.h"
#include "telemetry_subscriber.h"


using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;
using namespace intball::telemetry;
using namespace intball::message;

MainPage::MainPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MainPage),
    controlStatus_(CONTROL_STATUS::NONE),
    isEnableMarkerCorrectionOnDock_(false),
    isOffNominalBattery_(false),
    isOffNominalDiskSpace_(false),
    isOffNominalTemperature_(false),
    isDockSwitchDocking_(false)
{
    qRegisterMetaType<RoutePoint>("RoutePoint");
    qRegisterMetaType<QList<RoutePoint>>("QList<RoutePoint>");
    qRegisterMetaTypeStreamOperators<RoutePoint>("RoutePoint");
    qRegisterMetaTypeStreamOperators<QList<RoutePoint>>("QList<RoutePoint>");

    ui->setupUi(this);

    /*
     * Control
     */
    ui->controlGoButton->setEnabled(false);

    // スライダー.
    ui->controlAmountPositionSlider->setUnitLabel("[m]");
    ui->controlAmountPositionSlider->setSliderValueScale(100.0);
    ui->controlAmountPositionSlider->slider().setSingleStep(10);
    ui->controlAmountPositionSlider->slider().setPageStep(10);
    ui->controlAmountPositionSlider->slider().setMinimum(1);
    ui->controlAmountPositionSlider->slider().setMaximum(50);

    ui->controlAmountAttitudeSlider->setUnitLabel("[deg]");
    ui->controlAmountAttitudeSlider->slider().setSingleStep(10);
    ui->controlAmountAttitudeSlider->slider().setPageStep(10);
    ui->controlAmountAttitudeSlider->slider().setMinimum(5);
    ui->controlAmountAttitudeSlider->slider().setMaximum(180);

    // RouteEditor.
    ui->transferSideView->setDetectMouseEvent(false);
    ui->transferTopDownView->setAxisType(RouteEditor::AxisType::X_Y);
    ui->transferTopDownView->setDetectMouseEvent(false);

    // 特殊コマンドの選択初期設定.
    ui->controlAdditionalCommandComboBox->setCurrentIndex(0);

    isEnableMarkerCorrectionOnDock_ = Config::valueOnOffAsBool(KEY_ENABLE_MARKER_CORRECTION_ON_DOCK);
}

MainPage::~MainPage()
{
    delete ui;
}

void MainPage::initialize(const QString& pathRvizConfig,
                          RouteInformation* routeInformation,
                          QItemSelectionModel* routeInformationSelectionModel,
                          VideoController* videoController,
                          IntBallTelemetry* intballTelemetry,
                          TelecommandClient* telecommandClient,
                          DockTelemetry* dockTelemetry
                          )
{
    // rvizの座標系をQtの座標系に変換する設定.
    QTransform transform;
    // NOTE: 現状設定パラメータ化は無し
    // Y軸を反転する.
    ui->transferSideView->initialize(transform);
    ui->transferTopDownView->initialize(transform);
    ui->transferBirdEyeView->initialize(pathRvizConfig);

    // 経路のデータモデル登録.
    routeInformation_ = routeInformation;
    ui->transferSideView->setModel(routeInformation_);
    ui->transferTopDownView->setModel(routeInformation_);

    /*
     * QtのViewクラスでは,setModelが呼ばれた際に内部でSelectionModelが自動生成されるため
     * 必ずsetModelを呼び出した後にsetSelectionModelを呼び出す必要がある.
     */
    routeInformationSelectionModel_ = routeInformationSelectionModel;
    ui->transferSideView->setSelectionModel(routeInformationSelectionModel_);
    ui->transferTopDownView->setSelectionModel(routeInformationSelectionModel_);

    ui->transferSideView->setModel(routeInformation_);
    ui->transferTopDownView->setModel(routeInformation_);
    connect(routeInformation_, &RouteInformation::rowsInserted,
            this, &MainPage::RouteInformation_rowsInserted);
    connect(routeInformation_, &RouteInformation::rowsAboutToBeRemoved,
            this, &MainPage::RouteInformation_rowsAboutToBeRemoved);
    connect(routeInformation_, &RouteInformation::dataChanged,
            this, &MainPage::RouteInformation_dataChanged);
    connect(routeInformationSelectionModel_, &QItemSelectionModel::selectionChanged,
            this, &MainPage::RouteInformation_selectionChanged);

    videoController_ = videoController;

    // テレメトリとの接続.
    intballTelemetry_ = intballTelemetry;
    connect(intballTelemetry_, &IntBallTelemetry::rowsChanged,
            this, &MainPage::IntBall2Telemetry_rowsChanged);

    dockTelemetry_ = dockTelemetry;
    connect(dockTelemetry_, &QAbstractListModel::dataChanged,
            this, &MainPage::DockTelemetry_dataChanged);

    telecommandClient_ = telecommandClient;

    // 初期表示時.
    switchingControlOperations();

    // その他下位Widgetの初期化.
    ui->crewSupportWidget->initialize(intballTelemetry_, telecommandClient_);

    // 設定ダイアログでテレメトリを表示.
    DialogFactory::getCameraMicrophoneSettingsDialog().initialize(intballTelemetry_, telecommandClient_);
    DialogFactory::getLedSettingsDialog().initialize(intballTelemetry_, telecommandClient_);
}

void MainPage::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    ros::shutdown();
}

void MainPage::setVideoArea(QWidget* video)
{
    video->setParent(ui->videoArea);
    video->setFixedSize(ui->videoArea->size());
    video->show();
}

void MainPage::setStatusArea(QWidget* status)
{
    ui->statusGroup->layout()->addWidget(status);
}

void MainPage::IntBall2Telemetry_rowsChanged(QList<int> rowList)
{
    if(rowList.contains(static_cast<int>(telemetry::Index::MODE))){
        // Int-Ball2の動作モードによって表示を切り替える可能性があるため,呼び出し.
        switchingControlOperations();
    }
}

void MainPage::DockTelemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    isDockSwitchDocking_ = (dockTelemetry_->data(dock::telemetry::Index::SWITCH_STATE) == dock::telemetry::SWITCH_STATE_TYPE::DONE_DOCKING);
}

void MainPage::TelemetryMonitor_modeTrackingStarted()
{
    INFO_START_FUNCTION();

    if(intballTelemetry_->getMode() == ib2_msgs::Mode::STANDBY &&
            controlStatus_ != CONTROL_STATUS::NONE)
    {
        LOG_INFO() << "The GUI is in control display even though Int-Ball2 is in standby mode. Cancels the display under control.";
        controlStatus_ = CONTROL_STATUS::NONE;
        switchingControlOperations();
    }
}

void MainPage::TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value)
{
    Q_UNUSED(value);
    INFO_START_FUNCTION();

    if(!this->isVisible()) {
        // 非表示時は処理しない.
        return;
    }

    switch(event)
    {
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_WARNING:
    case TelemetryMonitor::Event::TEMPERATURE_IS_OVER_CRITICAL:
        isOffNominalTemperature_ = true;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::BATTERY_LOW:
    case TelemetryMonitor::Event::BATTERY_LOW_CRITICAL:
    case TelemetryMonitor::Event::BATTERY_OUT_OF_RANGE:
        isOffNominalBattery_ = true;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_WARNING:
    case TelemetryMonitor::Event::STORAGE_USAGE_IS_OVER_CRITICAL:
        isOffNominalDiskSpace_ = true;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::TEMPERATURE_COME_UNDER_WARNING:
        isOffNominalTemperature_ = false;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::BATTERY_NORMAL:
    case TelemetryMonitor::Event::BATTERY_FULLY_CHARGED:
        isOffNominalBattery_ = false;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::STORAGE_USAGE_COME_UNDER_WARNING:
        isOffNominalDiskSpace_ = false;
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::CTL_STANDBY:
    case TelemetryMonitor::Event::CTL_ABNORMAL_SHUTDOWN:
    case TelemetryMonitor::Event::KEEP_POSE:
    case TelemetryMonitor::Event::STOPPED_BY_COLLISION:
        if(isMovableModeAndCtlStatus())
        {
            /*
             * 機体モード・誘導制御ステータスが次のコマンドを受付待ちになったにもかかわらず
             * GUI側が移動中扱いから抜け出していない場合、GUIを再度移動制御可能とする.
             * オフノミナルなどを想定.
             */
            controlStatus_ = CONTROL_STATUS::NONE;
            switchingControlOperations();
        }
        break;
    case TelemetryMonitor::Event::START_CTL_COMMAND:
        if(controlStatus_ == CONTROL_STATUS::NONE)
        {
            /*
             * 移動が開始された場合を検知する.
             * GUIで制御コマンドを送信する前に移動が開始されたケースや,
             * リリースコマンドが正常に受け付けられたケースを想定.
             */
            if(intballTelemetry_->isGuidanceControlRunning())
            {
                LOG_INFO() << "MainPage switches CONTROL_STATUS::MOVE";
                controlStatus_ = CONTROL_STATUS::MOVE;

                // 画面表示の切り替え.
                switchingControlOperations();
            }
        }
        break;
    case TelemetryMonitor::Event::FINISH_CTL_COMMAND:
        if(controlStatus_ != CONTROL_STATUS::NONE)
        {
            auto result = intballTelemetry_->data(telemetry::Index::CTL_ACTION_RESULT_TYPE);
            LOG_INFO() << "CtlCommand finished: type=" << getCtlCommandResultAsString(result);

            // 制御処理を完了.
            controlStatus_ = CONTROL_STATUS::NONE;

            // 制御系のコマンドを送信可能とする.
            switchingControlOperations();

        }
        else
        {
            LOG_WARNING() << "The CtlCommand result was received before sending the CtlCommand.";
        }
        break;
    case TelemetryMonitor::Event::MODE_CHANGED:
        if(isMovableModeAndCtlStatus())
        {
            /*
             * 機体モード・誘導制御ステータスが次のコマンドを受付待ちになったにもかかわらず
             * GUI側が移動中扱いから抜け出していない場合、GUIを再度移動制御可能とする.
             * オフノミナルなどを想定.
             */
            controlStatus_ = CONTROL_STATUS::NONE;
        }

        // リリースボタンなど画面操作状況（CONTROL_STATUS）を見ずに
        // テレメトリの動作モード値や機体ソフトウェアの動作状況で実行可否を判定するボタンがあるため
        // 念の為ここで制御系のコマンドの実行可否を切り替える.
        switchingControlOperations();
        break;
    case TelemetryMonitor::Event::FLIGHT_SOFTWARE_ON:
    case TelemetryMonitor::Event::FLIGHT_SOFTWARE_OFF:
        // リリースボタンなど画面操作状況（CONTROL_STATUS）を見ずに
        // テレメトリの動作モード値や機体ソフトウェアの動作状況で実行可否を判定するボタンがあるため
        // 念の為ここで制御系のコマンドの実行可否を切り替える.
        switchingControlOperations();
        break;
    };
}

bool MainPage::isMovableModeAndCtlStatus()
{
    return (intballTelemetry_->getMode() == ib2_msgs::Mode::OPERATION ||
            intballTelemetry_->getMode() == ib2_msgs::Mode::STANDBY ||
            intballTelemetry_->getMode() == ib2_msgs::Mode::MAINTENANCE ||
            intballTelemetry_->getMode() == ib2_msgs::Mode::OFF_NOMINAL) &&
                (intballTelemetry_->getCtlStatus() == ib2_msgs::CtlStatusType::STAND_BY ||
                 intballTelemetry_->getCtlStatus() == ib2_msgs::CtlStatusType::KEEP_POSE ||
                 intballTelemetry_->getCtlStatus() == ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION);
}

bool MainPage::isReleaseEnabled()
{
    // 機体側でオフノミナルを検知していた場合,リリースコマンドが破棄されるため
    // リリースの実行可否判断は画面側の操作状況（CONTROL_STATUS）は見ずに
    // テレメトリ値のみを基準に行う
    return (intballTelemetry_->getMode() == ib2_msgs::Mode::STANDBY) &&
            (intballTelemetry_->isFlightSoftwareStarted() &&
            !isOffNominalBattery_ &&
            !isOffNominalDiskSpace_ &&
            !isOffNominalTemperature_);
}

bool MainPage::isDockingEnabled()
{
    // ドッキングはib2_msgs::Mode::STANDBY時は実行不可.
    return (controlStatus_ == CONTROL_STATUS::NONE) &&
            intballTelemetry_->isFlightSoftwareStarted() &&
            isMovableModeAndCtlStatus() &&
            intballTelemetry_->getMode() != ib2_msgs::Mode::STANDBY;
}

bool MainPage::isNormalMoveCommandEnabled()
{
    return (controlStatus_ == CONTROL_STATUS::NONE) &&
            intballTelemetry_->isFlightSoftwareStarted() &&
            isMovableModeAndCtlStatus();
}

void MainPage::switchingAdditionalCommands()
{
    switch(ui->controlAdditionalCommandComboBox->currentIndex())
    {
    case 2:
    case 3:
        //ExitDockingModeは、ドッキング中であれば送信可能とする.
        ui->executeAdditionalCommandButton->setEnabled(intballTelemetry_->getMode() == ib2_msgs::Mode::DOCKING);
        ui->executeAdditionalCommandButton->setEnabled(intballTelemetry_->getMode() == ib2_msgs::Mode::DOCKING);
        break;
    case 4:
        //Forced releaseが選択された場合、実行可否はリリースボタンと同じ.
        ui->executeAdditionalCommandButton->setEnabled(isReleaseEnabled());
        break;
    case 5: //Reboot
        // Rebootコマンドは常に有効.
        ui->executeAdditionalCommandButton->setEnabled(true);
        break;
    default:
        ui->executeAdditionalCommandButton->setEnabled(isNormalMoveCommandEnabled());

    }
}

void MainPage::switchingControlOperations()
{
    INFO_START_FUNCTION() << "isNormalMoveCommandEnabled=" << isNormalMoveCommandEnabled() <<
                             " isReleaseEnabled=" << isReleaseEnabled() <<
                             " isDockingEnabled=" << isDockingEnabled();

    bool normalMoveEnabled = isNormalMoveCommandEnabled();

    ui->releaseButton->setEnabled(isReleaseEnabled());
    ui->dockingButton->setEnabled(isDockingEnabled());
    switchingAdditionalCommands();

    // 通常の移動コマンド.
    ui->controlPositionUp->setEnabled(normalMoveEnabled);
    ui->controlPositionRight->setEnabled(normalMoveEnabled);
    ui->controlPositionDown->setEnabled(normalMoveEnabled);
    ui->controlPositionLeft->setEnabled(normalMoveEnabled);
    ui->controlPositionFront->setEnabled(normalMoveEnabled);
    ui->controlPositionBack->setEnabled(normalMoveEnabled);
    ui->controlAttitudeYawClock->setEnabled(normalMoveEnabled);
    ui->controlAttitudePitchClock->setEnabled(normalMoveEnabled);
    ui->controlAttitudeRollClock->setEnabled(normalMoveEnabled);
    ui->controlAttitudeYawCounterClock->setEnabled(normalMoveEnabled);
    ui->controlAttitudePitchCounterClock->setEnabled(normalMoveEnabled);
    ui->controlAttitudeRollCounterClock->setEnabled(normalMoveEnabled);
    ui->loadRouteButton->setEnabled(normalMoveEnabled);
    ui->editGoalButton->setEnabled(normalMoveEnabled);

    // 絶対座標指定の移動.
     ui->controlGoButton->setEnabled(normalMoveEnabled);

    // 設定した経路での移動実行可否.
    ui->transferGoButton->setEnabled((routeInformation_ != nullptr && routeInformation_->rowCount() > 1) && normalMoveEnabled);
}

void MainPage::updateRoute(const int lastToBeRemoved)
{
    // 現在位置.
    RoutePoint current = routeInformation_->currentIntBallPose();
    ui->transferXCurrent->setText(QString::number(roundPositionValue(current.position().x())));
    ui->transferYCurrent->setText(QString::number(roundPositionValue(current.position().y())));
    ui->transferZCurrent->setText(QString::number(roundPositionValue(current.position().z())));

    // オイラー角の順序はros(tf)基準.
    qreal roll, pitch, yaw;
    getRPY(current.orientation(), roll, pitch, yaw);
    ui->transferRollCurrent->setText(QString::number(roundDegree(roll)));
    ui->transferPitchCurrent->setText(QString::number(roundDegree(pitch)));
    ui->transferYawCurrent->setText(QString::number(roundDegree(yaw)));

    // ゴール位置.
    if((routeInformation_->rowCount() > 1) && (!routeInformation_->lastOrLater(lastToBeRemoved)))
    {
        RoutePoint goal = routeInformation_->goal();
        ui->transferXGoal->setText(QString::number(roundPositionValue(goal.position().x())));
        ui->transferYGoal->setText(QString::number(roundPositionValue(goal.position().y())));
        ui->transferZGoal->setText(QString::number(roundPositionValue(goal.position().z())));

        // オイラー角の順序はros(tf)基準.
        getRPY(goal.orientation(), roll, pitch, yaw);
        ui->transferRollGoal->setText(QString::number(roundDegree(roll)));
        ui->transferPitchGoal->setText(QString::number(roundDegree(pitch)));
        ui->transferYawGoal->setText(QString::number(roundDegree(yaw)));

        // 移動を実行可能.
        ui->transferGoButton->setEnabled(isNormalMoveCommandEnabled());
    }
    else
    {
        ui->transferXGoal->setText(" - ");
        ui->transferYGoal->setText(" - ");
        ui->transferZGoal->setText(" - ");
        ui->transferRollGoal->setText(" - ");
        ui->transferPitchGoal->setText(" - ");
        ui->transferYawGoal->setText(" - ");

        // 移動は実行不可.
        ui->transferGoButton->setDisabled(true);
    }
}

void MainPage::on_videoRotateSlider_valueChanged(int value)
{
    videoController_->setRotate(value);
}

void MainPage::on_loadRouteButton_clicked()
{
    INFO_START_FUNCTION();
    // OSのNativeDialogを利用すると
    // 別ウィジットのアニメーション処理がダイアログの呼び出しをブロックしてしまい、固まる。
    // QFileDialog::DontUseNativeDialogを指定して、NativeDialogの利用は避ける。
    QUrl path = QFileDialog::getOpenFileName(
                this,
                "Load the route definition file",
                Config::valueAsString(KEY_EDITED_ROUTE_SAVE_DIRECTORY),
                "INI file(*.ini);;*(*.*)",
                nullptr,
                QFileDialog::DontUseCustomDirectoryIcons | QFileDialog::DontUseNativeDialog);
    if(!path.isEmpty())
    {
        LOG_INFO() << "Select a route settings file: " << path.path();

        QFileInfo selectedFileInfo(path.path());
        if(!selectedFileInfo.isReadable())
        {
            LOG_WARNING() << "Permission denied: " << path.path();
        }
        else
        {
            QSettings routeConfig(path.path(), QSettings::IniFormat);
            QList<RoutePoint> routePointList = routeConfig.value(KEY_ROUTE_LIST).value<QList<RoutePoint>>();
            if(routePointList.empty())
            {
                LOG_WARNING() << "The route settings file is empty or invalid format: " << path.path();
            }
            else
            {
                routeInformation_->removeRows(1, routeInformation_->rowCount() - 1);
                for(auto i = routePointList.begin(); i != routePointList.end(); ++i)
                {
                    routeInformation_->pushBack(*i);
                }
            }
        }
    }
}

void MainPage::on_videoRotateResetButton_clicked()
{
    INFO_START_FUNCTION();
    ui->videoRotateSlider->setValue(0);
    videoController_->setRotate(0);
}

void MainPage::RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);
    updateRoute();
}

void MainPage::RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    Q_UNUSED(last);
    updateRoute();
}

void MainPage::RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    updateRoute(last);

}

void MainPage::RouteInformation_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    Q_UNUSED(selected);
    Q_UNUSED(deselected);
}

void MainPage::checkControlTelecommandResultCheck(const bool result, const CONTROL_STATUS &status)
{
    if(result)
    {
        // 制御処理中.
        controlStatus_ = status;

        // 誘導制御関連のUIをOFF.
        switchingControlOperations();
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
    }
}

void MainPage::on_controlGoButton_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(ui->controlInputX->text().toFloat(),
                           ui->controlInputY->text().toFloat(),
                           ui->controlInputZ->text().toFloat());
        QQuaternion orientation = fromRPYDegree(ui->controlInputRoll->text().toDouble(),
                                          ui->controlInputPitch->text().toDouble(),
                                          ui->controlInputYaw->text().toDouble());
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalAbsolute(position, orientation), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlReadButton_clicked()
{
    auto currentPose = routeInformation_->currentIntBallPose();
    ui->controlInputX->setValue(static_cast<double>(currentPose.position().x()));
    ui->controlInputY->setValue(static_cast<double>(currentPose.position().y()));
    ui->controlInputZ->setValue(static_cast<double>(currentPose.position().z()));

    qreal roll, pitch, yaw;
    getRPY(currentPose.orientation(), roll, pitch, yaw);
    ui->controlInputRoll->setValue(static_cast<int>(roundDegree(roll)));
    ui->controlInputPitch->setValue(static_cast<int>(roundDegree(pitch)));
    ui->controlInputYaw->setValue(static_cast<int>(roundDegree(yaw)));
}

void MainPage::on_dockingButton_clicked()
{
    if(!isDockSwitchDocking_)
    {
        // ドッキングステーションがドッキング可能なスイッチ状態でない場合、ドッキングを実行不可
        DialogFactory::alert("Cannot execute docking because docking station switch is not \"DONE_DOCKING\".");
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_DOCKING))
    {
        if(isEnableMarkerCorrectionOnDock_){
            checkControlTelecommandResultCheck(telecommandClient_->sendDockingWithMarkerCorrection(), CONTROL_STATUS::MOVE);
        }else{
            checkControlTelecommandResultCheck(telecommandClient_->sendDockingWithoutMarkerCorrection(), CONTROL_STATUS::MOVE);
        }
    }
}

void MainPage::on_releaseButton_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_RELEASE))
    {
        // 画面としては制御処理中に遷移させるが,
        // リリースボタンの実行可否は画面が制御処理中かは関係なく
        // テレメトリ内容のみに基づいて実行可否を判断する.
        // ※機体側でオフノミナル状態だった場合リリースコマンドが破棄されるため
        //   テレメトリ内容でしかリリースの実行可否を判断できない
        if(!telecommandClient_->sendRelease())
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void MainPage::on_emergencyButton_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_EMERGENCY_STOP))
    {
        // EmergencyStopの後は誘導制御関連のコマンドを無効とはせず
        // Int-Ball2状況に応じて追加の動作を可能とする
        if(!telecommandClient_->sendCtlCommandCancel())
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void MainPage::on_controlPositionUp_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(0, 0, ui->controlAmountPositionSlider->value());
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlPositionRight_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(0, -ui->controlAmountPositionSlider->value(), 0);
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlPositionDown_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(0, 0, -ui->controlAmountPositionSlider->value());
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlPositionLeft_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(0, ui->controlAmountPositionSlider->value(), 0);
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlPositionFront_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(ui->controlAmountPositionSlider->value(), 0, 0);
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlPositionBack_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        QVector3D position(-ui->controlAmountPositionSlider->value(), 0, 0);
        checkControlTelecommandResultCheck(telecommandClient_->sendTargetGoalRelative(position, QQuaternion()), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlAttitudeRollCounterClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()), 0, 0)), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlAttitudeRollClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(static_cast<qreal>(ui->controlAmountAttitudeSlider->value()), 0, 0)), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlAttitudePitchCounterClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(0, static_cast<qreal>(ui->controlAmountAttitudeSlider->value()), 0)), CONTROL_STATUS::MOVE);
    }
}

void MainPage::on_controlAttitudePitchClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(0, static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()), 0)), CONTROL_STATUS::MOVE);
    }

}

void MainPage::on_controlAttitudeYawCounterClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(0, 0, static_cast<qreal>(ui->controlAmountAttitudeSlider->value()))), CONTROL_STATUS::MOVE);
    }

}

void MainPage::on_controlAttitudeYawClock_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2))
    {
        checkControlTelecommandResultCheck(
                    telecommandClient_->sendTargetGoalRelative(
                        QVector3D(), fromRPYDegree(0, 0, static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()))), CONTROL_STATUS::MOVE);
    }

}

void MainPage::on_takeSnapshotButton_clicked()
{
    videoController_->takeSnapshot();
}

void MainPage::on_executeAdditionalCommandButton_clicked()
{
    if(ui->controlAdditionalCommandComboBox->currentIndex() != 0)
    {
        switch(ui->controlAdditionalCommandComboBox->currentIndex())
        {
        case 1:
            // マーカー補正.
            if(DialogFactory::telecommandCheck(COMMAND_NAME_MARKER_CORRECTION)) {
                if(!telecommandClient_->sendMarkerCorrection())
                {
                    // 送信失敗.
                    DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
                }
            }
            break;
        case 2:
            // ドッキング動作完了（スタンバイモードへ遷移）.
            if(DialogFactory::telecommandCheck(COMMAND_NAME_EXIT_DOCKING_MODE_FINISH)) {
                if(!telecommandClient_->sendExitDockingModeFinish())
                {
                    // 送信失敗.
                    DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
                }
            }
            break;
        case 3:
            // ドッキング動作のキャンセル（運用モードへ遷移）.
            if(DialogFactory::telecommandCheck(COMMAND_NAME_EXIT_DOCKING_MODE_CANCEL)) {
                if(!telecommandClient_->sendExitDockingModeCancel())
                {
                    // 送信失敗.
                    DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
                }
            }
            break;
        case 4:
            // 強制リリース.
            if(DialogFactory::telecommandCheck(COMMAND_NAME_FORCED_RELEASE)) {
                if(!telecommandClient_->sendForcedRelease())
                {
                    // 送信失敗.
                    DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
                }
            }
            break;
        case 5:
            // 再起動.
            if(DialogFactory::telecommandCheck(COMMAND_NAME_REBOOT)) {
                if(!telecommandClient_->sendReboot())
                {
                    // 送信失敗.
                    DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
                }
            }
            break;
        }

        ui->controlAdditionalCommandComboBox->setCurrentIndex(0);
    }
}

void MainPage::on_controlAdditionalCommandComboBox_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    switchingAdditionalCommands();
}
