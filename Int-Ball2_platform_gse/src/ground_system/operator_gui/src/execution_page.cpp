#include "execution_page.h"
#include "ui_execution_page.h"
#include <QDialog>
#include <QIcon>
#include <QMessageBox>
#include "dialog_factory.h"
#include "operator_gui_config.h"
#include "model/intball_telemetry.h"
#include "model/route_information.h"
#include "qdebug_custom.h"
#include "route_movement_execution_status_list.h"
#include "execution_page_statemachine.h"
#include "telecommand_client.h"
#include "telemetry_monitor.h"
#include "utils.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

/*
 * scxmlに基づく状態遷移を行う.
 * 状態遷移図はscxmlフォルダを参照のこと.
 */

ExecutionPage::ExecutionPage(QWidget *parent) :
    QWidget(parent), ui(new Ui::ExecutionPage), targetRoutePointIndex_(ExecutionPage::DEFAULT_TARGET_ROUTE_POINT_INDEX),
    elapsedTimeSecond_(0), restOfWaitingTime_(0), waitingTimeForCurrentMove_(0), waitingTimeForCurrentStopping_(0)
{
    ui->setupUi(this);

    // 一時停止ボタン.
    ui->execPauseButton->setAdditionalState("START", QIcon(":/ground_system/image/play.svg"));

    ui->goalSideView->setDetectMouseEvent(false);
    ui->goalTopDownView->setAxisType(RouteEditor::AxisType::X_Y);
    ui->goalTopDownView->setDetectMouseEvent(false);

    // 状態遷移時の処理.
    stateMachine_.connectToState("InitialState",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryInitialState));
    stateMachine_.connectToState("NormalOperation",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryNormalOperation));
    stateMachine_.connectToState("NormalOperation.InPreparation",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryNormalOperation_InPreparation));
    stateMachine_.connectToState("NormalOperation.PostProcess",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryNormalOperation_PostProcess));
    stateMachine_.connectToState("NormalOperation.PostProcess",
                                 QScxmlStateMachine::onExit(this, &ExecutionPage::onExitNormalOperation_PostProcess));
    stateMachine_.connectToState("TargetGuidanceControl.Processing",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryTargetGuidanceControl_Processing));
    stateMachine_.connectToState("TargetGuidanceControl.Processing",
                                 QScxmlStateMachine::onExit(this, &ExecutionPage::onExitTargetGuidanceControl_Processing));
    stateMachine_.connectToState("TargetGuidanceControl.Waiting",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryTargetGuidanceControl_Waiting));
    stateMachine_.connectToState("TargetGuidanceControl.Waiting",
                                 QScxmlStateMachine::onExit(this, &ExecutionPage::onExitTargetGuidanceControl_Waiting));
    stateMachine_.connectToState("TargetGuidanceControl.Goal",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryTargetGuidanceControl_Goal));
    stateMachine_.connectToState("Pause.Waiting",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryPause_Waiting));
    stateMachine_.connectToState("Pause.Completed",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryPause_Completed));
    stateMachine_.connectToState("Cancel.Waiting",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryCancel_Waiting));
    stateMachine_.connectToState("Cancel.Completed",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryCancel_Completed));
    stateMachine_.connectToState("EmergencyStop",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryEmergencyStop));
    stateMachine_.connectToState("ErrorState",
                                 QScxmlStateMachine::onEntry(this, &ExecutionPage::onEntryErrorState));
    stateMachine_.connectToState("ErrorState",
                                 QScxmlStateMachine::onExit(this, &ExecutionPage::onExitErrorState));
    /*
     * タイマー.
     */
    elapsedTimer_.setInterval(1000);
    waitingTimer_.setInterval(1000);
    connect(&elapsedTimer_, &QTimer::timeout, this, &ExecutionPage::elapsedTimerHandler);
    connect(&waitingTimer_, &QTimer::timeout, this, &ExecutionPage::waitingTimerHandler);

    stateMachine_.start();
}

ExecutionPage::~ExecutionPage()
{
    delete ui;
}

void ExecutionPage::initialize(const QString& pathRvizConfig,
                               intball::RouteInformation* routeInformation,
                               intball::TelecommandClient* telecommandClient,
                               intball::IntBallTelemetry* intballTelemetry
                              )
{
    routeInformation_ = routeInformation;
    intballTelemetry_ = intballTelemetry;
    telecommandClient_ = telecommandClient;

    // rvizの座標系をQtの座標系に変換する設定.
    QTransform transform;
    // NOTE: 現状設定パラメータ化は無し
//    transform.translate(ui->goalSideView->width() / 2, ui->goalSideView->height() / 2);
    // Y軸を反転する.
//    transform.scale(5, -5);
    ui->goalSideView->initialize(transform);
    ui->goalTopDownView->initialize(transform);
    ui->goalBirdEyeView->initialize(pathRvizConfig);

    // 経路のデータモデル登録.
    ui->goalSideView->setModel(routeInformation_);
    ui->goalTopDownView->setModel(routeInformation_);

    // 初期化処理.
    connect(routeInformation_, &RouteInformation::rowsInserted,
            this, &ExecutionPage::RouteInformation_rowsInserted);
    connect(routeInformation_, &RouteInformation::rowsAboutToBeRemoved,
            this, &ExecutionPage::RouteInformation_rowsAboutToBeRemoved);
    connect(routeInformation_, &RouteInformation::dataChanged,
            this, &ExecutionPage::RouteInformation_dataChanged);
    connect(intballTelemetry_, &IntBallTelemetry::dataChanged,
            this, &ExecutionPage::IntBall2Telemetry_dataChanged);

    // その他下位Widgetの初期化.
    ui->crewSupportWidget->initialize(intballTelemetry_, telecommandClient_);
}

void ExecutionPage::setVideoArea(QWidget* video)
{
    video->setParent(ui->videoArea);
    video->setFixedSize(ui->videoArea->size());
    video->show();
}

void ExecutionPage::setStatusArea(QWidget* status)
{
    ui->statusGroup->layout()->addWidget(status);
}


void ExecutionPage::start()
{
    if(!isStarted())
    {
        // カメラを現在のInt-Ball2位置に向ける.
        ui->goalBirdEyeView->setFocalPoint(routeInformation_->currentIntBallPose().position());
        stateMachine_.submitEvent("NormalOperation.Start");
    }
    else
    {
        LOG_WARNING() <<  __PRETTY_FUNCTION__ << " : Already started.";
    }
}

void ExecutionPage::stopTimer()
{
    if(elapsedTimer_.isActive())
    {
        elapsedTimer_.stop();
    }
}

bool ExecutionPage::isStarted()
{
    return !stateMachine_.isActive("InitialState");
}

void ExecutionPage::anomalyDetected()
{
    stateMachine_.submitEvent("Error.Detected");
}

void ExecutionPage::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    if(isStarted())
    {
        // Int-Ball2現在位置の更新.
        // 現在位置.
        RoutePoint current = routeInformation_->currentIntBallPose();
        ui->labelXCurrent->setText(QString::number(roundPositionValue(current.position().x())));
        ui->labelYCurrent->setText(QString::number(roundPositionValue(current.position().y())));
        ui->labelZCurrent->setText(QString::number(roundPositionValue(current.position().z())));

        // オイラー角の順序はros(tf)基準.
        qreal roll, pitch, yaw;
        getRPY(current.orientation(), roll, pitch, yaw);
        ui->labelRollCurrent->setText(QString::number(roundDegree(roll)));
        ui->labelPitchCurrent->setText(QString::number(roundDegree(pitch)));
        ui->labelYawCurrent->setText(QString::number(roundDegree(yaw)));
    }
}

void ExecutionPage::TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value)
{
    if(!this->isVisible()) {
        // 非表示時は処理しない.
        return;
    }

    if(event == TelemetryMonitor::Event::FINISH_CTL_COMMAND)
    {
        auto result = value.value<unsigned char>();
        if(stateMachine_.isActive("TargetGuidanceControl.Processing"))
        {
            if(result == ib2_msgs::CtlCommandResult::Type::TERMINATE_SUCCESS)
            {
                // 目標点に到達.
                stateMachine_.submitEvent("TargetGuidanceControl.Arrived");
            }
            else
            {
                // その他エラー.
                stateMachine_.submitEvent("Error.Detected");
            }
        }

        if(stateMachine_.isActive("Pause.Waiting"))
        {
            // STOP_MOVINGの完了、またはその時点で実行されていたアクションのキャンセル
            stateMachine_.submitEvent("Pause.Stopped");

            // 再開可能とする（ボタンを有効化）
            ui->execPauseButton->setEnabled(true);
        }

        if(stateMachine_.isActive("Cancel.Waiting"))
        {
            // STOP_MOVINGの完了、またはその時点で実行されていたアクションのキャンセル
            stateMachine_.submitEvent("Cancel.Stopped");
        }
    }

    if(event == TelemetryMonitor::Event::START_CTL_COMMAND)
    {
        if(!stateMachine_.isActive("Cancel.Waiting") && !stateMachine_.isActive("EmergencyStop.Waiting"))
        {
            // キャンセル/停止以外の場合は一時停止を可能とする（ボタンを有効化）
            ui->execPauseButton->setEnabled(true);
        }
    }
}

void ExecutionPage::RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

}

void ExecutionPage::RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    Q_UNUSED(last);
}

void ExecutionPage::RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    Q_UNUSED(last);
}


void ExecutionPage::onEntryInitialState()
{
    INFO_START_FUNCTION();

    // 初期化処理.
    ui->labelElapsedTimeValue->setText(secondToTimeStringUpToHour(0));
    targetRoutePointIndex_ = DEFAULT_TARGET_ROUTE_POINT_INDEX;

    // 移動経路の固定表示False.
    ui->goalSideView->setFixedStart(false);
    ui->goalTopDownView->setFixedStart(false);
}

void ExecutionPage::onEntryErrorState()
{
    INFO_START_FUNCTION();

    // 既存ダイアログを全て閉じる.
    if(QApplication::activeModalWidget() != nullptr)
    {
        QApplication::activeModalWidget()->close();
    }

    // エラーダイアログ表示.
    DialogFactory::alert("An error was detected.\nPlease check the event that occurred.");
    stateMachine_.submitEvent("SwitchPage");
}

void ExecutionPage::onExitErrorState()
{
    // メイン画面に切り替えを通知する.
    emit readyToSwitch(SwitchPageEvent::ERROR);
}

void ExecutionPage::onEntryNormalOperation()
{
    INFO_START_FUNCTION();

    // 移動経路の固定表示True.
    ui->goalSideView->setFixedStart(true);
    ui->goalTopDownView->setFixedStart(true);

    // ステータスの初期化
    ui->listExecutionStatus->initialize(routeInformation_);
    ui->labelStartAtValue->setText(dateTimeString(QDateTime::currentDateTime()));

    // 経過時間の計測開始.
    elapsedTimeSecond_ = 0;
    elapsedTimer_.start();

    // ボタン表示の初期化.
    ui->execPauseButton->setStatus(false);
    ui->execPauseButton->setEnabled(true);
    ui->execCancelButton->setEnabled(true);
}

void ExecutionPage::onEntryNormalOperation_InPreparation()
{
    INFO_START_FUNCTION();

    // 待機中の表示を解除.
    ui->listExecutionStatus->setWaiting(false);
    ui->labelRemainingWaitingTimeValue->setNum(0);
    ui->labelRemainingWaitingTimeValue->hide();
    ui->labelRemainingWaitingTime->hide();
    ui->lineRemainingWaitingTime->hide();

    if(targetRoutePointIndex_ >= routeInformation_->count())
    {
        // ゴールに到達.
        stateMachine_.submitEvent("TargetGuidanceControl.ReachTheGoal");
    }
    else
    {
        LOG_INFO() << "Next index: " << targetRoutePointIndex_;

        // 次の点を目標地点に設定する.
        targetRoutePoint_ = routeInformation_->data(targetRoutePointIndex_);

        LOG_INFO() << " Next porint: position=" << targetRoutePoint_.position() << " orientation=" << targetRoutePoint_.orientation();

        // 残り待機時間の初期化.
        restOfWaitingTime_ = routeInformation_->data(targetRoutePointIndex_).waitSecond();
        LOG_INFO() << " Next porint: waitingTime=" << restOfWaitingTime_;

        // 表示の初期化.
        ui->listExecutionStatus->setNext(targetRoutePointIndex_);


        // 移動開始
        stateMachine_.submitEvent("TargetGuidanceControl.GoNext");

        // 次回呼び出し向けにインデックスを更新.
        targetRoutePointIndex_++;
    }
}

void ExecutionPage::onEntryNormalOperation_PostProcess()
{
    INFO_START_FUNCTION();

    stateMachine_.submitEvent("SwitchPage");
}

void ExecutionPage::onExitNormalOperation_PostProcess()
{
    INFO_START_FUNCTION();

    // メイン画面に切り替えを通知する.
    emit readyToSwitch(SwitchPageEvent::DONE);
}


void ExecutionPage::onEntryTargetGuidanceControl_Processing()
{
    INFO_START_FUNCTION();

    if(!telecommandClient_->sendTargetGoalAbsolute(targetRoutePoint_.position(),
                                                  targetRoutePoint_.orientation()))
    {
        LOG_WARNING() << "Sending the move command failed";
        stateMachine_.submitEvent("Error.Detected");
    }
}

void ExecutionPage::onExitTargetGuidanceControl_Processing()
{
    INFO_START_FUNCTION();
}

void ExecutionPage::elapsedTimerHandler()
{
    elapsedTimeSecond_++;

    // 経過時間タイマーの表示更新.
    ui->labelElapsedTimeValue->setText(secondToTimeStringUpToHour(elapsedTimeSecond_));
}

void ExecutionPage::onEntryTargetGuidanceControl_Waiting()
{
    INFO_START_FUNCTION();
    if(restOfWaitingTime_ <= 0)
    {
        // 即座に次状態に遷移する.
        stateMachine_.submitEvent("TargetGuidanceControl.CheckNext");
    }
    else
    {
        // 現在の位置で待機する.

        // 待機中の表示を有効化.
        ui->listExecutionStatus->setWaiting(true);
        ui->labelRemainingWaitingTimeValue->setText(secondToTimeStringUpToMinute(restOfWaitingTime_));
        ui->labelRemainingWaitingTimeValue->show();
        ui->labelRemainingWaitingTime->show();
        ui->lineRemainingWaitingTime->show();

        // タイマーのカウント開始.
        waitingTimer_.start();
    }
}

void ExecutionPage::onExitTargetGuidanceControl_Waiting()
{
    INFO_START_FUNCTION();

    // 待機用タイマーの停止.
    waitingTimer_.stop();
}

void ExecutionPage::waitingTimerHandler()
{
    if(restOfWaitingTime_ > 0)
    {
        restOfWaitingTime_--;
    }

    // 残り待機秒数の表示更新.
    ui->labelRemainingWaitingTimeValue->setText(secondToTimeStringUpToMinute(restOfWaitingTime_));
    if(restOfWaitingTime_ == 0)
    {
        // 待機終了、次状態への遷移をチェック.
        // onExit内でタイマーを終了する.
        stateMachine_.submitEvent("TargetGuidanceControl.CheckNext");
    }
}


void ExecutionPage::onEntryTargetGuidanceControl_Goal()
{
    INFO_START_FUNCTION();

    // 経過時間タイマーの停止.
    elapsedTimer_.stop();

    // ゴール到着を通知.
    DialogFactory::showInformation("Arrived at the specified goal");
    stateMachine_.submitEvent("NormalOperation.Finish");
}

void ExecutionPage::onEntryPause_Waiting()
{
    INFO_START_FUNCTION();
    telecommandClient_->sendCtlCommandStop();
}

void ExecutionPage::onEntryPause_Completed()
{
    INFO_START_FUNCTION();
}

void ExecutionPage::onEntryCancel_Waiting()
{
    INFO_START_FUNCTION();
}

void ExecutionPage::onEntryCancel_Completed()
{
    INFO_START_FUNCTION();

    // キャンセル完了を通知.
    DialogFactory::showInformation("Cancel the operation");

    // 完了.
    stateMachine_.submitEvent("NormalOperation.Finish");
}

void ExecutionPage::onEntryEmergencyStop()
{
    INFO_START_FUNCTION();

    // 停止完了を通知.
    DialogFactory::showInformation("Stop the operation");

    // 完了.
    stateMachine_.submitEvent("NormalOperation.Finish");
}

void ExecutionPage::on_execPauseButton_clicked()
{
    INFO_START_FUNCTION();
    if(ui->execPauseButton->getStatus() == CommandToolButton::ADDITIONAL_STATE)
    {
        if(DialogFactory::executeConfirm("Do you want to restart?")) {
            // Start実行（再開）.
            stateMachine_.submitEvent("TargetGuidanceControl.Restart");
            ui->execPauseButton->changeStatus();
            if(stateMachine_.isActive("TargetGuidanceControl.Processing"))
            {
                // 移動中だった場合,テレメトリで誘導制御の制御開始を検知できるまで一時停止/を無効.
                ui->execPauseButton->setDisabled(true);
            }
        }
    }
    else
    {
        if(DialogFactory::executeConfirm("Do you want to pause the move?")) {
            // Pause実行.
            stateMachine_.submitEvent("TargetGuidanceControl.PauseClicked");
            ui->execPauseButton->changeStatus();
            // テレメトリで誘導制御の停止を検知できるまで無効.
            ui->execPauseButton->setDisabled(true);
        }
    }
}

void ExecutionPage::on_execCancelButton_clicked()
{
    INFO_START_FUNCTION();
    if(DialogFactory::executeConfirm("Do you want to cancel the operation?")) {
        // タイマーのカウントを停止.
        waitingTimer_.stop();
        // ボタン操作を無効化.
        ui->execPauseButton->setDisabled(true);
        ui->execCancelButton->setDisabled(true);

        // コマンド送信.
        if(!telecommandClient_->sendCtlCommandStop())
        {
            LOG_WARNING() << "Sending the stop command failed";
            stateMachine_.submitEvent("Error.Detected");
        }

        // Cancel実行.
        stateMachine_.submitEvent("Cancel.Clicked");
    }
}

void intball::ExecutionPage::on_emergencyButton_clicked()
{
    INFO_START_FUNCTION();
    if(DialogFactory::executeConfirm("Do you want to stop the operation?")) {

        // ボタン操作を無効化.
        ui->execPauseButton->setDisabled(true);
        ui->execCancelButton->setDisabled(true);

        // コマンド送信.
        if(!telecommandClient_->sendCtlCommandCancel())
        {
            LOG_WARNING() << "Sending the stop command failed";
            stateMachine_.submitEvent("Error.Detected");
        }

        // 停止実行.
        stateMachine_.submitEvent("EmergencyStop.Clicked");
    }
}
