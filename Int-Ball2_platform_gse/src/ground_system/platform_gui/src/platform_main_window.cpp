#include <ros/ros.h>
#include <QStandardItemModel>
#include <ros/package.h>
#include "ros_common.h"
#include "platform_main_window.h"
#include "ui_platform_main_window.h"
#include "common_log_object.h"
#include "gui_common.h"
#include "gui_color.h"
#include "ib2_msgs.h"
#include "platform_msgs.h"
#include "dialog_factory.h"
#include "qdebug_custom.h"
#include "platform_gui_config.h"
#include "telemetry_telecommand_config.h"
#include "telecommand_client.h"
#include "telemetry_subscriber.h"
#include "utils.h"
#include "monitor_status_model.h"
#include "amount_slider_widget.h"
#include "status_widget.h"

using namespace intball;
using namespace qsettings;
using namespace qsettings::key;
using namespace intball::telemetry;
using namespace intball::message;
using namespace platform_msgs;

const QColor PlatformMainWindow::DEFAULT_FRAME_COLOR = Color::U2;
const QString PlatformMainWindow::SUFFIX_ALIVE_STATUS_MAIN_CAMERA = "/camera_main/status";
const QString PlatformMainWindow::SUFFIX_ALIVE_STATUS_LEFT_CAMERA = "/camera_left/status";
const QString PlatformMainWindow::SUFFIX_ALIVE_STATUS_RIGHT_CAMERA = "/camera_right/status";
const QString PlatformMainWindow::SUFFIX_ALIVE_STATUS_MICROPHONE = "/microphone/status";

namespace  {

template<typename T>
void buttonStatusCheckAndExecuteCommand(const T* button, const QString& label, const QString& command, std::function<void()> const& callbackOk)
{
    auto beforeStatus = button->getStatus();
    if (button->isWaiting())
    {
        // コマンド送信後の待機中

        DialogFactory::telecommandWaitCheck(callbackOk);
        return;
    }
    else if(!button->isInitialized())
    {
        // 現在状態が未設定（テレメトリを未受信）
        DialogFactory::telecommandInitCheck(callbackOk);
        return;
    }

    if(beforeStatus != button->getStatus())
    {
        LOG_INFO() << label << ": Processing was interrupted because the state of the RecordMovieButton changed during processing.";
        DialogFactory::showInformation("Processing was interrupted because the recording status changed during processing.");
        return;
    }

    DialogFactory::telecommandCheck(command, callbackOk);
}

}

PlatformMainWindow::PlatformMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PlatformMainWindow),
    lastMonitorStatusCheckTime_(QDateTime()),
    isOffNominalDetected_(false)
{
    ui->setupUi(this);
    setWindowTitle(tr("Platform GUI"));

    monitorStatusClearInterval_ = Config::valueAsInt(KEY_NODE_STATUS_CLEAR_INTERVAL);

    // テレメトリ.
    intballTelemetry_ = new IntBallTelemetry();
    connect(intballTelemetry_, &QAbstractListModel::dataChanged, this, &PlatformMainWindow::IntBall2Telemetry_dataChanged);

    dockTelemetry_ = new DockTelemetry();

    // テレメトリからのイベント検知（モニター）.
    telemetryMonitor_ = new TelemetryMonitor(intballTelemetry_, dockTelemetry_);
    connect(telemetryMonitor_, &TelemetryMonitor::offNominalDetected, this, &PlatformMainWindow::TelemetryMonitor_offNominalDetected);
    connect(telemetryMonitor_, &TelemetryMonitor::releaseAllOffNominal, this, &PlatformMainWindow::TelemetryMonitor_releaseAllOffNominal);

    telecommandClient_ = new TelecommandClient(*getNodeHandle());

    // 初期表示時.
    getUserPackageList();
    getContainerImageList();
    switchingControlOperations();
    connect(ui->controlUserNodeCommandComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlatformMainWindow::switchingUserLaunchFileComboBox);
    connect(ui->controlUserLaunchFileCommandComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlatformMainWindow::switchingControlOperations);
    connect(ui->controlUserImageCommandComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlatformMainWindow::switchingControlOperations);

    QStringList containerStatusHeaderLabel;
    containerStatusHeaderLabel.append("ID");
    containerStatusHeaderLabel.append("Image");
    containerStatusHeaderLabel.append("Status");
    containerStatusHeaderLabel.append("Timestamp");
    ui->tableWidgetContainerMonitorStatus->setHorizontalHeaderLabels(containerStatusHeaderLabel);
    ui->tableWidgetContainerMonitorStatus->horizontalHeader()->setDefaultAlignment(Qt::AlignLeft);
    ui->tableWidgetContainerMonitorStatus->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

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

    // テレメトリ情報表示用ウィジット.
    ui->statusWidget->initialize(intballTelemetry_, dockTelemetry_, telemetryMonitor_);

    // カメラ・マイク関連ボタン.
    ui->controlMainCameraStreamingButton->setAdditionalState("PLAY", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));
    ui->controlLeftCameraStreamingButton->setAdditionalState("PLAY", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));
    ui->controlRightCameraStreamingButton->setAdditionalState("PLAY", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));
    ui->controlMicrophoneStreamingButton->setAdditionalState("PLAY", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));

    // 特殊コマンドの選択初期設定.
    ui->controlUserNodeCommandComboBox->setCurrentIndex(0);
    ui->controlUserLaunchFileCommandComboBox->setCurrentIndex(0);
    ui->controlUserImageCommandComboBox->setCurrentIndex(0);

    // 監視情報
    monitorStatusModel_.setHorizontalHeaderLabels(QStringList({"Monitor status"}));

    // その他初期化が完了してからテレメトリの受信処理を開始する.
    telemetrySubscriber_ = new TelemetrySubscriber();
    telemetrySubscriber_->start(*getNodeHandle(), intballTelemetry_, dockTelemetry_);
}

PlatformMainWindow::~PlatformMainWindow()
{
    delete ui;
}

void PlatformMainWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    ros::shutdown();
}

void PlatformMainWindow::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    // カメラ状態を受信済みであれば,ボタンにステータス値を反映する.
    if(intballTelemetry_->getInsertStatus(telemetry::Index::CAMERA_MAIN_STREAMING_STATUS))
    {
        auto streamingStatus = intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_MAIN_STREAMING_STATUS).status;
        ui->controlMainCameraStreamingButton->setStatus(streamingStatus == platform_msgs::PowerStatus::ON);
    }

    if(intballTelemetry_->getInsertStatus(telemetry::Index::CAMERA_LEFT_STREAMING_STATUS))
    {
        auto streamingStatus = intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_LEFT_STREAMING_STATUS).status;
        ui->controlLeftCameraStreamingButton->setStatus(streamingStatus == platform_msgs::PowerStatus::ON);
    }

    if(intballTelemetry_->getInsertStatus(telemetry::Index::CAMERA_RIGHT_STREAMING_STATUS))
    {
        auto streamingStatus = intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::CAMERA_RIGHT_STREAMING_STATUS).status;
        ui->controlRightCameraStreamingButton->setStatus(streamingStatus == platform_msgs::PowerStatus::ON);
    }

    if(intballTelemetry_->getInsertStatus(telemetry::Index::MICROPHONE_STREAMING_STATUS))
    {
        auto streamingStatus = intballTelemetry_->data<platform_msgs::PowerStatus>(telemetry::Index::MICROPHONE_STREAMING_STATUS).status;
        ui->controlMicrophoneStreamingButton->setStatus(streamingStatus == platform_msgs::PowerStatus::ON);
    }

    /*
     * ユーザプログラムノードからのメッセージ.
     */

    // 汎用メッセージ
    if(intballTelemetry_->getInsertStatus(telemetry::Index::USER_NODE_STATUS_TIMESTAMP))
    {
        ui->labelUseLogicMessageTimestampValue->setText(intballTelemetry_->getUserNodeStatusTimestampString());
        displayUserNodeMessage(intballTelemetry_->getUserNodeStatusMessage().toStdVector());
    }

    // 監視結果の表示
    if(intballTelemetry_->getInsertStatus(telemetry::Index::PLATFORM_MONITOR_CHECK_TIME) ||
            intballTelemetry_->getInsertStatus(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS) ||
            intballTelemetry_->getInsertStatus(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS) ||
            intballTelemetry_->getInsertStatus(telemetry::Index::PLATFORM_MONITOR_SERVICES))
    {
        if(lastMonitorStatusCheckTime_.isValid() && lastMonitorStatusCheckTime_ > intballTelemetry_->getPlatformMonitorCheckTimeAsQDateTime())
        {
            // 監視結果タイムスタンプが巻き戻った場合, 機体側の再起動か, シミュレータの再起動が実行されたと判断し、
            // 古い監視結果を一旦除去する.
            monitorStatusModel_.deleteAll();
        }
        monitorStatusModel_.updateNode(MonitorStatusModel::KeyType::PUBLICATIONS, intballTelemetry_->getPlatformMonitorPublications().toStdVector());
        monitorStatusModel_.updateNode(MonitorStatusModel::KeyType::SUBSCRIPTIONS, intballTelemetry_->getPlatformMonitorSubscriptions().toStdVector());
        monitorStatusModel_.updateNode(MonitorStatusModel::KeyType::SERVICES, intballTelemetry_->getPlatformMonitorServices().toStdVector());

        // 監視結果タイムスタンプよりも古いデータを消去する
        monitorStatusModel_.deleteOldData(intballTelemetry_->getPlatformMonitorCheckTimeAsQDateTime(), monitorStatusClearInterval_);
        ui->treeViewMonitorStatus->setModel(&monitorStatusModel_);
        ui->treeViewMonitorStatus->expandAll();
        ui->treeViewMonitorStatus->resizeColumnToContents(0);
        ui->treeViewMonitorStatus->resizeColumnToContents(1);

        lastMonitorStatusCheckTime_ = intballTelemetry_->getPlatformMonitorCheckTimeAsQDateTime();
    }

    // コンテナの監視結果
    if(intballTelemetry_->getInsertStatus(telemetry::Index::PLATFORM_MONITOR_CONTAINERS))
    {
        auto containerStatusVector = intballTelemetry_->data<QVector<ContainerStatus>>(telemetry::Index::PLATFORM_MONITOR_CONTAINERS);
        // 現在の行を削除して新しいデータを表示する.
        ui->tableWidgetContainerMonitorStatus->setRowCount(0);
        for(auto i = containerStatusVector.begin(); i != containerStatusVector.end(); ++i)
        {
            // 正常なID文字列を持ち, 監視結果タイムスタンプよりも古すぎないデータのみを表示対象とする.
            if(!i->id.empty() &&
                    (lastMonitorStatusCheckTime_.isValid() &&
                     (lastMonitorStatusCheckTime_.toSecsSinceEpoch() - rosToQt(i->stamp).toSecsSinceEpoch()) < monitorStatusClearInterval_))
            {
                QString id = QString::fromStdString(i->id);
                auto itemList = ui->tableWidgetContainerMonitorStatus->findItems(id, Qt::MatchFlag::MatchFixedString);
                QTableWidgetItem* targetItemId;
                QTableWidgetItem* targetItemImage;
                QTableWidgetItem* targetItemStatus;
                QTableWidgetItem* targetItemCheckTime;
                ui->tableWidgetContainerMonitorStatus->insertRow(0);
                ui->tableWidgetContainerMonitorStatus->setItem(0, 0, new QTableWidgetItem());
                ui->tableWidgetContainerMonitorStatus->setItem(0, 1, new QTableWidgetItem());
                ui->tableWidgetContainerMonitorStatus->setItem(0, 2, new QTableWidgetItem());
                ui->tableWidgetContainerMonitorStatus->setItem(0, 3, new QTableWidgetItem());
                targetItemId = ui->tableWidgetContainerMonitorStatus->item(0, 0);
                targetItemImage = ui->tableWidgetContainerMonitorStatus->item(0, 1);
                targetItemStatus = ui->tableWidgetContainerMonitorStatus->item(0, 2);
                targetItemCheckTime = ui->tableWidgetContainerMonitorStatus->item(0, 3);
                targetItemId->setText(id);
                targetItemImage->setText(QString::fromStdString(i->image));
                targetItemStatus->setText(getContainerStatusString(i->status));
                targetItemCheckTime->setText(dateTimeString(i->stamp));
            }
        }
    }

    switchingControlOperations();
}

void PlatformMainWindow::on_controlMainCameraStreamingButton_clicked()
{
    // メインカメラのストリーミングON/OFF
    buttonStatusCheckAndExecuteCommand(ui->controlMainCameraStreamingButton,
                                       "controlMainCameraStreamingButton",
                                       COMMAND_NAME_STREAMING,
                                       std::bind(&PlatformMainWindow::sendMainCameraStreaming, this));
}

void PlatformMainWindow::sendMainCameraStreaming()
{
    // 「ボタン状態が通常状態(OFF)」 = 「PLAY を送信する状態」 = 「SwitchPowerでON（true）」を送信する.
    auto isStatusOff = ui->controlMainCameraStreamingButton->getStatus() == CommandButton::STATE_OFF;

    // rosparam設定.
    RosParam request;
    request.id = rosparam::CAMERA_MAIN_STREAMING_ON;
    request.value = isStatusOff ? "true" : "false";
    request.type = RosParamType::BOOL;

    // rosparam送信, Update parameter.
    if(telecommandClient_->sendSetRosParam(request) &&
       telecommandClient_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_MAIN))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
        ui->controlMainCameraStreamingButton->setWaitingState(true);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::on_controlLeftCameraStreamingButton_clicked()
{
    // 航法カメラ左のストリーミングON/OFF
    buttonStatusCheckAndExecuteCommand(ui->controlLeftCameraStreamingButton,
                                       "controlLeftCameraStreamingButton",
                                       COMMAND_NAME_STREAMING,
                                       std::bind(&PlatformMainWindow::sendLeftCameraStreaming, this));
}

void PlatformMainWindow::sendLeftCameraStreaming()
{
    // 「ボタン状態が通常状態(OFF)」 = 「PLAY を送信する状態」 = 「SwitchPowerでON（true）」を送信する.
    auto isStatusOff = ui->controlLeftCameraStreamingButton->getStatus() == CommandButton::STATE_OFF;

    // rosparam設定.
    RosParam request;
    request.id = rosparam::CAMERA_LEFT_STREAMING_ON;
    request.value = isStatusOff ? "true" : "false";
    request.type = RosParamType::BOOL;

    // rosparam送信, Update parameter.
    if(telecommandClient_->sendSetRosParam(request) &&
       telecommandClient_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_LEFT))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
        ui->controlLeftCameraStreamingButton->setWaitingState(true);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::on_controlRightCameraStreamingButton_clicked()
{
    // 航法カメラ右のストリーミングON/OFF
    buttonStatusCheckAndExecuteCommand(ui->controlRightCameraStreamingButton,
                                       "controlRightCameraStreamingButton",
                                       COMMAND_NAME_STREAMING,
                                       std::bind(&PlatformMainWindow::sendRightCameraStreaming, this));
}

void PlatformMainWindow::sendRightCameraStreaming()
{
    // 「ボタン状態が通常状態(OFF)」 = 「PLAY を送信する状態」 = 「SwitchPowerでON（true）」を送信する.
    auto isStatusOff = ui->controlRightCameraStreamingButton->getStatus() == CommandButton::STATE_OFF;

    // rosparam設定.
    RosParam request;
    request.id = rosparam::CAMERA_RIGHT_STREAMING_ON;
    request.value = isStatusOff ? "true" : "false";
    request.type = RosParamType::BOOL;

    // rosparam送信, Update parameter.
    if(telecommandClient_->sendSetRosParam(request) &&
       telecommandClient_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_RIGHT))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
        ui->controlRightCameraStreamingButton->setWaitingState(true);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::on_controlMicrophoneStreamingButton_clicked()
{
    // マイクのストリーミングON/OFF
    buttonStatusCheckAndExecuteCommand(ui->controlMicrophoneStreamingButton,
                                       "controlMicrophoneStreamingButton",
                                       COMMAND_NAME_AUDIO_STREAMING,
                                       std::bind(&PlatformMainWindow::sendMicrophoneStreaming, this));
}

void PlatformMainWindow::sendMicrophoneStreaming()
{
    // 「ボタン状態が通常状態(OFF)」 = 「PLAY を送信する状態」 = 「SwitchPowerでON（true）」を送信する.
    auto isStatusOff = ui->controlMicrophoneStreamingButton->getStatus() == CommandButton::STATE_OFF;

    // rosparam設定.
    RosParam request;
    request.id = rosparam::MICROPHONE_STREAMING_ON;
    request.value = isStatusOff ? "true" : "false";
    request.type = RosParamType::BOOL;

    // rosparam送信, Update parameter.
    if(telecommandClient_->sendSetRosParam(request) &&
       telecommandClient_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::MICROPHONE))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
        ui->controlMicrophoneStreamingButton->setWaitingState(true);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::getUserPackageList()
{
    std::string path = ros::package::getPath(THIS_PACKAGE_NAME.toStdString());
    std::string file = Config::valueAsStdString(KEY_USER_PACKAGE_LIST_FILE);
    if(file.front() != '/')
    {
        file = "/" + file;
    }

    std::string fullPath = path + file;

    QFile userPackageListFile(QString::fromStdString(fullPath));
    if(!userPackageListFile.open(QFile::ReadOnly))
    {
        LOG_WARNING() << "Open Error: User package file";
        return;
    }

    QTextStream userPackageListText(&userPackageListFile);

    // jsonファイルの読み込み
    QJsonDocument userPackageListJsonDoc = QJsonDocument::fromJson(userPackageListText.readAll().toUtf8());
    QJsonObject userPackageListObj = userPackageListJsonDoc.object();
    userPackageList_ = userPackageListObj.value("user_program_list").toArray();

    // ユーザノードのリストを設定
    //ui->controlUserNodeCommandComboBox->addItem("( Select User Program )", QVariant());
    for(auto userPackageElement : userPackageList_)
    {
        QJsonObject userPackageObj = userPackageElement.toObject();
        QString userNode = userPackageObj.value("user").toString();
        ui->controlUserNodeCommandComboBox->addItem(userNode, QVariant());
    }
    LOG_INFO() << "The user package file was successfully loaded.";
}

void PlatformMainWindow::getContainerImageList()
{
    std::string path = ros::package::getPath(THIS_PACKAGE_NAME.toStdString());
    std::string file = Config::valueAsStdString(KEY_CONTAINER_IMAGE_LIST_FILE);
    if(file.front() != '/')
    {
        file = "/" + file;
    }

    std::string fullPath = path + file;

    QFile containerImageListFile(QString::fromStdString(fullPath));
    if(!containerImageListFile.open(QFile::ReadOnly))
    {
        LOG_WARNING() << "Open Error: Container image file";
        return;
    }

    QTextStream containerImageListText(&containerImageListFile);

    // jsonファイルの読み込み
    QJsonDocument containerImageListJsonDoc = QJsonDocument::fromJson(containerImageListText.readAll().toUtf8());
    QJsonObject containerImageListObj = containerImageListJsonDoc.object();
    containerImageList_ = containerImageListObj.value("container_image_list").toArray();

    // Dockerイメージのリストを設定
    for(auto containerImageElement : containerImageList_)
    {
        QString containerImage = containerImageElement.toString();
        ui->controlUserImageCommandComboBox->addItem(containerImage, QVariant());
    }
    LOG_INFO() << "The container image file was successfully loaded.";
}

void PlatformMainWindow::on_pushButtonUserNodeStart_clicked()
{
    if(ui->controlUserNodeCommandComboBox->currentIndex() != 0 &&
            ui->controlUserLaunchFileCommandComboBox->currentIndex() != 0 &&
            ui->controlUserImageCommandComboBox->currentIndex() != 0)
    {
        // ユーザプログラム実行.
        DialogFactory::telecommandCheck(COMMAND_NAME_USER_PROGRAMING_NODE_START,
                                        std::bind(&PlatformMainWindow::sendUserNode,
                                                  this,
                                                  true,
                                                  ui->controlUserNodeCommandComboBox->currentText(),
                                                  ui->controlUserLaunchFileCommandComboBox->currentText(),
                                                  ui->controlUserImageCommandComboBox->currentText()));
    }
}

void PlatformMainWindow::on_pushButtonUserNodeStop_clicked()
{
    // ユーザプログラム停止.
    DialogFactory::telecommandCheck(COMMAND_NAME_USER_PROGRAMING_NODE_STOP,
                                    std::bind(&PlatformMainWindow::sendUserNode,
                                              this,
                                              false,
                                              "",
                                              "",
                                              ""));
}

void PlatformMainWindow::sendUserNode(const bool on, const QString& user, const QString& launch, const QString& container)
{
    if(telecommandClient_->sendUserNode(on, user, launch, container))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::on_pushButtonUserLogicStart_clicked()
{
    if(ui->controlUserNodeCommandComboBox->currentIndex() != 0)
    {
        // ユーザロジック実行.
        platform_msgs::UserLogic logic;
        logic.id = ui->spinBoxLogicId->text().toUShort();
        DialogFactory::telecommandCheck(COMMAND_NAME_USER_LOGIC_START,
                                        std::bind(&PlatformMainWindow::sendUserLogic,
                                                  this,
                                                  true,
                                                  logic));
    }
}

void PlatformMainWindow::on_pushButtonUserLogicStop_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_USER_LOGIC_STOP,
                                    std::bind(&PlatformMainWindow::sendUserLogic,
                                              this,
                                              false,
                                              platform_msgs::UserLogic()));
}

void PlatformMainWindow::sendUserLogic(const bool on, const platform_msgs::UserLogic& userLogic)
{
    if(telecommandClient_->sendUserLogic(on, userLogic))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::switchingUserLaunchFileComboBox()
{
    ui->controlUserLaunchFileCommandComboBox->clear();
    ui->controlUserLaunchFileCommandComboBox->addItem("( Select Launch File )", QVariant());

    if(ui->controlUserNodeCommandComboBox->currentIndex() != 0)
    {
        for(auto userPackageElement : userPackageList_)
        {
            QJsonObject userPackageObj = userPackageElement.toObject();
            QString userNode = userPackageObj.value("user").toString();
            if(ui->controlUserNodeCommandComboBox->currentText() == userNode)
            {
                QJsonArray launches = userPackageObj.value("launch").toArray();
                for(auto launch : launches)
                {
                    ui->controlUserLaunchFileCommandComboBox->addItem(launch.toString(), QVariant());
                }
            }
        }
    }
}

void PlatformMainWindow::TelemetryMonitor_offNominalDetected(TelemetryMonitor::OffNominalType type)
{
    Q_UNUSED(type)

    isOffNominalDetected_ = true;
    switchingControlOperations();
}

void PlatformMainWindow::TelemetryMonitor_releaseAllOffNominal()
{
    isOffNominalDetected_ = false;
    switchingControlOperations();
}

bool PlatformMainWindow::isNormalMoveCommandEnabled()
{
    auto controlStatus_ = intballTelemetry_->data<int>(telemetry::Index::CTL_STATUS_TYPE);
    auto operationType = intballTelemetry_->data<unsigned char>(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE);
    return (controlStatus_ == ib2_msgs::CtlStatusType::STAND_BY ||
            controlStatus_ == ib2_msgs::CtlStatusType::KEEP_POSE ||
            controlStatus_ == ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION) &&
            operationType == platform_msgs::OperationType::NAV_ON &&
            intballTelemetry_->isPlatformFlightSoftwareStarted();
}

bool PlatformMainWindow::isSetOperationTypeEnabled()
{
    // NAV_ON/NAV_OFFはplatform_msgs::Mode::USER_OFFまたはUSER_READYのときのみ可
    return (intballTelemetry_->getPlatformMode() == platform_msgs::Mode::USER_OFF ||
            intballTelemetry_->getPlatformMode() == platform_msgs::Mode::USER_READY);
}

bool PlatformMainWindow::isLaunchEnabled()
{
    // ユーザノードのlaunchはplatform_msgs::Mode::USER_OFFのときのみ可
    return (intballTelemetry_->getPlatformMode() == platform_msgs::Mode::USER_OFF);
}

bool PlatformMainWindow::isLogicEnabled()
{
    // ユーザロジックのlaunchはplatform_msgs::Mode::USER_READYのときのみ可
    return (intballTelemetry_->getPlatformMode() == platform_msgs::Mode::USER_READY);
}

bool PlatformMainWindow::isMainCameraEnabled()
{
    auto aliveStatusMap = intballTelemetry_->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC);

    // メインカメラのストリーミングは監視結果がSuccessのときのみ可
    if(aliveStatusMap.find(SUFFIX_ALIVE_STATUS_MAIN_CAMERA) != aliveStatusMap.end() &&
       aliveStatusMap.value(SUFFIX_ALIVE_STATUS_MAIN_CAMERA).result == ib2_msgs::AliveStatus::SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PlatformMainWindow::isLeftCameraEnabled()
{
    auto aliveStatusMap = intballTelemetry_->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC);

    // 航法カメラのストリーミングは監視結果がSuccessのときのみ可
    if(aliveStatusMap.find(SUFFIX_ALIVE_STATUS_LEFT_CAMERA) != aliveStatusMap.end() &&
       aliveStatusMap.value(SUFFIX_ALIVE_STATUS_LEFT_CAMERA).result == ib2_msgs::AliveStatus::SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PlatformMainWindow::isRightCameraEnabled()
{
    auto aliveStatusMap = intballTelemetry_->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC);

    // 航法カメラのストリーミングは監視結果がSuccessのときのみ可
    if(aliveStatusMap.find(SUFFIX_ALIVE_STATUS_RIGHT_CAMERA) != aliveStatusMap.end() &&
       aliveStatusMap.value(SUFFIX_ALIVE_STATUS_RIGHT_CAMERA).result == ib2_msgs::AliveStatus::SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PlatformMainWindow::isMicrophoneEnabled()
{
    auto aliveStatusMap = intballTelemetry_->data<QMap<QString, ib2_msgs::AliveStatus>>(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC);

    // マイクのストリーミングは監視結果がSuccessのときのみ可
    if(aliveStatusMap.find(SUFFIX_ALIVE_STATUS_MICROPHONE) != aliveStatusMap.end() &&
       aliveStatusMap.value(SUFFIX_ALIVE_STATUS_MICROPHONE).result == ib2_msgs::AliveStatus::SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool PlatformMainWindow::isPlatformFlightSoftwareStarted()
{
    return intballTelemetry_->data<bool>(telemetry::Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS);
}

void PlatformMainWindow::switchingControlOperations()
{
    LOG_DEBUG() << "isNormalMoveCommandEnabled=" << isNormalMoveCommandEnabled() <<
                   " isOffNominalDetected_=" << isOffNominalDetected_ <<
                   " isLaunchEnabled=" << isLaunchEnabled() <<
                   " isLogicEnabled=" << isLogicEnabled() <<
                   " isSetOperationTypeEnabled=" << isSetOperationTypeEnabled() <<
                   " isMainCameraEnabled=" << isMainCameraEnabled() <<
                   " isLeftCameraEnabled=" << isLeftCameraEnabled() <<
                   " isRightCameraEnabled=" << isRightCameraEnabled() <<
                   " isMicrophoneEnabled=" << isMicrophoneEnabled();

    bool normalMoveEnabled = isNormalMoveCommandEnabled();

    ui->pushButtonOperationTypeOn->setEnabled(isSetOperationTypeEnabled());
    ui->pushButtonOperationTypeOff->setEnabled(isSetOperationTypeEnabled());

    ui->controlMainCameraStreamingButton->setEnabled(isMainCameraEnabled());
    ui->controlLeftCameraStreamingButton->setEnabled(isLeftCameraEnabled());
    ui->controlRightCameraStreamingButton->setEnabled(isRightCameraEnabled());
    ui->controlMicrophoneStreamingButton->setEnabled(isMicrophoneEnabled());

    bool isUserNodeStartEnabledByCommandComboBox =
            (ui->controlUserLaunchFileCommandComboBox->currentIndex() != 0 &&
             ui->controlUserImageCommandComboBox->currentIndex() != 0);
    ui->pushButtonUserNodeStart->setEnabled(isPlatformFlightSoftwareStarted() &&
                                            isLaunchEnabled() &&
                                            !isOffNominalDetected_ &&
                                            isUserNodeStartEnabledByCommandComboBox);
    ui->pushButtonUserLogicStart->setEnabled(isPlatformFlightSoftwareStarted() &&
                                             isLogicEnabled() &&
                                             !isOffNominalDetected_);

    // 絶対座標指定の移動.
     ui->controlGoButton->setEnabled(normalMoveEnabled);

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
}

void PlatformMainWindow::on_pushButtonOperationTypeOn_clicked()
{
    // Operation type on.
    DialogFactory::telecommandCheck(COMMAND_NAME_SET_OPERATION_TYPE,
                                    std::bind(&PlatformMainWindow::sendOperationType,
                                              this,
                                              platform_msgs::OperationType::NAV_ON));
}

void PlatformMainWindow::on_pushButtonOperationTypeOff_clicked()
{
    // Operation type off.
    DialogFactory::telecommandCheck(COMMAND_NAME_SET_OPERATION_TYPE,
                                    std::bind(&PlatformMainWindow::sendOperationType,
                                              this,
                                              platform_msgs::OperationType::NAV_OFF));
}

void PlatformMainWindow::sendOperationType(const unsigned char type)
{
    platform_msgs::OperationType operationType;
    operationType.type = type;
    if(telecommandClient_->sendSetOperationType(operationType))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::displayUserNodeMessage(std::vector<char> userNodeStatusData)
{
    // 汎用メッセージの表示
    // 汎用メッセージのデータであるint8[]は、C++ではstd::vectorで定義される
    QByteArray userNodeStatusDataBytes(userNodeStatusData.data(), static_cast<int>(userNodeStatusData.size()));

    // 16進数表記文字列
    QString userNodeStatusHex;
    QByteArray userNodeStatusDataHexStringBytes = userNodeStatusDataBytes.toHex().toUpper();
    QTextStream userNodeStatusHexStream(&userNodeStatusHex);
    for(int i = 0; i < userNodeStatusDataHexStringBytes.size(); ++i)
    {
        userNodeStatusHexStream << userNodeStatusDataHexStringBytes.at(i);
        if(i % 2 > 0)
        {
            // 2文字（1バイト）出力済み
            if(i != 0 && i % (16 * 2) == 0)
            {
                // 16バイト(1バイト2文字 * 16)出力毎に改行する
                userNodeStatusHexStream << "\n";
            }
            else
            {
                userNodeStatusHexStream << " ";
            }
        }
    }

    // ASCII文字列
    QString userNodeStatusAscii = QString::fromUtf8(userNodeStatusDataBytes);
    ui->labelUserStatusHex->setText(userNodeStatusHex);
    ui->labelUserStatusAscii->setText(userNodeStatusAscii);
}

void PlatformMainWindow::on_controlGoButton_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalAbsolute, this));
}

void PlatformMainWindow::sendTargetGoalAbsolute()
{
    QVector3D position(ui->controlInputX->text().toFloat(),
                       ui->controlInputY->text().toFloat(),
                       ui->controlInputZ->text().toFloat());
    QQuaternion orientation = fromRPYDegree(ui->controlInputRoll->text().toDouble(),
                                      ui->controlInputPitch->text().toDouble(),
                                      ui->controlInputYaw->text().toDouble());

    if(telecommandClient_->sendTargetGoalAbsolute(position, orientation))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

void PlatformMainWindow::on_controlPositionUp_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(0, 0, ui->controlAmountPositionSlider->value()),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlPositionRight_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(0, -ui->controlAmountPositionSlider->value(), 0),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlPositionDown_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(0, 0, -ui->controlAmountPositionSlider->value()),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlPositionLeft_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(0, ui->controlAmountPositionSlider->value(), 0),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlPositionFront_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(ui->controlAmountPositionSlider->value(), 0, 0),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlPositionBack_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(-ui->controlAmountPositionSlider->value(), 0, 0),
                                              QQuaternion()));
}

void PlatformMainWindow::on_controlAttitudeRollCounterClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()), 0, 0)));
}

void PlatformMainWindow::on_controlAttitudeRollClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(static_cast<qreal>(ui->controlAmountAttitudeSlider->value()), 0, 0)));
}

void PlatformMainWindow::on_controlAttitudePitchCounterClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(0, static_cast<qreal>(ui->controlAmountAttitudeSlider->value()), 0)));
}

void PlatformMainWindow::on_controlAttitudePitchClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(0, static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()), 0)));
}

void PlatformMainWindow::on_controlAttitudeYawCounterClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(0, static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()), 0)));
}

void PlatformMainWindow::on_controlAttitudeYawClock_clicked()
{
    DialogFactory::telecommandCheck(COMMAND_NAME_MOVE_INTBALL2,
                                    std::bind(&PlatformMainWindow::sendTargetGoalRelative,
                                              this,
                                              QVector3D(),
                                              fromRPYDegree(0, 0, static_cast<qreal>(-ui->controlAmountAttitudeSlider->value()))));
}

void PlatformMainWindow::sendTargetGoalRelative(const QVector3D& position, const QQuaternion& orientation)
{
    if(telecommandClient_->sendTargetGoalRelative(position, orientation))
    {
        // 送信成功.
        DialogFactory::showInformation(DIALOG_MSG_COMMAND_SEND_SUCCESS, false);
    }
    else
    {
        // 送信失敗.
        DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR, false);
    }
}

