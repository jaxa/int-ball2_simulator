#include "crew_support_command_widget.h"
#include "ui_crew_support_command_widget.h"
#include <QMessageBox>
#include <QWidget>
#include "camera_microphone_settings_dialog.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "ib2_msgs.h"
#include "led_settings_dialog.h"
#include "model/intball_telemetry.h"
#include "operator_gui_common.h"
#include "qdebug_custom.h"
#include "telecommand_client.h"

using namespace intball;
using namespace intball::message;

namespace  {

template<typename T>
bool buttonStatusCheck(const T* button, const QString& label)
{
    auto beforeStatus = button->getStatus();
    if (button->isWaiting())
    {
        // コマンド送信後の待機中

        // キャンセル確認
        if(!DialogFactory::telecommandWaitCheck())
        {
            LOG_INFO() << label << ": Processing was canceled because it was waiting after command transmission.";
            return false;
        }
    }
    else if(!button->isInitialized())
    {
        // 現在状態が未設定（テレメトリを未受信）

        // キャンセル確認
        if(!DialogFactory::telecommandInitCheck())
        {
            LOG_INFO() << label << ": Processing has been canceled because the current status is not set (before receiving telemetry).";
            return false;
        }
    }

    if(beforeStatus != button->getStatus())
    {
        LOG_INFO() << label << ": Processing was interrupted because the state of the RecordMovieButton changed during processing.";
        DialogFactory::showInformation("Processing was interrupted because the recording status changed during processing.");
        return false;
    }

    return true;
}

}

CrewSupportCommandWidget::CrewSupportCommandWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CrewSupportCommandWidget)
{
    ui->setupUi(this);

    // カメラ関連ボタン.
    ui->controlStreamingButton->setAdditionalState("PLAY", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));
    ui->controlRecordMovieButton->setAdditionalState("REC", QIcon(":/ground_system/image/controller-stop_fill_white.svg"));

    // カメラまたはマイクがONとなるまで,ストリーミングおよび録画は実行不可.
    ui->controlStreamingButton->setDisabled(true);
    ui->controlRecordMovieButton->setDisabled(true);
}

CrewSupportCommandWidget::~CrewSupportCommandWidget()
{
    delete ui;
}

void CrewSupportCommandWidget::initialize(IntBallTelemetry* intballTelemetry,
                                          TelecommandClient* telecommandClient)

{
    telecommandClient_ = telecommandClient;
    intballTelemetry_ = intballTelemetry;
    connect(intballTelemetry_, &IntBallTelemetry::dataChanged,
            this, &CrewSupportCommandWidget::IntBall2Telemetry_dataChanged);
}

void CrewSupportCommandWidget::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    // カメラ状態を受信済みであれば,ボタンにステータス値を反映する.
    if(intballTelemetry_->getInsertStatus(telemetry::Index::CAMERA_MIC_CAMERA_POWER))
    {
        auto cameraPowerStatus = intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_CAMERA_POWER).status;
        if(cameraPowerStatus == ib2_msgs::PowerStatus::ON)
        {
            ui->controltoggleMainCamera->setStatus(CommandToggleSlider::STATUS::ON);
        }
        else
        {
            ui->controltoggleMainCamera->setStatus(CommandToggleSlider::STATUS::OFF);
        }

        auto microphonePowerStatus = intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_MICROPHONE_POWER).status;
        if(microphonePowerStatus == ib2_msgs::PowerStatus::ON)
        {
            ui->controltoggleMicrophone->setStatus(CommandToggleSlider::STATUS::ON);
        }
        else
        {
            ui->controltoggleMicrophone->setStatus(CommandToggleSlider::STATUS::OFF);
        }

        auto streamingStatus = intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_STREAMING_STATUS).status;
        ui->controlStreamingButton->setStatus(streamingStatus == ib2_msgs::PowerStatus::ON);

        auto recordStatus = intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::CAMERA_MIC_RECORDING_STATUS).status;
        ui->controlRecordMovieButton->setStatus(recordStatus == ib2_msgs::PowerStatus::ON);
    }

    // 表示管理の状態を受信済みであればステータスを反映する.
    if(intballTelemetry_->getInsertStatus(telemetry::Index::DISPLAY_MANAGER_STATUS_MODE))
    {
        auto lightingStatus = intballTelemetry_->data<ib2_msgs::PowerStatus>(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH).status;
        if(lightingStatus == ib2_msgs::PowerStatus::ON)
        {
           ui->controltoggleLighting->setStatus(CommandToggleSlider::STATUS::ON);
        }
        else
        {
            ui->controltoggleLighting->setStatus(CommandToggleSlider::STATUS::OFF);
        }
    }

    // カメラとマイクが両方OFFの場合はストリーミングおよび録画は開始不可.
    if(ui->controltoggleMainCamera->getStatus() == CommandToggleSlider::STATUS::OFF &&
            ui->controltoggleMicrophone->getStatus() == CommandToggleSlider::STATUS::OFF)
    {
        ui->controlStreamingButton->setDisabled(true);
        ui->controlRecordMovieButton->setDisabled(true);
    }
    else
    {
        ui->controlStreamingButton->setEnabled(true);
        ui->controlRecordMovieButton->setEnabled(true);
    }
}

void CrewSupportCommandWidget::on_controlStreamingButton_clicked()
{
    if(!buttonStatusCheck(ui->controlStreamingButton, "controlStreamingButton"))
    {
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_STREAMING))
    {
        // 送信処理.
        // 「ボタン状態が通常状態(OFF)」 = 「PLAY を送信する状態」 = 「SwitchPowerでON（true）」を送信する.
        auto isStatusOff = ui->controlStreamingButton->getStatus() == CommandButton::STATE_OFF;
        if(telecommandClient_->sendSwitchPower(
                    telecommand::SWITCH_POWER_TARGET::CAMERA_STREAMING,
                    isStatusOff ? true : false)
                )
        {
            // 送信成功.
            ui->controlStreamingButton->setWaitingState(true);
        }
        else
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void CrewSupportCommandWidget::on_controlRecordMovieButton_clicked()
{
    if(!buttonStatusCheck(ui->controlRecordMovieButton, "controlRecordMovieButton"))
    {
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_RECORD))
    {
        // 送信処理.
        // 「ボタン状態が通常状態(OFF)」 = 「RECORD ON を送信する状態」 = 「RecordでON（true）」を送信する.
        auto isStatusOff = ui->controlRecordMovieButton->getStatus() == CommandButton::STATE_OFF;
        if(telecommandClient_->sendRecord(isStatusOff ? true : false))
        {
            // 送信成功.
            ui->controlRecordMovieButton->setWaitingState(true);
        }
        else
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void CrewSupportCommandWidget::on_controlSettingMainCameraButton_clicked()
{
    DialogFactory::getCameraMicrophoneSettingsDialog().resetValues();
    DialogFactory::activate(&DialogFactory::getCameraMicrophoneSettingsDialog());
}

void CrewSupportCommandWidget::on_controlSettingLED_clicked()
{
    DialogFactory::getLedSettingsDialog().showGainsOnly(true);
    DialogFactory::activate(&DialogFactory::getLedSettingsDialog());
}

void CrewSupportCommandWidget::on_controltoggleMainCamera_clicked()
{
    if(!buttonStatusCheck(ui->controltoggleMainCamera, "controltoggleMainCamera"))
    {
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_CAMERA_POWER))
    {
        // 現在状態の逆（現在がOFFなら、ON）をリクエストする.
        bool request = ui->controltoggleMainCamera->getStatus() == CommandToggleSlider::STATUS::ON ? false : true;
        if(telecommandClient_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_CAMERA, request))
        {
            // カメラのスイッチを待ち状態（テレメトリの変化待ち状態）に遷移させる.
            ui->controltoggleMainCamera->setWaiting();
        }
        else
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void CrewSupportCommandWidget::on_controltoggleMicrophone_clicked()
{
    if(!buttonStatusCheck(ui->controltoggleMicrophone, "controltoggleMicrophone"))
    {
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_MICROPHONE_POWER))
    {
        // 現在状態の逆（現在がOFFなら、ON）をリクエストする.
        bool request = ui->controltoggleMicrophone->getStatus() == CommandToggleSlider::STATUS::ON ? false : true;
        if(telecommandClient_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_MICROPHONE, request))
        {
            // カメラのスイッチを待ち状態（テレメトリの変化待ち状態）に遷移させる.
            ui->controltoggleMicrophone->setWaiting();
        }
        else
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}

void CrewSupportCommandWidget::on_controltoggleLighting_clicked()
{
    if(!buttonStatusCheck(ui->controltoggleLighting, "controltoggleLighting"))
    {
        return;
    }

    if(DialogFactory::telecommandCheck(COMMAND_NAME_LIGHTING))
    {
        // 現在状態の逆（現在がOFFなら、ON）をリクエストする.
        bool isOn = ui->controltoggleLighting->getStatus() == CommandToggleSlider::STATUS::ON ? false : true;
        if(telecommandClient_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DISPLAY_MANAGER_FLASH, isOn))
        {
            // スイッチを待ち状態（テレメトリの変化待ち状態）に遷移させる.
            ui->controltoggleLighting->setWaiting();
        }
        else
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}
