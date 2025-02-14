#include "telecommand_camera_microphone_widget.h"
#include "ui_telecommand_camera_microphone_widget.h"
#include "camera_config.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "gui_config_base.h"
#include "ib2_msgs.h"
#include "ros_common.h"
#include "telemetry_telecommand_config.h"
#include "utils.h"

using namespace intball;
using namespace intball::message;

TelecommandCameraMicrophoneWidget::TelecommandCameraMicrophoneWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandCameraMicrophoneWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandCameraMicrophoneWidget::executed);
}

TelecommandCameraMicrophoneWidget::~TelecommandCameraMicrophoneWidget()
{
    delete ui;
}

void TelecommandCameraMicrophoneWidget::on_pushButtonStreamingON_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_STREAMING))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::CAMERA_STREAMING, true);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonStreamingOFF_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_STREAMING))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::CAMERA_STREAMING, false);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonRecordingON_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_RECORD))
    {
        return;
    }
    client_->sendRecord(true);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonRecordingOFF_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_RECORD))
    {
        return;
    }
    client_->sendRecord(false);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonCameraON_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_CAMERA_POWER))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_CAMERA, true);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonCameraOFF_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_CAMERA_POWER))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_CAMERA, false);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonMicrophoneON_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_MICROPHONE_POWER))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_MICROPHONE, true);
}

void TelecommandCameraMicrophoneWidget::on_pushButtonMicrophoneOFF_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_MICROPHONE_POWER))
    {
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::DEVICE_MICROPHONE, false);
}
