#include "telecommand_dock_widget.h"
#include "ui_telecommand_dock_widget.h"
#include <QString>
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"
#include "telemetry_telecommand_config.h"

using namespace intball;
using namespace intball::message;

TelecommandDockWidget::TelecommandDockWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandDockWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandDockWidget::executed);

    QRegExp regExpIp("^[0-9]+\\.[0-9]+\\.[0-9]+\\.[0-9]+$");
    QRegExpValidator* regExpValidator = new QRegExpValidator(regExpIp, this);
    ui->lineEditlabelGroundSystemIP->setValidator(regExpValidator);
    ui->lineEditIntBall2IP->setValidator(regExpValidator);

    QIntValidator* portValidator = new QIntValidator(0, 65535, this);
    ui->lineEditReceivePort->setValidator(portValidator);

    ui->comboBoxChargeOnOff->addItem("",  QVariant(static_cast<uint8_t>(0)));
    ui->comboBoxChargeOnOff->addItem(QString("ON(%1)").arg(static_cast<uint8_t>(dock::telecommand::CHARGE_ON_OFF_TYPE::ON)),
                                    QVariant::fromValue(dock::telecommand::CHARGE_ON_OFF_TYPE::ON));
    ui->comboBoxChargeOnOff->addItem(QString("OFF(%1)").arg(static_cast<uint8_t>(dock::telecommand::CHARGE_ON_OFF_TYPE::OFF)),
                                    QVariant::fromValue(dock::telecommand::CHARGE_ON_OFF_TYPE::OFF));
    ui->comboBoxChargeOnOff->setCurrentIndex(0);

    ui->comboBoxMotorOnOff->addItem("",  QVariant(static_cast<uint8_t>(0)));
    ui->comboBoxMotorOnOff->addItem(QString("STOP(%1)").arg(static_cast<uint8_t>(dock::telecommand::MOTOR_ON_OFF_TYPE::STOP)),
                                   QVariant::fromValue(dock::telecommand::MOTOR_ON_OFF_TYPE::STOP));
    ui->comboBoxMotorOnOff->addItem(QString("RELEASE(%1)").arg(static_cast<uint8_t>(dock::telecommand::MOTOR_ON_OFF_TYPE::RELEASE)),
                                    QVariant::fromValue(dock::telecommand::MOTOR_ON_OFF_TYPE::RELEASE));
    ui->comboBoxMotorOnOff->addItem(QString("DOCK(%1)").arg(static_cast<uint8_t>(dock::telecommand::MOTOR_ON_OFF_TYPE::DOCK)),
                                    QVariant::fromValue(dock::telecommand::MOTOR_ON_OFF_TYPE::DOCK));
    ui->comboBoxMotorOnOff->setCurrentIndex(0);

}

TelecommandDockWidget::~TelecommandDockWidget()
{
    delete ui;
}

void TelecommandDockWidget::disableInput(const dock::telecommand::Index withoutIndex)
{
    if(withoutIndex != dock::telecommand::Index::SET_HOST_IP_ADDR)
    {
        ui->lineEditlabelGroundSystemIP->setText("");
        ui->lineEditlabelGroundSystemIP->setDisabled(true);
    }

    if(withoutIndex != dock::telecommand::Index::SET_IB_ADDR)
    {
        ui->lineEditIntBall2IP->setText("");
        ui->lineEditIntBall2IP->setDisabled(true);
    }


    if(withoutIndex != dock::telecommand::Index::SET_COMMAND_PORT)
    {
        ui->lineEditReceivePort->setText("");
        ui->lineEditReceivePort->setDisabled(true);
    }

    if(withoutIndex != dock::telecommand::Index::CHARGE_ON_OFF)
    {
        ui->comboBoxChargeOnOff->setCurrentIndex(0);
        ui->comboBoxChargeOnOff->setDisabled(true);
    }

    if(withoutIndex != dock::telecommand::Index::MOTOR_ON_OFF)
    {
        ui->comboBoxMotorOnOff->setCurrentIndex(0);
        ui->comboBoxMotorOnOff->setDisabled(true);
    }
}

void TelecommandDockWidget::checkAndEnableInput()
{
    if(ui->lineEditlabelGroundSystemIP->text().isEmpty() &&
            ui->lineEditIntBall2IP->text().isEmpty() &&
            ui->lineEditReceivePort->text().isEmpty() &&
            ui->comboBoxChargeOnOff->currentIndex() == 0 &&
            ui->comboBoxMotorOnOff->currentIndex() == 0
            )
    {
        ui->lineEditlabelGroundSystemIP->setEnabled(true);
        ui->lineEditIntBall2IP->setEnabled(true);
        ui->lineEditReceivePort->setEnabled(true);
        ui->comboBoxChargeOnOff->setEnabled(true);
        ui->comboBoxMotorOnOff->setEnabled(true);
    }
}

void TelecommandDockWidget::checkAndChangeButtonStatus()
{
    if(ui->lineEditlabelGroundSystemIP->text().isEmpty() &&
            ui->lineEditIntBall2IP->text().isEmpty() &&
            ui->lineEditReceivePort->text().isEmpty() &&
            ui->comboBoxMotorOnOff->currentIndex() == 0 &&
            ui->comboBoxChargeOnOff->currentIndex() == 0)
    {
        ui->dockSendButton->setDisabled(true);
        return;
    }

    if(!ui->lineEditlabelGroundSystemIP->text().isEmpty())
    {
        ui->dockSendButton->setEnabled(ui->lineEditlabelGroundSystemIP->hasAcceptableInput());
        return;
    }

    if(!ui->lineEditIntBall2IP->text().isEmpty())
    {
        ui->dockSendButton->setEnabled(ui->lineEditIntBall2IP->hasAcceptableInput());
        return;
    }

    if(!ui->lineEditReceivePort->text().isEmpty())
    {
        ui->dockSendButton->setEnabled(ui->lineEditReceivePort->hasAcceptableInput());
        return;
    }

    if(ui->comboBoxChargeOnOff->currentIndex() != 0)
    {
        ui->dockSendButton->setEnabled(true);
        return;
    }

    if(ui->comboBoxMotorOnOff->currentIndex() != 0)
    {
        ui->dockSendButton->setEnabled(true);
        return;
    }

    ui->dockSendButton->setDisabled(true);
}

void TelecommandDockWidget::on_lineEditlabelGroundSystemIP_textChanged(const QString &arg1)
{
    if(!arg1.isEmpty())
    {
        // 1つのインプットが入力された場合,その他インプットを無効化する.
        disableInput(dock::telecommand::Index::SET_HOST_IP_ADDR);
    }
    else
    {
        // すべての入力がクリアされている場合は全インプットを入力可能とする.
        checkAndEnableInput();
    }

    checkAndChangeButtonStatus();
}

void TelecommandDockWidget::on_lineEditIntBall2IP_textChanged(const QString &arg1)
{
    if(!arg1.isEmpty())
    {
        // 1つのインプットが入力された場合,その他インプットを無効化する.
        disableInput(dock::telecommand::Index::SET_IB_ADDR);
    }
    else
    {
        // すべての入力がクリアされている場合は全インプットを入力可能とする.
        checkAndEnableInput();
    }

    checkAndChangeButtonStatus();
}


void TelecommandDockWidget::on_lineEditReceivePort_textChanged(const QString &arg1)
{
    if(!arg1.isEmpty())
    {
        // 1つのインプットが入力された場合,その他インプットを無効化する.
        disableInput(dock::telecommand::Index::SET_COMMAND_PORT);
    }
    else
    {
        // すべての入力がクリアされている場合は全インプットを入力可能とする.
        checkAndEnableInput();
    }

    checkAndChangeButtonStatus();
}

void TelecommandDockWidget::on_comboBoxChargeOnOff_currentIndexChanged(int index)
{
    if(index != 0)
    {
        // 1つのインプットが入力された場合,その他インプットを無効化する.
        disableInput(dock::telecommand::Index::CHARGE_ON_OFF);
    }
    else
    {
        // すべての入力がクリアされている場合は全インプットを入力可能とする.
        checkAndEnableInput();
    }

    checkAndChangeButtonStatus();
}

void TelecommandDockWidget::on_comboBoxMotorOnOff_currentIndexChanged(int index)
{
    if(index != 0)
    {
        // 1つのインプットが入力された場合,その他インプットを無効化する.
        disableInput(dock::telecommand::Index::MOTOR_ON_OFF);
    }
    else
    {
        // すべての入力がクリアされている場合は全インプットを入力可能とする.
        checkAndEnableInput();
    }

    checkAndChangeButtonStatus();
}


void intball::TelecommandDockWidget::on_dockSendButton_clicked()
{
    if(ui->lineEditlabelGroundSystemIP->isEnabled() && DialogFactory::telecommandCheck(COMMAND_NAME_DOCK_SEND_IP_GROUND))
    {
        client_->sendDockSetHostIPAddr(QHostAddress(ui->lineEditlabelGroundSystemIP->text()));
        ui->lineEditlabelGroundSystemIP->setText("");
        return;
    }

    if(ui->lineEditIntBall2IP->isEnabled() && DialogFactory::telecommandCheck(COMMAND_NAME_DOCK_SEND_IP_INTBALL2))
    {
        client_->sendDockSetIBIPAddr(QHostAddress(ui->lineEditIntBall2IP->text()));
        ui->lineEditIntBall2IP->setText("");
        return;
    }

    if(ui->lineEditReceivePort->isEnabled() && DialogFactory::telecommandCheck(COMMAND_NAME_DOCK_PORT))
    {
        client_->sendDockSetCommandPort(ui->lineEditIntBall2IP->text().toUShort());
        ui->lineEditIntBall2IP->setText("");
        return;
    }

    if(ui->comboBoxChargeOnOff->isEnabled() && DialogFactory::telecommandCheck(COMMAND_NAME_DOCK_CHARGE))
    {
        client_->sendDockChargeOnOff(ui->comboBoxChargeOnOff->currentData().value<dock::telecommand::CHARGE_ON_OFF_TYPE>());
        ui->comboBoxChargeOnOff->setCurrentIndex(0);
        return;
    }

    if(ui->comboBoxMotorOnOff->isEnabled() && DialogFactory::telecommandCheck(COMMAND_NAME_DOCK_MOTOR))
    {
        client_->sendDockMotorOnOff(ui->comboBoxMotorOnOff->currentData().value<dock::telecommand::MOTOR_ON_OFF_TYPE>());
        ui->comboBoxMotorOnOff->setCurrentIndex(0);
        return;
    }
}
