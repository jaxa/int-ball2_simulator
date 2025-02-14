#include "telecommand_prop_widget.h"
#include "ui_telecommand_prop_widget.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::message;

TelecommandPropWidget::TelecommandPropWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandPropWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandPropWidget::executed);
}

TelecommandPropWidget::~TelecommandPropWidget()
{
    delete ui;
}

void TelecommandPropWidget::on_pushButtonPropOn_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_PROPULSION))){
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::PROP, true);
}

void TelecommandPropWidget::on_pushButtonPropOff_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_PROPULSION))){
        return;
    }
    client_->sendSwitchPower(telecommand::SWITCH_POWER_TARGET::PROP, false);
}

void TelecommandPropWidget::on_DutySendButton_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_DUTY)){
        return;
    }
    QList<double> duty;
    duty.push_back(ui->doubleSpinBox0->value());
    duty.push_back(ui->doubleSpinBox1->value());
    duty.push_back(ui->doubleSpinBox2->value());
    duty.push_back(ui->doubleSpinBox3->value());
    duty.push_back(ui->doubleSpinBox4->value());
    duty.push_back(ui->doubleSpinBox5->value());
    duty.push_back(ui->doubleSpinBox6->value());
    duty.push_back(ui->doubleSpinBox7->value());
    client_->sendDuty(duty);
}

void TelecommandPropWidget::on_doubleSpinBoxAll_valueChanged(double arg1)
{
    ui->doubleSpinBox0->setValue(arg1);
    ui->doubleSpinBox1->setValue(arg1);
    ui->doubleSpinBox2->setValue(arg1);
    ui->doubleSpinBox3->setValue(arg1);
    ui->doubleSpinBox4->setValue(arg1);
    ui->doubleSpinBox5->setValue(arg1);
    ui->doubleSpinBox6->setValue(arg1);
    ui->doubleSpinBox7->setValue(arg1);
}
