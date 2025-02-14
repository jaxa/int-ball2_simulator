#include <QDoubleValidator>
#include "telecommand_led_widget.h"
#include "ui_telecommand_led_widget.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::message;

TelecommandLedWidget::TelecommandLedWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandLedWidget)
{
    ui->setupUi(this);
    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandLedWidget::executed);
}

TelecommandLedWidget::~TelecommandLedWidget()
{
    delete ui;
}

void TelecommandLedWidget::on_buttonDisplayManagementOn_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_DISPLAY_MANAGEMENT)))
    {
        return;
    }
    client_->sendDisplayManagerSwitch(true);
}

void TelecommandLedWidget::on_buttonDisplayManagementOff_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_DISPLAY_MANAGEMENT)))
    {
        return;
    }
    client_->sendDisplayManagerSwitch(false);
}

void TelecommandLedWidget::on_buttonLightingOn_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_LIGHTING)))
    {
        return;
    }
    client_->sendLighting(true);
}

void TelecommandLedWidget::on_buttonLightingOff_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_LIGHTING)))
    {
        return;
    }
    client_->sendLighting(false);
}
