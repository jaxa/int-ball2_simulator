#include "telecommand_navigation_widget.h"
#include "ui_telecommand_navigation_widget.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::message;

TelecommandNavigationWidget::TelecommandNavigationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandNavigationWidget)
{
    ui->setupUi(this);
    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandNavigationWidget::executed);
}

TelecommandNavigationWidget::~TelecommandNavigationWidget()
{
    delete ui;
}

void TelecommandNavigationWidget::on_pushButtonNavigationOn_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_NAVIGATION)))
    {
        return;
    }
    client_->sendNavigationStartUp(true);
}

void TelecommandNavigationWidget::on_pushButtonNavigationOff_clicked()
{
    if(!DialogFactory::telecommandCheck(FORMAT_COMMAND_NAME_SWITCH_POWER.arg(ARG_FUNCTION_NAVIGATION)))
    {
        return;
    }
    client_->sendNavigationStartUp(false);
}

void TelecommandNavigationWidget::on_sendMarkerCorrection_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_MARKER_CORRECTION))
    {
        return;
    }
    client_->sendMarkerCorrection();
}
