#include "telecommand_manager_widget.h"
#include "ui_telecommand_manager_widget.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::message;

TelecommandManagerWidget::TelecommandManagerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandManagerWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandManagerWidget::executed);
}

TelecommandManagerWidget::~TelecommandManagerWidget()
{
    delete ui;
}

void TelecommandManagerWidget::on_buttonExitDockingMode_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_EXIT_DOCKING_MODE))
    {
        return;
    }
    auto mode = ui->comboBoxExitDockingMode->currentIndex() == 0 ? ib2_msgs::Mode::OPERATION : ib2_msgs::Mode::STANDBY;
    client_->sendExitDockingMode(mode);
}


void TelecommandManagerWidget::on_buttonSetMaintenanceMode_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_SET_MAINTENANCE_MODE))
    {
        return;
    }
    auto mode = ui->comboBoxSetMaintenanceMode->currentIndex() == 0 ? ib2_msgs::Mode::MAINTENANCE : ib2_msgs::Mode::STANDBY;
    client_->sendSetMaintenanceMode(mode);
}

void TelecommandManagerWidget::on_buttonForcedRelease_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_FORCED_RELEASE))
    {
        return;
    }
    client_->sendForcedRelease();
}

void TelecommandManagerWidget::on_buttonReboot_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_REBOOT))
    {
        return;
    }
    client_->sendReboot();
}
