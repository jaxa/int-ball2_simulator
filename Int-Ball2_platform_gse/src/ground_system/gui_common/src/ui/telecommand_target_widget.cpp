#include "telecommand_target_widget.h"
#include "ui_telecommand_target_widget.h"
#include <QtMath>
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"
#include "telemetry_telecommand_config.h"
#include "utils.h"

using namespace intball;
using namespace intball::telecommand;
using namespace intball::message;

TelecommandTargetWidget::TelecommandTargetWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandTargetWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandTargetWidget::executed);

    for(auto i = telecommand::CTL_ACTION_TYPE_LABEL.keyBegin(); i != telecommand::CTL_ACTION_TYPE_LABEL.keyEnd(); ++i)
    {
        ui->comboBoxCtlCommandType->addItem(telecommand::CTL_ACTION_TYPE_LABEL[*i]);
    }
    ui->groupBoxTarget->setEnabled(false);
}

TelecommandTargetWidget::~TelecommandTargetWidget()
{
    delete ui;
}

void TelecommandTargetWidget::on_comboBoxCtlCommandType_currentIndexChanged(const QString &arg1)
{
    if((arg1 == telecommand::CTL_ACTION_TYPE_LABEL[ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET]) ||
            (arg1 == telecommand::CTL_ACTION_TYPE_LABEL[ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET]))
    {
        ui->groupBoxTarget->setEnabled(true);
    }
    else
    {
        ui->groupBoxTarget->setEnabled(false);
    }
}


void TelecommandTargetWidget::on_pushButtonSendAction_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_CTL_COMMAND))
    {
        return;
    }
    QVector3D position(static_cast<float>(ui->targetInputX->value()),
                       static_cast<float>(ui->targetInputY->value()),
                       static_cast<float>(ui->targetInputZ->value()));
    QQuaternion attitude(tfToQt(tf::createQuaternionFromRPY(
                                    qDegreesToRadians(static_cast<double>(ui->targetInputRoll->value())),
                                    qDegreesToRadians(static_cast<double>(ui->targetInputPitch->value())),
                                    qDegreesToRadians(static_cast<double>(ui->targetInputYaw->value())))));

    bool result = false;
    int type = getKeyFromValue(telecommand::CTL_ACTION_TYPE_LABEL, ui->comboBoxCtlCommandType->currentText());
    if(type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET)
    {
        result = client_->sendTargetGoalAbsolute(position, attitude);
    }
    else if(type == ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET)
    {
        result = client_->sendTargetGoalRelative(position, attitude);
    }
    else
    {
        result = client_->sendCtlCommand(type, QVector3D(), QQuaternion());
    }
}

void TelecommandTargetWidget::on_pushButtonCancelAction_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_CANCEL_CTL_COMMAND))
    {
        client_->sendCtlCommandCancel();
    }
}
