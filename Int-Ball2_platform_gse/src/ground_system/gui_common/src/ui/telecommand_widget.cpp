#include <QHeaderView>
#include <QScrollBar>
#include "common_log_object.h"
#include "ros_common.h"
#include "telecommand_widget.h"
#include "telecommand_client.h"
#include "telemetry_telecommand_config.h"
#include "tf/transform_datatypes.h"
#include "ui_telecommand_widget.h"
#include "utils.h"

using namespace intball;
using namespace intball::telecommand;

TelecommandWidget::TelecommandWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandWidget),
    logAreaWidget_(new TelecommandLogAreaWidget(this))
{
    ui->setupUi(this);

    qRegisterMetaType<CommandLog>();
}

TelecommandWidget::~TelecommandWidget()
{
    delete ui;
}

void TelecommandWidget::enableControlsOnly()
{
    ui->column003->hide();
}

void TelecommandWidget::enableParameterSettingsOnly()
{
    ui->column001->hide();
    ui->column002->hide();
}

void TelecommandWidget::setLogAreaWidget(TelecommandLogAreaWidget* widget)
{
    logAreaWidget_ = widget;

    // コマンド実行結果ログ.
    connect(ui->widgetTarget, &TelecommandTargetWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetManager, &TelecommandManagerWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetNavigation, &TelecommandNavigationWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetProp, &TelecommandPropWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetLedDisplay, &TelecommandLedWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetLedDisplaySettings, &TelecommandLedSettingsWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetCameraMicrophone, &TelecommandCameraMicrophoneWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetCameraMicrophoneSettings, &TelecommandCameraMicrophoneSettingsWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetDock, &TelecommandDockWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
    connect(ui->widgetRosparams, &TelecommandRosparamWidget::executed, logAreaWidget_, &TelecommandLogAreaWidget::setLog);
}

void TelecommandWidget::showLogAreaWidget()
{
    if(logAreaWidget_ != nullptr)
    {
        logAreaWidget_->setParent(ui->logArea);
        ui->logArea->layout()->addWidget(logAreaWidget_);
    }
}
