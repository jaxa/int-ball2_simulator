#include "led_settings_dialog.h"
#include "ui_led_settings_dialog.h"

using namespace intball;

LedSettingsDialog::LedSettingsDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LedSettingsDialog),
    intballTelemetry_(nullptr)
{
    ui->setupUi(this);

    connect(ui->widget, &TelecommandLedSettingsWidget::openDialog, this, &LedSettingsDialog::hide);
    connect(ui->widget, &TelecommandLedSettingsWidget::closeDialog, this, &LedSettingsDialog::show);
}

LedSettingsDialog::~LedSettingsDialog()
{
    delete ui;
}

void LedSettingsDialog::showGainsOnly(const bool flag)
{
    ui->widget->showGainsOnly(flag);
}

void LedSettingsDialog::initialize(IntBallTelemetry* intballTelemetry, TelecommandClient* telecommandClient)
{
    INFO_START_FUNCTION();
    Q_ASSERT(intballTelemetry_ == nullptr);

    intballTelemetry_ = intballTelemetry;
    telecommandClient_ = telecommandClient;
    connect(intballTelemetry_, &IntBallTelemetry::dataChanged, this, &LedSettingsDialog::IntBall2Telemetry_dataChanged);
    ui->groupCurrentLEDGains->show();
}

void LedSettingsDialog::IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    auto publishParameterMap = intballTelemetry_->data<QMap<QString, QString>>(telemetry::Index::PUBLISHING_PARAMS_LIST);
    auto paramIndex = 0;
    for(auto key: publishParameterMap.keys())
    {
        if(key.startsWith(QString::fromStdString(rosparam::LED_LEFT_GAINS)))
        {
            auto splits = publishParameterMap.value(key).split(QRegExp("[\\[\\],\\s]+"), QString::SkipEmptyParts);
            if(splits.count() == 3 * 8)
            {
                paramIndex = 0;

                // index=0
                ui->labelLeftR->setText(splits[paramIndex++]);
                ui->labelLeftG->setText(splits[paramIndex++]);
                ui->labelLeftB->setText(splits[paramIndex++]);
                // index=1
                ui->labelLeftR_2->setText(splits[paramIndex++]);
                ui->labelLeftG_2->setText(splits[paramIndex++]);
                ui->labelLeftB_2->setText(splits[paramIndex++]);
                // index=2
                ui->labelLeftR_3->setText(splits[paramIndex++]);
                ui->labelLeftG_3->setText(splits[paramIndex++]);
                ui->labelLeftB_3->setText(splits[paramIndex++]);
                // index=3
                ui->labelLeftR_4->setText(splits[paramIndex++]);
                ui->labelLeftG_4->setText(splits[paramIndex++]);
                ui->labelLeftB_4->setText(splits[paramIndex++]);
                // index=4
                ui->labelLeftR_5->setText(splits[paramIndex++]);
                ui->labelLeftG_5->setText(splits[paramIndex++]);
                ui->labelLeftB_5->setText(splits[paramIndex++]);
                // index=5
                ui->labelLeftR_6->setText(splits[paramIndex++]);
                ui->labelLeftG_6->setText(splits[paramIndex++]);
                ui->labelLeftB_6->setText(splits[paramIndex++]);
                // index=6
                ui->labelLeftR_7->setText(splits[paramIndex++]);
                ui->labelLeftG_7->setText(splits[paramIndex++]);
                ui->labelLeftB_7->setText(splits[paramIndex++]);
                // index=7
                ui->labelLeftR_8->setText(splits[paramIndex++]);
                ui->labelLeftG_8->setText(splits[paramIndex++]);
                ui->labelLeftB_8->setText(splits[paramIndex++]);
            }
            else
            {
                LOG_WARNING() << "Invalid format: " << rosparam::LED_LEFT_GAINS << "=" << publishParameterMap.value(key);
            }
        }
        else if(key.startsWith(QString::fromStdString(rosparam::LED_RIGHT_GAINS)))
        {
            auto splits = publishParameterMap.value(key).split(QRegExp("[\\[\\],\\s]+"), QString::SkipEmptyParts);
            if(splits.count() == 3 * 8)
            {
                paramIndex = 0;

                // index=0
                ui->labelRightR->setText(splits[paramIndex++]);
                ui->labelRightG->setText(splits[paramIndex++]);
                ui->labelRightB->setText(splits[paramIndex++]);
                // index=1
                ui->labelRightR_2->setText(splits[paramIndex++]);
                ui->labelRightG_2->setText(splits[paramIndex++]);
                ui->labelRightB_2->setText(splits[paramIndex++]);
                // index=2
                ui->labelRightR_3->setText(splits[paramIndex++]);
                ui->labelRightG_3->setText(splits[paramIndex++]);
                ui->labelRightB_3->setText(splits[paramIndex++]);
                // index=3
                ui->labelRightR_4->setText(splits[paramIndex++]);
                ui->labelRightG_4->setText(splits[paramIndex++]);
                ui->labelRightB_4->setText(splits[paramIndex++]);
                // index=4
                ui->labelRightR_5->setText(splits[paramIndex++]);
                ui->labelRightG_5->setText(splits[paramIndex++]);
                ui->labelRightB_5->setText(splits[paramIndex++]);
                // index=5
                ui->labelRightR_6->setText(splits[paramIndex++]);
                ui->labelRightG_6->setText(splits[paramIndex++]);
                ui->labelRightB_6->setText(splits[paramIndex++]);
                // index=6
                ui->labelRightR_7->setText(splits[paramIndex++]);
                ui->labelRightG_7->setText(splits[paramIndex++]);
                ui->labelRightB_7->setText(splits[paramIndex++]);
                // index=7
                ui->labelRightR_8->setText(splits[paramIndex++]);
                ui->labelRightG_8->setText(splits[paramIndex++]);
                ui->labelRightB_8->setText(splits[paramIndex++]);
            }
            else
            {
                LOG_WARNING() << "Invalid format: " << rosparam::LED_RIGHT_GAINS << "=" << publishParameterMap.value(key);
            }
        }
    }
}
