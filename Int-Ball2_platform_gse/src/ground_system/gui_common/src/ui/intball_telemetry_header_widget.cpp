#include "intball_telemetry_header_widget.h"
#include "ui_intball_telemetry_header_widget.h"

using namespace intball;

IntBallTelemetryHeaderWidget::IntBallTelemetryHeaderWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::IntBallTelemetryHeaderWidget)
{
    ui->setupUi(this);
}

IntBallTelemetryHeaderWidget::~IntBallTelemetryHeaderWidget()
{
    delete ui;
}

void IntBallTelemetryHeaderWidget::setLabelValues(
        const QString timestamp, const QString lastExecutedCommand,
        const QString currentSplitIndex, const QString splitNumber,
        const QString sendingPortIndex)
{
    ui->labelTimestampValue->setText(timestamp);
    ui->labelLastExecutedCommandValue->setText(lastExecutedCommand);
    ui->labelSplitNumberValue->setText(QString("%1 / %2").arg(currentSplitIndex).arg(splitNumber));
    ui->labelSendingPortIndexTitle->setText(QString("Sending Port Index %1").arg(sendingPortIndex));
}
