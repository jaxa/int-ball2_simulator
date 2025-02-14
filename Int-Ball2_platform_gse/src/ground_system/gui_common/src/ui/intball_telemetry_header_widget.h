#ifndef INTBALL_TELEMETRY_HEADER_WIDGET_H
#define INTBALL_TELEMETRY_HEADER_WIDGET_H

#include <QWidget>

namespace intball
{

namespace Ui {
class IntBallTelemetryHeaderWidget;
}

class IntBallTelemetryHeaderWidget : public QWidget
{
    Q_OBJECT

public:
    explicit IntBallTelemetryHeaderWidget(QWidget *parent = nullptr);
    ~IntBallTelemetryHeaderWidget();
    void setLabelValues(
            const QString timestamp, const QString lastExecutedCommand,
            const QString currentSplitIndex, const QString splitNumber,
            const QString sendingPortIndex);
private:
    Ui::IntBallTelemetryHeaderWidget *ui;
};

}

#endif // INTBALL_TELEMETRY_HEADER_WIDGET_H
