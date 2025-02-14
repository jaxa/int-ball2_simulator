#ifndef INTBALL_STATUS_WIDGET_H
#define INTBALL_STATUS_WIDGET_H

#include <QDateTime>
#include <QMediaPlayer>
#include <QPropertyAnimation>
#include <QTableWidgetItem>
#include <QTimer>
#include <QWidget>
#include "common_log_object.h"
#include "model/dock_telemetry.h"
#include "model/intball_telemetry.h"
#include "telemetry_monitor.h"

namespace intball
{

namespace Ui
{
class StatusWidget;
}

class StatusWidget : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QColor borderColor READ borderColor WRITE setBorderColor)

public:
    static const int BATTERY_MARGIN_RIGHT = 0;
    static const int STORAGE_MARGIN_RIGHT = 70;
    static const int COLUMN_WIDTH_LOG_MESSAGE = 400;
    static const int BLINK_ANIMATION_DURATION_MSECS = 4000;
    static const QColor DEFAULT_FRAME_COLOR;
    static const QString DEFAULE_VALUE_LABEL;

    explicit StatusWidget(QWidget *parent = nullptr);
    ~StatusWidget();

    void initialize(intball::IntBallTelemetry* intballTelemetry,
                    intball::DockTelemetry* dockTelemetry,
                    intball::TelemetryMonitor* telemetryMonitor);

public slots:
    void TelemetryMonitor_detected(TelemetryMonitor::Event event, QVariant value);
    void eventOccurred(CommandLog log);
    QColor borderColor() const;
    void setBorderColor(const QColor color);

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());
    void stopPlayingAlertSound();
    void stopPlayingWarningSound();

    void on_stopBlinkingButton_clicked();

private:
    Ui::StatusWidget *ui;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::DockTelemetry* dockTelemetry_;
    intball::TelemetryMonitor* telemetryMonitor_;
    int maxLogNumber_;
    bool isTooltipEnabled_;
    QMediaPlayer* soundPlayerAlert_;
    int soundPlayerAlertRingingTime_;
    QTimer* soundPlayerAlertStopTimer_;
    QMediaPlayer* soundPlayerWarning_;
    int soundPlayerWarningRingingTime_;
    QTimer* soundPlayerWarningStopTimer_;
    QColor borderColor_;
    QPropertyAnimation* blinkAnimation_;

    void setLabelColor(QWidget* label, const QColor& color);
    void setProgressBarStyleSheet(QWidget* progressBar, const QColor& color, const int marginRight);
    void enableTooltip(QTableWidgetItem& item, const QColor& color = QColor());
    void disableTooltip(QTableWidgetItem& item);
    void changeTextColor(const int rowIndex, const QColor& color);
    void playAlertSound();
    void playWarningSound();
    void startBlinking(const QColor& color);
    void stopBlinking();

};


} // namespace intball
#endif // INTBALL_STATUS_WIDGET_H
