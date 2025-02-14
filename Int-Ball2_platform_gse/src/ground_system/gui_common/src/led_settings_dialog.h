#ifndef INTBALL_LED_SETTINGS_DIALOG_H
#define INTBALL_LED_SETTINGS_DIALOG_H

#include <QDialog>
#include "model/intball_telemetry.h"
#include "telecommand_client.h"

namespace intball {

namespace Ui {
class LedSettingsDialog;
}

class LedSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LedSettingsDialog(QWidget *parent = nullptr);
    ~LedSettingsDialog();

    void initialize(intball::IntBallTelemetry* intballTelemetry, intball::TelecommandClient* telecommandClient);
    void showGainsOnly(const bool flag);

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());

private:
    Ui::LedSettingsDialog *ui;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::TelecommandClient* telecommandClient_;
};


} // namespace intball
#endif // INTBALL_LED_SETTINGS_DIALOG_H
