#ifndef INTBALL_TELECOMMAND_LED_SETTINGS_WIDGET_H
#define INTBALL_TELECOMMAND_LED_SETTINGS_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandLedSettingsWidget;
}

class TelecommandLedSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandLedSettingsWidget(QWidget *parent = nullptr);
    ~TelecommandLedSettingsWidget();

    void showGainsOnly(const bool flag);

signals:
    void executed(CommandLog log);
    void openDialog();
    void closeDialog();

private slots:
    void on_buttonSendLeftGains_clicked();

    void on_buttonSendRightGains_clicked();

    void on_buttonSendLeftColors_clicked();

    void on_buttonSendRightColors_clicked();

    void on_doubleSpinBoxLeftRAll_valueChanged(double arg1);

    void on_doubleSpinBoxLeftGAll_valueChanged(double arg1);

    void on_doubleSpinBoxLeftBAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightRAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightGAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightBAll_valueChanged(double arg1);

    void on_doubleSpinBoxLeftGainsRAll_valueChanged(double arg1);

    void on_doubleSpinBoxLeftGainsGAll_valueChanged(double arg1);

    void on_doubleSpinBoxLeftGainsBAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightGainsRAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightGainsGAll_valueChanged(double arg1);

    void on_doubleSpinBoxRightGainsBAll_valueChanged(double arg1);

private:
    Ui::TelecommandLedSettingsWidget *ui;
    TelecommandClient* client_;
    float minGain_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_LED_SETTINGS_WIDGET_H
