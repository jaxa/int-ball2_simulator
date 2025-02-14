#ifndef INTBALL_TELECOMMAND_DOCK_WIDGET_H
#define INTBALL_TELECOMMAND_DOCK_WIDGET_H

#include <QWidget>
#include "telemetry_telecommand_config.h"
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandDockWidget;
}

class TelecommandDockWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandDockWidget(QWidget *parent = nullptr);
    ~TelecommandDockWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_lineEditlabelGroundSystemIP_textChanged(const QString &arg1);

    void on_dockSendButton_clicked();

    void on_lineEditIntBall2IP_textChanged(const QString &arg1);

    void on_lineEditReceivePort_textChanged(const QString &arg1);

    void on_comboBoxChargeOnOff_currentIndexChanged(int index);

    void on_comboBoxMotorOnOff_currentIndexChanged(int index);

private:
    Ui::TelecommandDockWidget *ui;
    TelecommandClient* client_;
    void disableInput(const dock::telecommand::Index withoutIndex);
    void checkAndEnableInput();
    void checkAndChangeButtonStatus();
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_DOCK_WIDGET_H
