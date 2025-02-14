#ifndef INTBALL_TELECOMMAND_MANAGER_WIDGET_H
#define INTBALL_TELECOMMAND_MANAGER_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandManagerWidget;
}

class TelecommandManagerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandManagerWidget(QWidget *parent = nullptr);
    ~TelecommandManagerWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_buttonExitDockingMode_clicked();

    void on_buttonSetMaintenanceMode_clicked();

    void on_buttonForcedRelease_clicked();

    void on_buttonReboot_clicked();

private:
    Ui::TelecommandManagerWidget *ui;
    TelecommandClient* client_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_MANAGER_WIDGET_H
