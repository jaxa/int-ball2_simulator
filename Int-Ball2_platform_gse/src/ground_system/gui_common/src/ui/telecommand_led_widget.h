#ifndef INTBALL_TELECOMMAND_LED_WIDGET_H
#define INTBALL_TELECOMMAND_LED_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandLedWidget;
}

class TelecommandLedWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandLedWidget(QWidget *parent = nullptr);
    ~TelecommandLedWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_buttonDisplayManagementOn_clicked();

    void on_buttonDisplayManagementOff_clicked();

    void on_buttonLightingOn_clicked();

    void on_buttonLightingOff_clicked();

private:
    Ui::TelecommandLedWidget *ui;
    TelecommandClient* client_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_LED_WIDGET_H
