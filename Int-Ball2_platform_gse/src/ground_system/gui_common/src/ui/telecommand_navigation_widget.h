#ifndef INTBALL_TELECOMMAND_NAVIGATION_WIDGET_H
#define INTBALL_TELECOMMAND_NAVIGATION_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandNavigationWidget;
}

class TelecommandNavigationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandNavigationWidget(QWidget *parent = nullptr);
    ~TelecommandNavigationWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_pushButtonNavigationOn_clicked();

    void on_pushButtonNavigationOff_clicked();

    void on_sendMarkerCorrection_clicked();

private:
    Ui::TelecommandNavigationWidget *ui;
    TelecommandClient* client_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_NAVIGATION_WIDGET_H
