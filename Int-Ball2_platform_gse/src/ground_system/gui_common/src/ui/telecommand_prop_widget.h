#ifndef INTBALL_TELECOMMAND_PROP_WIDGET_H
#define INTBALL_TELECOMMAND_PROP_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandPropWidget;
}

class TelecommandPropWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandPropWidget(QWidget *parent = nullptr);
    ~TelecommandPropWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_pushButtonPropOn_clicked();

    void on_pushButtonPropOff_clicked();

    void on_DutySendButton_clicked();

    void on_doubleSpinBoxAll_valueChanged(double arg1);

private:
    Ui::TelecommandPropWidget *ui;
    TelecommandClient* client_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_PROP_WIDGET_H
