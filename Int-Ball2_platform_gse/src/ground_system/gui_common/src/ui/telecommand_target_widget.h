#ifndef TELECOMMAND_TARGET_WIDGET_H
#define TELECOMMAND_TARGET_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball
{

namespace Ui
{
class TelecommandTargetWidget;
}

class TelecommandTargetWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandTargetWidget(QWidget *parent = nullptr);
    ~TelecommandTargetWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_comboBoxCtlCommandType_currentIndexChanged(const QString &arg1);

    void on_pushButtonSendAction_clicked();

    void on_pushButtonCancelAction_clicked();

private:
    Ui::TelecommandTargetWidget *ui;
    TelecommandClient* client_;
};

}

#endif // TELECOMMAND_TARGET_WIDGET_H
