#ifndef INTBALL_TELECOMMAND_ROSPARAM_WIDGET_H
#define INTBALL_TELECOMMAND_ROSPARAM_WIDGET_H

#include <QWidget>
#include "common_log_object.h"
#include "telecommand_client.h"

namespace intball {

namespace Ui {
class TelecommandRosparamWidget;
}

class TelecommandRosparamWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandRosparamWidget(QWidget *parent = nullptr);
    ~TelecommandRosparamWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_pushButtonDump_clicked();

    void on_pushButtonLoad_clicked();

    void on_checkBoxOnlyDefault_stateChanged(int arg1);

    void on_pushButtonSendRosparam_clicked();

    void on_comboBoxUpdateParameter_currentIndexChanged(int index);

    void on_pushButtonExecuteUpdate_clicked();

    void on_pushButtonGetRosparam_clicked();

    void on_pushButtonClearGetRosparamKey_clicked();

    void on_radioButtonGetRosparamSingle_clicked();

    void on_radioButtonGetRosparamMultiple_clicked();

    void on_comboBoxSendRosparamWithUpdate_currentIndexChanged(int index);

    void on_pushButtonSendRosparamWithUpdate_clicked();

    void on_lineEditKey_textChanged(const QString &arg1);

    void on_lineEditValue_textChanged(const QString &arg1);

    void on_comboBoxType_currentIndexChanged(int index);

private:
    Ui::TelecommandRosparamWidget *ui;
    TelecommandClient* client_;
    void setExecuteUpdateButtonStatus();
    void setSendRosparamButtonsStatus();
    bool isValidSendRosparam();
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_ROSPARAM_WIDGET_H
