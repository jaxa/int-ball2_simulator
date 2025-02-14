#ifndef TELECOMMAND_CONTROL_SETTINGS_WIDGET_H
#define TELECOMMAND_CONTROL_SETTINGS_WIDGET_H

#include <QWidget>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball
{

namespace Ui {
class TelecommandControlSettingsWidget;
}

class TelecommandControlSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandControlSettingsWidget(QWidget *parent = nullptr);
    ~TelecommandControlSettingsWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_guidanceAndControlSendButton_clicked();

    void on_navigationSendButton_clicked();

    void on_lineEditPosKp_textChanged(const QString &arg1);

    void on_lineEditPosKi_textChanged(const QString &arg1);

    void on_lineEditPosKd_textChanged(const QString &arg1);

    void on_lineEditAttKp_textChanged(const QString &arg1);

    void on_lineEditAttKd_textChanged(const QString &arg1);

    void on_lineEditDurationGoal_textChanged(const QString &arg1);

    void on_lineEditTolerancePos_textChanged(const QString &arg1);

    void on_lineEditToleranceAtt_textChanged(const QString &arg1);

    void on_lineEditQ_position_diag_x_textChanged(const QString &arg1);

    void on_lineEditQ_position_diag_v_textChanged(const QString &arg1);

    void on_lineEditR_position_diag_x_textChanged(const QString &arg1);

    void on_lineEditR_position_diag_v_textChanged(const QString &arg1);

    void on_lineEditQ_angle_diag_q_textChanged(const QString &arg1);

    void on_lineEditQ_angle_diag_b_textChanged(const QString &arg1);

    void on_lineEditR_angle_diag_textChanged(const QString &arg1);

private:
    Ui::TelecommandControlSettingsWidget *ui;
    TelecommandClient* client_;
    bool validationGuidanceAndControl();
    bool validationNavigation();
};

} // namespace intball

#endif // TELECOMMAND_CONTROL_SETTINGS_WIDGET_H
