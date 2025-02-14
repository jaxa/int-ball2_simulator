#include "telecommand_control_settings_widget.h"
#include "ui_telecommand_control_settings_widget.h"
#include <QIntValidator>
#include <QDoubleValidator>
#include "dialog_factory.h"
#include "gui_common.h"
#include "ros_common.h"

/**
 * @note 現状、画面で利用していないが実装コード自体は残す。
 */


using namespace intball;
using namespace intball::message;

namespace {

template<typename T>
bool doubleValidatorGreaterThanZero(const T* widget)
{
    return (widget->hasAcceptableInput() && widget->text().toDouble() > 0);
}

}

TelecommandControlSettingsWidget::TelecommandControlSettingsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandControlSettingsWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandControlSettingsWidget::executed);

    // バリデーション.

    // 0より大きい整数.
    QIntValidator* intValidatorGreaterThanZero = new QIntValidator(this);
    intValidatorGreaterThanZero->setBottom(1);
    ui->lineEditDurationGoal->setValidator(intValidatorGreaterThanZero);

    // 0以上の小数.
    // 0より大きい判定が必要な場合は追加で値をチェックする.
    QDoubleValidator* doubleValidator = new QDoubleValidator(this);
    doubleValidator->setNotation(QDoubleValidator::StandardNotation);
    doubleValidator->setBottom(0);
    ui->lineEditTolerancePos->setValidator(doubleValidator);
    ui->lineEditToleranceAtt->setValidator(doubleValidator);
    ui->lineEditPosKp->setValidator(doubleValidator);
    ui->lineEditPosKi->setValidator(doubleValidator);
    ui->lineEditPosKd->setValidator(doubleValidator);
    ui->lineEditAttKp->setValidator(doubleValidator);
    ui->lineEditAttKd->setValidator(doubleValidator);
    ui->lineEditQ_position_diag_x->setValidator(doubleValidator);
    ui->lineEditQ_position_diag_v->setValidator(doubleValidator);
    ui->lineEditR_position_diag_x->setValidator(doubleValidator);
    ui->lineEditR_position_diag_v->setValidator(doubleValidator);
    ui->lineEditR_angle_diag->setValidator(doubleValidator);
    ui->lineEditQ_angle_diag_b->setValidator(doubleValidator);
    ui->lineEditQ_angle_diag_q->setValidator(doubleValidator);

    // ボタンの初期状態.
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
    ui->navigationSendButton->setEnabled(validationNavigation());
}

TelecommandControlSettingsWidget::~TelecommandControlSettingsWidget()
{
    delete ui;
}

bool TelecommandControlSettingsWidget::validationGuidanceAndControl()
{
    return doubleValidatorGreaterThanZero(ui->lineEditTolerancePos)||
            doubleValidatorGreaterThanZero(ui->lineEditToleranceAtt) ||
            (ui->lineEditPosKp->hasAcceptableInput()) ||
            (ui->lineEditPosKi->hasAcceptableInput()) ||
            (ui->lineEditPosKd->hasAcceptableInput()) ||
            (ui->lineEditAttKp->hasAcceptableInput()) ||
            (ui->lineEditAttKd->hasAcceptableInput()) ||
            (ui->lineEditDurationGoal->hasAcceptableInput());
}

bool TelecommandControlSettingsWidget::validationNavigation()
{
    return (!ui->lineEditQ_position_diag_x->text().isEmpty()) ||
            (!ui->lineEditQ_position_diag_x->text().isEmpty()) ||
            (!ui->lineEditQ_position_diag_v->text().isEmpty()) ||
            (!ui->lineEditR_position_diag_x->text().isEmpty()) ||
            (!ui->lineEditR_position_diag_v->text().isEmpty()) ||
            (!ui->lineEditQ_angle_diag_b->text().isEmpty()) ||
            (!ui->lineEditQ_angle_diag_q->text().isEmpty()) ||
            (!ui->lineEditR_angle_diag->text().isEmpty());
}

void TelecommandControlSettingsWidget::on_guidanceAndControlSendButton_clicked()
{
    if(!validationGuidanceAndControl())
    {
        return;
    }

    QList<RosParam> request;

    // 変更があったパラメータを送信.
    if(ui->lineEditPosKp->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_POS_KP;
        set.value = ui->lineEditPosKp->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(ui->lineEditPosKi->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_POS_KI;
        set.value = ui->lineEditPosKi->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(ui->lineEditPosKd->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_POS_KD;
        set.value = ui->lineEditPosKd->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(ui->lineEditAttKp->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_POS_KP;
        set.value = ui->lineEditAttKp->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(ui->lineEditAttKd->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_POS_KD;
        set.value = ui->lineEditAttKd->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(ui->lineEditDurationGoal->hasAcceptableInput())
    {
        RosParam set;
        set.id = rosparam::CTL_DURATION_GOAL;
        set.value = ui->lineEditDurationGoal->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(doubleValidatorGreaterThanZero(ui->lineEditTolerancePos))
    {
        RosParam set;
        set.id = rosparam::CTL_TOLERANCE_POS;
        set.value = ui->lineEditTolerancePos->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(doubleValidatorGreaterThanZero(ui->lineEditToleranceAtt))
    {
        RosParam set;
        set.id = rosparam::CTL_TOLERANCE_ATT;
        set.value = ui->lineEditToleranceAtt->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }

    if(request.size() > 0 && DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_SEND_RELOAD))
    {
        if(client_->sendSetRosParams(request))
        {
            // 全パラメータ送信後、updateparameterで更新を通知する
            // パラメータの読み込みを要求.
            client_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::CTL);
        }
    }
}

void TelecommandControlSettingsWidget::on_navigationSendButton_clicked()
{
    if(!validationNavigation())
    {
        return;
    }

    // 変更があったパラメータを送信.
    QList<RosParam> request;

    // 変更があったパラメータを送信.
    if(!ui->lineEditQ_position_diag_x->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_Q_POSITION_DIAG_X;
        set.value = ui->lineEditQ_position_diag_x->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditQ_position_diag_v->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_Q_POSITION_DIAG_V;
        set.value = ui->lineEditQ_position_diag_v->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditR_position_diag_x->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_R_POSITION_DIAG_X;
        set.value = ui->lineEditR_position_diag_x->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditR_position_diag_v->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_R_POSITION_DIAG_V;
        set.value = ui->lineEditR_position_diag_v->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditQ_angle_diag_b->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_Q_ANGLE_DIAG_B;
        set.value = ui->lineEditQ_angle_diag_b->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditQ_angle_diag_q->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_Q_ANGLE_DIAG_Q;
        set.value = ui->lineEditQ_angle_diag_q->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(!ui->lineEditR_angle_diag->text().isEmpty())
    {
        RosParam set;
        set.id = rosparam::NAVIGATION_R_ANGLE_DIAG;
        set.value = ui->lineEditR_angle_diag->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }

    if(request.size() > 0 && DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_SEND_RELOAD))
    {
        if(client_->sendSetRosParams(request))
        {
            // 全パラメータ送信後、updateparameterで更新を通知する
            // パラメータの読み込みを要求.
            client_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::SENSOR_FUSION);
        }
    }
}

void TelecommandControlSettingsWidget::on_lineEditPosKp_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditPosKi_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditPosKd_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditAttKp_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditAttKd_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditDurationGoal_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditTolerancePos_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditToleranceAtt_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->guidanceAndControlSendButton->setEnabled(validationGuidanceAndControl());
}

void TelecommandControlSettingsWidget::on_lineEditQ_position_diag_x_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditQ_position_diag_v_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditR_position_diag_x_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditR_position_diag_v_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditQ_angle_diag_q_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditQ_angle_diag_b_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}

void TelecommandControlSettingsWidget::on_lineEditR_angle_diag_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    ui->navigationSendButton->setEnabled(validationNavigation());
}
