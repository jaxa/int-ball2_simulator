#include "telecommand_led_settings_widget.h"
#include "ui_telecommand_led_settings_widget.h"
#include <QDoubleValidator>
#include "dialog_factory.h"
#include "gui_common.h"
#include "gui_config_base.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::message;

TelecommandLedSettingsWidget::TelecommandLedSettingsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandLedSettingsWidget),
    minGain_(Config::valueAsFloat(key::KEY_LED_MIN_GAIN))
{
    ui->setupUi(this);
    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandLedSettingsWidget::executed);

    ui->doubleSpinBoxLeftGainsRAll->setMinimum(minGain_);
    ui->doubleSpinBoxLeftGainsGAll->setMinimum(minGain_);
    ui->doubleSpinBoxLeftGainsBAll->setMinimum(minGain_);
    ui->doubleSpinBoxRightGainsRAll->setMinimum(minGain_);
    ui->doubleSpinBoxRightGainsGAll->setMinimum(minGain_);
    ui->doubleSpinBoxRightGainsBAll->setMinimum(minGain_);

    ui->doubleSpinBoxLeft0R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft0G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft0B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft1R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft1G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft1B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft2R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft2G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft2B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft3R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft3G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft3B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft4R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft4G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft4B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft5R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft5G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft5B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft6R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft6G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft6B->setMinimum(minGain_);
    ui->doubleSpinBoxLeft7R->setMinimum(minGain_);
    ui->doubleSpinBoxLeft7G->setMinimum(minGain_);
    ui->doubleSpinBoxLeft7B->setMinimum(minGain_);

    ui->doubleSpinBoxRight0R->setMinimum(minGain_);
    ui->doubleSpinBoxRight0G->setMinimum(minGain_);
    ui->doubleSpinBoxRight0B->setMinimum(minGain_);
    ui->doubleSpinBoxRight1R->setMinimum(minGain_);
    ui->doubleSpinBoxRight1G->setMinimum(minGain_);
    ui->doubleSpinBoxRight1B->setMinimum(minGain_);
    ui->doubleSpinBoxRight2R->setMinimum(minGain_);
    ui->doubleSpinBoxRight2G->setMinimum(minGain_);
    ui->doubleSpinBoxRight2B->setMinimum(minGain_);
    ui->doubleSpinBoxRight3R->setMinimum(minGain_);
    ui->doubleSpinBoxRight3G->setMinimum(minGain_);
    ui->doubleSpinBoxRight3B->setMinimum(minGain_);
    ui->doubleSpinBoxRight4R->setMinimum(minGain_);
    ui->doubleSpinBoxRight4G->setMinimum(minGain_);
    ui->doubleSpinBoxRight4B->setMinimum(minGain_);
    ui->doubleSpinBoxRight5R->setMinimum(minGain_);
    ui->doubleSpinBoxRight5G->setMinimum(minGain_);
    ui->doubleSpinBoxRight5B->setMinimum(minGain_);
    ui->doubleSpinBoxRight6R->setMinimum(minGain_);
    ui->doubleSpinBoxRight6G->setMinimum(minGain_);
    ui->doubleSpinBoxRight6B->setMinimum(minGain_);
    ui->doubleSpinBoxRight7R->setMinimum(minGain_);
    ui->doubleSpinBoxRight7G->setMinimum(minGain_);
    ui->doubleSpinBoxRight7B->setMinimum(minGain_);
}

TelecommandLedSettingsWidget::~TelecommandLedSettingsWidget()
{
    delete ui;
}


void TelecommandLedSettingsWidget::showGainsOnly(const bool flag)
{
    ui->groupLedColors->setVisible(!flag);
    if(flag)
    {
        ui->groupLedGains->setTitle("");
    }
}

void TelecommandLedSettingsWidget::on_buttonSendLeftGains_clicked()
{
    /**
     * @brief ダイアログとして表示している際は,親Widgetにシグナル経由でダイアログの表示・非表示を通知する.
     * @see LedSettingsDialog
     */
    openDialog();
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_LED_GAINS_LEFT))
    {
        closeDialog();
        return;
    }

    QList<RosParam> request;
    QString value = QString("[[%1,%2,%3],[%4,%5,%6],[%7,%8,%9],[%10,%11,%12],[%13,%14,%15],[%16,%17,%18],[%19,%20,%21],[%22,%23,%24]]")
                        .arg(ui->doubleSpinBoxLeft0R->text()).arg(ui->doubleSpinBoxLeft0G->text()).arg(ui->doubleSpinBoxLeft0B->text())
                        .arg(ui->doubleSpinBoxLeft1R->text()).arg(ui->doubleSpinBoxLeft1G->text()).arg(ui->doubleSpinBoxLeft1B->text())
                        .arg(ui->doubleSpinBoxLeft2R->text()).arg(ui->doubleSpinBoxLeft2G->text()).arg(ui->doubleSpinBoxLeft2B->text())
                        .arg(ui->doubleSpinBoxLeft3R->text()).arg(ui->doubleSpinBoxLeft3G->text()).arg(ui->doubleSpinBoxLeft3B->text())
                        .arg(ui->doubleSpinBoxLeft4R->text()).arg(ui->doubleSpinBoxLeft4G->text()).arg(ui->doubleSpinBoxLeft4B->text())
                        .arg(ui->doubleSpinBoxLeft5R->text()).arg(ui->doubleSpinBoxLeft5G->text()).arg(ui->doubleSpinBoxLeft5B->text())
                        .arg(ui->doubleSpinBoxLeft6R->text()).arg(ui->doubleSpinBoxLeft6G->text()).arg(ui->doubleSpinBoxLeft6B->text())
                        .arg(ui->doubleSpinBoxLeft7R->text()).arg(ui->doubleSpinBoxLeft7G->text()).arg(ui->doubleSpinBoxLeft7B->text());
    RosParam set;
    set.id = rosparam::LED_LEFT_GAINS;
    set.value = value.toStdString();
    set.type = RosParamType::LIST;
    request.push_back(set);

    // 処理結果はclientのログおよびsignalで確認する.
    if(client_->sendSetRosParams(request))
    {
        client_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::LED_LEFT);
    }

    closeDialog();
    return;
}

void TelecommandLedSettingsWidget::on_buttonSendRightGains_clicked()
{
    /**
     * @brief ダイアログとして表示している際は,親Widgetにシグナル経由でダイアログの表示・非表示を通知する.
     * @see LedSettingsDialog
     */
    openDialog();
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_LED_GAINS_RIGHT))
    {
        closeDialog();
        return;
    }

    QList<RosParam> request;
    QString value = QString("[[%1,%2,%3],[%4,%5,%6],[%7,%8,%9],[%10,%11,%12],[%13,%14,%15],[%16,%17,%18],[%19,%20,%21],[%22,%23,%24]]")
                        .arg(ui->doubleSpinBoxRight0R->text()).arg(ui->doubleSpinBoxRight0G->text()).arg(ui->doubleSpinBoxRight0B->text())
                        .arg(ui->doubleSpinBoxRight1R->text()).arg(ui->doubleSpinBoxRight1G->text()).arg(ui->doubleSpinBoxRight1B->text())
                        .arg(ui->doubleSpinBoxRight2R->text()).arg(ui->doubleSpinBoxRight2G->text()).arg(ui->doubleSpinBoxRight2B->text())
                        .arg(ui->doubleSpinBoxRight3R->text()).arg(ui->doubleSpinBoxRight3G->text()).arg(ui->doubleSpinBoxRight3B->text())
                        .arg(ui->doubleSpinBoxRight4R->text()).arg(ui->doubleSpinBoxRight4G->text()).arg(ui->doubleSpinBoxRight4B->text())
                        .arg(ui->doubleSpinBoxRight5R->text()).arg(ui->doubleSpinBoxRight5G->text()).arg(ui->doubleSpinBoxRight5B->text())
                        .arg(ui->doubleSpinBoxRight6R->text()).arg(ui->doubleSpinBoxRight6G->text()).arg(ui->doubleSpinBoxRight6B->text())
                        .arg(ui->doubleSpinBoxRight7R->text()).arg(ui->doubleSpinBoxRight7G->text()).arg(ui->doubleSpinBoxRight7B->text());
    RosParam set;
    set.id = rosparam::LED_RIGHT_GAINS;
    set.value = value.toStdString();
    set.type = RosParamType::LIST;
    request.push_back(set);

    // 処理結果はclientのログおよびsignalで確認する.
    if(client_->sendSetRosParams(request))
    {
        client_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::LED_RIGHT);
    }

    closeDialog();
    return;
}

void TelecommandLedSettingsWidget::on_buttonSendLeftColors_clicked()
{
    /**
     * @brief ダイアログとして表示している際は,親Widgetにシグナル経由でダイアログの表示・非表示を通知する.
     * @see LedSettingsDialog
     */
    openDialog();
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_LED_COLOR_LEFT))
    {
        closeDialog();
        return;
    }

    QList<QList<float>> colors;

    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft0R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft0G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft0B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft1R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft1G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft1B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft2R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft2G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft2B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft3R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft3G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft3B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft4R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft4G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft4B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft5R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft5G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft5B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft6R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft6G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft6B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxLeft7R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft7G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxLeft7B_2->value()));
        colors.push_back(color);
    }

    client_->sendLedLeftColors(colors);

    closeDialog();
    return;

}

void TelecommandLedSettingsWidget::on_buttonSendRightColors_clicked()
{
    /**
     * @brief ダイアログとして表示している際は,親Widgetにシグナル経由でダイアログの表示・非表示を通知する.
     * @see LedSettingsDialog
     */
    openDialog();
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_LED_COLOR_RIGHT))
    {
        closeDialog();
        return;
    }

    QList<QList<float>> colors;

    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight0R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight0G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight0B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight1R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight1G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight1B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight2R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight2G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight2B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight3R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight3G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight3B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight4R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight4G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight4B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight5R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight5G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight5B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight6R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight6G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight6B_2->value()));
        colors.push_back(color);
    }
    {
        QList<float> color;
        color.append(static_cast<float>(ui->doubleSpinBoxRight7R_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight7G_2->value()));
        color.append(static_cast<float>(ui->doubleSpinBoxRight7B_2->value()));
        colors.push_back(color);
    }

    client_->sendLedRightColors(colors);

    closeDialog();
    return;
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftRAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftRAll->value();
    ui->doubleSpinBoxLeft0R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7R_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftGAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftGAll->value();
    ui->doubleSpinBoxLeft0G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7G_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftBAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftBAll->value();
    ui->doubleSpinBoxLeft0B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7B_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightRAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightRAll->value();
    ui->doubleSpinBoxRight0R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6R_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7R_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightGAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightGAll->value();
    ui->doubleSpinBoxRight0G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6G_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7G_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightBAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightBAll->value();
    ui->doubleSpinBoxRight0B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6B_2->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7B_2->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftGainsRAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftGainsRAll->value();
    ui->doubleSpinBoxLeft0R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7R->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftGainsGAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftGainsGAll->value();
    ui->doubleSpinBoxLeft0G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7G->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxLeftGainsBAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxLeftGainsBAll->value();
    ui->doubleSpinBoxLeft0B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft1B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft2B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft3B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft4B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft5B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft6B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxLeft7B->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightGainsRAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightGainsRAll->value();
    ui->doubleSpinBoxRight0R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6R->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7R->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightGainsGAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightGainsGAll->value();
    ui->doubleSpinBoxRight0G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6G->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7G->setValue(static_cast<double>(setValue));
}

void TelecommandLedSettingsWidget::on_doubleSpinBoxRightGainsBAll_valueChanged(double arg1)
{
    Q_UNUSED(arg1);
    auto setValue = ui->doubleSpinBoxRightGainsBAll->value();
    ui->doubleSpinBoxRight0B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight1B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight2B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight3B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight4B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight5B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight6B->setValue(static_cast<double>(setValue));
    ui->doubleSpinBoxRight7B->setValue(static_cast<double>(setValue));
}
