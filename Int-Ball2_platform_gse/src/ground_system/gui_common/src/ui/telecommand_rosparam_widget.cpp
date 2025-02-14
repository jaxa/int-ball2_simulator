#include "telecommand_rosparam_widget.h"
#include "ui_telecommand_rosparam_widget.h"
#include "dialog_factory.h"
#include "gui_color.h"
#include "gui_common.h"
#include "gui_config_base.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::message;

TelecommandRosparamWidget::TelecommandRosparamWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandRosparamWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandRosparamWidget::executed);

    ui->checkBoxOnlyDefault->setCheckState(Qt::CheckState::Checked);
    ui->comboBoxDump->insertItem(0,
                                 Config::valueAsString(key::KEY_DEFAULT_ROSPARAM_FILE_PATH),
                                 QVariant(Config::valueAsString(key::KEY_DEFAULT_ROSPARAM_FILE_PATH)));
    ui->comboBoxDump->setCurrentIndex(0);
    ui->comboBoxDump->setEditable(false);
    ui->comboBoxLoad->insertItem(0,
                                 Config::valueAsString(key::KEY_DEFAULT_ROSPARAM_FILE_PATH),
                                 QVariant(Config::valueAsString(key::KEY_DEFAULT_ROSPARAM_FILE_PATH)));
    ui->comboBoxLoad->setCurrentIndex(0);
    ui->comboBoxLoad->setEditable(false);

    ui->comboBoxType->addItem("", QVariant());
    ui->comboBoxType->addItem("String",  QVariant::fromValue(RosParamType::STRING));
    ui->comboBoxType->addItem("Bool",    QVariant::fromValue(RosParamType::BOOL));
    ui->comboBoxType->addItem("Integer", QVariant::fromValue(RosParamType::INTEGER));
    ui->comboBoxType->addItem("Float",   QVariant::fromValue(RosParamType::FLOAT));
    ui->comboBoxType->addItem("List",    QVariant::fromValue(RosParamType::LIST));
    ui->comboBoxType->addItem("Dict",    QVariant::fromValue(RosParamType::DICT));
    ui->comboBoxType->setCurrentIndex(0);

    ui->comboBoxUpdateParameter->addItem("", QVariant());
    ui->comboBoxUpdateParameter->addItem("ctl", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::CTL));
    ui->comboBoxUpdateParameter->addItem("sensor_fusion", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::SENSOR_FUSION));
    ui->comboBoxUpdateParameter->addItem("prop", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::PROP));
    ui->comboBoxUpdateParameter->addItem("imu", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::IMU));
    ui->comboBoxUpdateParameter->addItem("slam_wrapper", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::SLAM_WRAPPER));
    ui->comboBoxUpdateParameter->addItem("camera_and_microphone", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_AND_MICROPHONE));
    ui->comboBoxUpdateParameter->addItem("led_left", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::LED_LEFT));
    ui->comboBoxUpdateParameter->addItem("led_right", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::LED_RIGHT));
    ui->comboBoxUpdateParameter->addItem("display_manager", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::DISPLAY_MANAGER));
    ui->comboBoxUpdateParameter->setCurrentIndex(0);

    ui->comboBoxSendRosparamWithUpdate->addItem("", QVariant());
    ui->comboBoxSendRosparamWithUpdate->addItem("ctl", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::CTL));
    ui->comboBoxSendRosparamWithUpdate->addItem("sensor_fusion", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::SENSOR_FUSION));
    ui->comboBoxSendRosparamWithUpdate->addItem("slam_wrapper", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::SLAM_WRAPPER));
    ui->comboBoxSendRosparamWithUpdate->addItem("prop", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::PROP));
    ui->comboBoxSendRosparamWithUpdate->addItem("imu", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::IMU));
    ui->comboBoxSendRosparamWithUpdate->addItem("camera_and_microphone", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_AND_MICROPHONE));
    ui->comboBoxSendRosparamWithUpdate->addItem("led_left", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::LED_LEFT));
    ui->comboBoxSendRosparamWithUpdate->addItem("led_right", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::LED_RIGHT));
    ui->comboBoxSendRosparamWithUpdate->addItem("display_manager", QVariant::fromValue(telecommand::UPDATE_PARAMETER_TARGET::DISPLAY_MANAGER));
    ui->comboBoxUpdateParameter->setCurrentIndex(0);

    // GetRosparam
    // 初期表示は単一取得
    ui->stackedWidgetGerRosparam->setCurrentIndex(0);
    ui->radioButtonGetRosparamSingle->setChecked(true);
    ui->pushButtonClearGetRosparamKey->hide();
    ui->tableWidgetGetRosparam->setHorizontalHeaderItem(0, new QTableWidgetItem());
    ui->tableWidgetGetRosparam->horizontalHeaderItem(0)->setText("Key");
    for(auto i = 0; i < ui->tableWidgetGetRosparam->rowCount(); ++i)
    {
        ui->tableWidgetGetRosparam->setItem(i, 0, new QTableWidgetItem());
    }

    // ボタン状態の初期化
    setExecuteUpdateButtonStatus();
}

TelecommandRosparamWidget::~TelecommandRosparamWidget()
{
    delete ui;
}

void TelecommandRosparamWidget::setSendRosparamButtonsStatus()
{
    bool isValidParam = isValidSendRosparam();
    ui->pushButtonSendRosparam->setEnabled(isValidParam);
    ui->pushButtonSendRosparamWithUpdate->setEnabled(isValidParam && (ui->comboBoxSendRosparamWithUpdate->currentIndex() != 0));
}

void TelecommandRosparamWidget::setExecuteUpdateButtonStatus()
{
    ui->pushButtonExecuteUpdate->setEnabled(ui->comboBoxUpdateParameter->currentIndex() != 0);
}

void TelecommandRosparamWidget::on_pushButtonDump_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_DUMP_ROSPARAM))
    {
        return;
    }
    client_->sendDumpRosparams(ui->comboBoxDump->currentText());
}

void TelecommandRosparamWidget::on_pushButtonLoad_clicked()
{
    if(!DialogFactory::telecommandCheck(COMMAND_NAME_LOAD_ROSPARAM))
    {
        return;
    }
    client_->sendDumpRosparams(ui->comboBoxLoad->currentText());
}

void TelecommandRosparamWidget::on_checkBoxOnlyDefault_stateChanged(int arg1)
{
    if(arg1 == Qt::CheckState::Checked)
    {
        // ファイルパスにデフォルト値のみを利用する.
        ui->comboBoxDump->setCurrentIndex(0);
        ui->comboBoxDump->setEditable(false);
        ui->comboBoxLoad->setCurrentIndex(0);
        ui->comboBoxLoad->setEditable(false);
    }
    else
    {
        // ファイルパスを編集可能とする.
        ui->comboBoxDump->setEditable(true);
        // Editable編集時にスタイルが反映されない挙動の対応.
        ui->comboBoxDump->setStyleSheet(QString("background-color: %1;color: %2;")
                                        .arg(Color::styleSheetRGB(Color::U3))
                                        .arg(Color::styleSheetRGB(Color::F1)));
        ui->comboBoxLoad->setEditable(true);
        ui->comboBoxLoad->setStyleSheet(QString("background-color: %1;color: %2;")
                                        .arg(Color::styleSheetRGB(Color::U3))
                                        .arg(Color::styleSheetRGB(Color::F1)));
    }
}

bool TelecommandRosparamWidget::isValidSendRosparam()
{
    return !ui->lineEditKey->text().isEmpty() &&
            !ui->lineEditValue->text().isEmpty() &&
            (ui->comboBoxType->currentIndex() != 0);
}

void TelecommandRosparamWidget::on_pushButtonSendRosparam_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_SEND))
    {
        RosParam request;
        request.id = ui->lineEditKey->text().toStdString();
        request.value = ui->lineEditValue->text().toStdString();
        request.type = ui->comboBoxType->currentData().value<RosParamType>();
        client_->sendSetRosParam(request);
    }
}

void TelecommandRosparamWidget::on_comboBoxSendRosparamWithUpdate_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    setSendRosparamButtonsStatus();
}

void TelecommandRosparamWidget::on_comboBoxUpdateParameter_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    setExecuteUpdateButtonStatus();
}

void TelecommandRosparamWidget::on_pushButtonExecuteUpdate_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_RELOAD))
    {
        client_->sendUpdateParameter(ui->comboBoxUpdateParameter->currentData().value<telecommand::UPDATE_PARAMETER_TARGET>());
    }
}

void TelecommandRosparamWidget::on_pushButtonGetRosparam_clicked()
{
    if(ui->stackedWidgetGerRosparam->currentIndex() == 0)
    {
        // GetRosparam単一取得
        if(!ui->lineEditGetRosparam->text().isEmpty() > 0 && DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_GET))
        {
            client_->getRosParam(ui->lineEditGetRosparam->text());
        }
    }
    else
    {
        // GetRosparam複数取得
        QList<QString> keyList;
        for(auto i = 0; i < ui->tableWidgetGetRosparam->rowCount(); ++i)
        {
            if(!ui->tableWidgetGetRosparam->item(i, 0)->text().isEmpty())
            {
                keyList.push_back(ui->tableWidgetGetRosparam->item(i, 0)->text());
            }
        }

        if(keyList.size() > 0 && DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_GET))
        {
            client_->getRosParams(keyList);
        }

    }
}

void TelecommandRosparamWidget::on_pushButtonClearGetRosparamKey_clicked()
{
    for(auto i = 0; i < ui->tableWidgetGetRosparam->rowCount(); ++i)
    {
        ui->tableWidgetGetRosparam->item(i, 0)->setText("");
    }
}

void TelecommandRosparamWidget::on_radioButtonGetRosparamSingle_clicked()
{
    ui->stackedWidgetGerRosparam->setCurrentIndex(0);
    ui->pushButtonClearGetRosparamKey->hide();
}

void TelecommandRosparamWidget::on_radioButtonGetRosparamMultiple_clicked()
{
    ui->stackedWidgetGerRosparam->setCurrentIndex(1);
    ui->pushButtonClearGetRosparamKey->show();
}

void TelecommandRosparamWidget::on_pushButtonSendRosparamWithUpdate_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_SEND_RELOAD))
    {
        // rosparam送信.
        RosParam request;
        request.id = ui->lineEditKey->text().toStdString();
        request.value = ui->lineEditValue->text().toStdString();
        request.type = ui->comboBoxType->currentData().value<RosParamType>();
        client_->sendSetRosParam(request);

        // Update parameter.
        client_->sendUpdateParameter(ui->comboBoxSendRosparamWithUpdate->currentData().value<telecommand::UPDATE_PARAMETER_TARGET>());
    }
}

void TelecommandRosparamWidget::on_lineEditKey_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    setSendRosparamButtonsStatus();
}

void TelecommandRosparamWidget::on_lineEditValue_textChanged(const QString &arg1)
{
    Q_UNUSED(arg1);
    setSendRosparamButtonsStatus();
}

void TelecommandRosparamWidget::on_comboBoxType_currentIndexChanged(int index)
{
    Q_UNUSED(index);
    setSendRosparamButtonsStatus();
}
