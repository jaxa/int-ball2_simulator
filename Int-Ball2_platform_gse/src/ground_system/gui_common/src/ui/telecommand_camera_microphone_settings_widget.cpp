#include "telecommand_camera_microphone_settings_widget.h"
#include "ui_telecommand_camera_microphone_settings_widget.h"
#include <QDoubleSpinBox>
#include "camera_config.h"
#include "dialog_factory.h"
#include "gui_common.h"
#include "gui_color.h"
#include "gui_config_base.h"
#include "ib2_msgs.h"
#include "ros_common.h"
#include "telemetry_telecommand_config.h"
#include "utils.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::message;

namespace {

bool isValid(const QDoubleSpinBox* ui)
{
    return ui->isEnabled() && !comparisonFloat(static_cast<float>(ui->value()), static_cast<float>(ui->minimum()));
}

bool isValid(const QComboBox* ui)
{
    return ui->isEnabled() && (ui->currentIndex() > 0);
}

}

const QString TelecommandCameraMicrophoneSettingsWidget::RESOLUTION_KEYWORD_4K = "Four_K";

TelecommandCameraMicrophoneSettingsWidget::TelecommandCameraMicrophoneSettingsWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TelecommandCameraMicrophoneSettingsWidget)
{
    ui->setupUi(this);

    client_ = new TelecommandClient(*getNodeHandle(), this);
    connect(client_, &TelecommandClient::executed, this, &TelecommandCameraMicrophoneSettingsWidget::executed);

    // 画質設定の選択値を定義.
    // カメラ用設定ファイルから読みだした内容を反映する.
    cameraConfig_.reset(new CameraConfig);
    auto cameraConfigMap = cameraConfig_->getAllQualitySettings();
    ui->comboBoxResolution->addItem("(No change)",  QVariant());
    for(auto i = cameraConfigMap.keyBegin(); i != cameraConfigMap.keyEnd(); ++i)
    {
        ui->comboBoxResolution->addItem(cameraConfig_->getQualitySettingAsStringById(*i),
                                        QVariant::fromValue(cameraConfigMap.value(*i).id));
    }
    ui->comboBoxResolution->setCurrentIndex(0);

    // ホワイトバランスの定義.
    ui->comboBoxWhiteBalance->addItem("(No change)",  QVariant(static_cast<uint8_t>(0)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::AUTO),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::AUTO)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::INCANDESCENT),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::INCANDESCENT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::FLUORESCENT),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::FLUORESCENT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::WARM_FLUORESCENT),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::WARM_FLUORESCENT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::DAYLIGHT),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::DAYLIGHT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::CLOUDY_DAYLIGHT)),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::CLOUDY_DAYLIGHT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::TWILIGHT)),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::TWILIGHT)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::SHADE)),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::SHADE)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::MANUAL)),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::MANUAL)));
    ui->comboBoxWhiteBalance->addItem(getWhiteBalanceModeAsString(ib2_msgs::MainCameraWhiteBalanceMode::OFF),
                                      QVariant::fromValue(static_cast<uint8_t>(ib2_msgs::MainCameraWhiteBalanceMode::OFF)));
    ui->comboBoxWhiteBalance->setCurrentIndex(0);

    // ビットレートの定義を設定.
    auto bitrateList = Config::valueAsStringList(key::KEY_CAMERA_BITRATE_LIST);
    ui->comboBoxBitRate->addItem("(No change)",  QVariant());
    for(auto i = bitrateList.begin(); i != bitrateList.end(); ++i)
    {
        auto value = (*i).simplified();
        ui->comboBoxBitRate->addItem(value, value.toULongLong());
    }
    ui->comboBoxBitRate->setCurrentIndex(0);

    // FPSの定義.
    ui->comboBoxFrameRate->addItem("(No change)",  QVariant());
    ui->comboBoxFrameRate->addItem("5",  5);
    ui->comboBoxFrameRate->addItem("10", 10);
    ui->comboBoxFrameRate->addItem("15", 15);
    ui->comboBoxFrameRate->addItem("20(HD/Full HD)", 20);
    ui->comboBoxFrameRate->addItem("25(HD/Full HD)", 25);
    ui->comboBoxFrameRate->addItem("30(HD/Full HD)", 30);
    ui->comboBoxFrameRate->setCurrentIndex(0);

    // 送信ボタン状態.
    ui->settingsSendButton->setEnabled(isValidInput());

    // 消去ボタン.
    ui->toolButtonClearZoom->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearZoom->hide();
    ui->toolButtonClearEV->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearEV->hide();
    ui->toolButtonClearCameraGain->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearCameraGain->hide();
    ui->toolButtonClearWhiteBalance->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearWhiteBalance->hide();
    ui->toolButtonClearFrameRate->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearFrameRate->hide();
    ui->toolButtonClearBitRate->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearBitRate->hide();
    ui->toolButtonClearResolution->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearResolution->hide();
    ui->toolButtonClearMicrophoneGain->setIcon(QApplication::style()->standardIcon( QStyle::SP_DialogCancelButton ));
    ui->toolButtonClearMicrophoneGain->hide();
}

TelecommandCameraMicrophoneSettingsWidget::~TelecommandCameraMicrophoneSettingsWidget()
{
    delete ui;
}

bool TelecommandCameraMicrophoneSettingsWidget::isValidInput()
{
    return isValid(ui->doubleSpinBoxEV) ||
            isValid(ui->doubleSpinBoxZoom) ||
            isValid(ui->doubleSpinBoxCameraGain) ||
            isValid(ui->comboBoxFrameRate) ||
            isValid(ui->comboBoxWhiteBalance) ||
            isValid(ui->comboBoxResolution) ||
            isValid(ui->comboBoxBitRate) ||
            isValid(ui->doubleSpinBoxMicrophoneGain);
}

void TelecommandCameraMicrophoneSettingsWidget::valueChanged(QObject* sender, double arg, QToolButton* clearButton, QLabel* label)
{
    auto checkQDoubleSpinBox = dynamic_cast<QDoubleSpinBox*>(sender);
    auto checkComboBox = dynamic_cast<QComboBox*>(sender);

    bool flag = false;
    if(checkQDoubleSpinBox != nullptr)
    {
        flag = comparisonFloat(static_cast<float>(arg), static_cast<float>(checkQDoubleSpinBox->minimum()));
    }
    else if(checkComboBox != nullptr)
    {
        flag = static_cast<int>(arg) == 0;
    }
    else
    {
        Q_ASSERT(true);
    }

    if(flag)
    {
        // 値を変更しない場合.
        clearButton->hide();
        label->setStyleSheet("");
    }
    else
    {
        // 値を変更する（入力値が存在する）場合.
        clearButton->show();
        label->setStyleSheet(QString("color: %1;").arg(Color::styleSheetRGB(Color::U_HIGHLIGHT)));
    }

    ui->settingsSendButton->setEnabled(isValidInput());
}

void TelecommandCameraMicrophoneSettingsWidget::resetValues()
{

    ui->doubleSpinBoxZoom->setValue(ui->doubleSpinBoxZoom->minimum());
    ui->doubleSpinBoxEV->setValue(ui->doubleSpinBoxEV->minimum());
    ui->comboBoxWhiteBalance->setCurrentIndex(0);
    ui->comboBoxResolution->setCurrentIndex(0);
    ui->comboBoxFrameRate->setCurrentIndex(0);
    ui->doubleSpinBoxCameraGain->setValue(ui->doubleSpinBoxCameraGain->minimum());
    ui->comboBoxBitRate->setCurrentIndex(0);
    ui->doubleSpinBoxMicrophoneGain->setValue(ui->doubleSpinBoxMicrophoneGain->minimum());
}


void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearZoom_clicked()
{
    ui->doubleSpinBoxZoom->setValue(ui->doubleSpinBoxZoom->minimum());
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearResolution_clicked()
{
    ui->comboBoxResolution->setCurrentIndex(0);
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearEV_clicked()
{
    ui->doubleSpinBoxEV->setValue(ui->doubleSpinBoxEV->minimum());
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearWhiteBalance_clicked()
{
    ui->comboBoxWhiteBalance->setCurrentIndex(0);
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearCameraGain_clicked()
{
    ui->doubleSpinBoxCameraGain->setValue(ui->doubleSpinBoxCameraGain->minimum());
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearFrameRate_clicked()
{
    ui->comboBoxFrameRate->setCurrentIndex(0);
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearBitRate_clicked()
{
    ui->comboBoxBitRate->setCurrentIndex(0);
}

void TelecommandCameraMicrophoneSettingsWidget::on_toolButtonClearMicrophoneGain_clicked()
{
    ui->doubleSpinBoxMicrophoneGain->setValue(ui->doubleSpinBoxMicrophoneGain->minimum());
}

void TelecommandCameraMicrophoneSettingsWidget::on_doubleSpinBoxZoom_valueChanged(double arg1)
{
    valueChanged(sender(), arg1, ui->toolButtonClearZoom, ui->labelZoom);
}


void TelecommandCameraMicrophoneSettingsWidget::on_doubleSpinBoxEV_valueChanged(double arg1)
{
    valueChanged(sender(), arg1, ui->toolButtonClearEV, ui->labelEV);
}

void TelecommandCameraMicrophoneSettingsWidget::on_comboBoxResolution_currentIndexChanged(int index)
{
    valueChanged(sender(), index, ui->toolButtonClearResolution, ui->labelResolution);
}


void TelecommandCameraMicrophoneSettingsWidget::on_comboBoxWhiteBalance_currentIndexChanged(int index)
{
    valueChanged(sender(), index, ui->toolButtonClearWhiteBalance, ui->labelWhiteBalance);
}

void TelecommandCameraMicrophoneSettingsWidget::on_doubleSpinBoxCameraGain_valueChanged(double arg1)
{
    valueChanged(sender(), arg1, ui->toolButtonClearCameraGain, ui->labelCameraGain);
}

void TelecommandCameraMicrophoneSettingsWidget::on_comboBoxFrameRate_currentIndexChanged(int index)
{
    if(ui->comboBoxFrameRate->currentData().toInt() > 15)
    {
        if(ui->comboBoxResolution->currentIndex() > 0)
        {
            // 解像度の変更が指定されていた場合,変更後解像度が4Kの場合はFPS15が最大値
            if(ui->comboBoxResolution->currentText().contains(RESOLUTION_KEYWORD_4K))
            {
                // 指定値がFPS15を超えていた場合は選択を変更する.
                ui->comboBoxFrameRate->setCurrentText("15");
            }
        }
    }

    valueChanged(sender(), index, ui->toolButtonClearFrameRate, ui->labelFrameRate);
}

void TelecommandCameraMicrophoneSettingsWidget::on_comboBoxBitRate_currentIndexChanged(int index)
{
    valueChanged(sender(), index, ui->toolButtonClearBitRate, ui->labelBitRate);
}

void TelecommandCameraMicrophoneSettingsWidget::on_doubleSpinBoxMicrophoneGain_valueChanged(double arg1)
{
    if(comparisonFloat(static_cast<float>(arg1), static_cast<float>(dynamic_cast<QDoubleSpinBox*>(sender())->minimum())))
    {
        ui->toolButtonClearMicrophoneGain->hide();
    }
    else
    {
        ui->toolButtonClearMicrophoneGain->show();
    }

    ui->settingsSendButton->setEnabled(isValidInput());
    valueChanged(sender(), arg1, ui->toolButtonClearMicrophoneGain, ui->labelMicGain);
}

void TelecommandCameraMicrophoneSettingsWidget::on_settingsSendButton_clicked()
{
    QList<RosParam> request;

    if(isValid(ui->doubleSpinBoxEV))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_EV;
        set.value = ui->doubleSpinBoxEV->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(isValid(ui->doubleSpinBoxZoom))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_ZOOM;
        set.value = ui->doubleSpinBoxZoom->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(isValid(ui->doubleSpinBoxCameraGain))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_GAIN;
        set.value = ui->doubleSpinBoxCameraGain->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }
    if(isValid(ui->comboBoxFrameRate))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_FRAME_RATE;
        set.value = QString::number(ui->comboBoxFrameRate->currentData().toInt()).toStdString();
        set.type = RosParamType::INTEGER;
        request.push_back(set);
    }
    if(isValid(ui->comboBoxWhiteBalance))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_WHITEBALANCE;
        set.value = QString::number(ui->comboBoxWhiteBalance->currentData().toInt()).toStdString();
        set.type = RosParamType::INTEGER;
        request.push_back(set);
    }
    if(isValid(ui->comboBoxResolution))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_RESOLUTION;
        set.value = QString::number(ui->comboBoxResolution->currentData().toInt()).toStdString();
        set.type = RosParamType::INTEGER;
        request.push_back(set);
    }
    if(isValid(ui->comboBoxBitRate))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_CAMERA_BIT_RATE;
        set.value = QString::number(ui->comboBoxBitRate->currentData().toInt()).toStdString();
        set.type = RosParamType::INTEGER;
        request.push_back(set);
    }
    if(isValid(ui->doubleSpinBoxMicrophoneGain))
    {
        RosParam set;
        set.id = rosparam::CAMERA_ANC_MICROPHONE_MICROPHONE_GAIN;
        set.value = ui->doubleSpinBoxMicrophoneGain->text().toStdString();
        set.type = RosParamType::FLOAT;
        request.push_back(set);
    }

    if(request.size() > 0)
    {
        if(!DialogFactory::telecommandCheck(COMMAND_NAME_PARAMETER_SEND_RELOAD))
        {
            return;
        }
        if(client_->sendSetRosParams(request))
        {
            // Rosparam値送信完了後は入力をクリアする.
            ui->doubleSpinBoxEV->setValue(ui->doubleSpinBoxEV->minimum());
            ui->doubleSpinBoxZoom->setValue(ui->doubleSpinBoxZoom->minimum());
            ui->doubleSpinBoxCameraGain->setValue(ui->doubleSpinBoxCameraGain->minimum());
            ui->comboBoxFrameRate->setCurrentIndex(0);
            ui->comboBoxWhiteBalance->setCurrentIndex(0);
            ui->comboBoxResolution->setCurrentIndex(0);
            ui->comboBoxBitRate->setCurrentIndex(0);
            ui->doubleSpinBoxMicrophoneGain->setValue(ui->doubleSpinBoxMicrophoneGain->minimum());

            // 全パラメータ送信後、updateparameterで更新を通知する
           if(!client_->sendUpdateParameter(telecommand::UPDATE_PARAMETER_TARGET::CAMERA_AND_MICROPHONE))
           {
               // 送信失敗.
               DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
           }
        }
    }
}
