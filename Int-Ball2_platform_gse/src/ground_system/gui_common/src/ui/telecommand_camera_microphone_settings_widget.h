#ifndef INTBALL_TELECOMMAND_CAMERA_MICROPHONE_SETTINGS_WIDGET_H
#define INTBALL_TELECOMMAND_CAMERA_MICROPHONE_SETTINGS_WIDGET_H

#include <QLabel>
#include <QWidget>
#include <QToolButton>
#include <QVariant>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandCameraMicrophoneSettingsWidget;
}

class TelecommandCameraMicrophoneSettingsWidget : public QWidget
{
    Q_OBJECT

public:
    static const QString RESOLUTION_KEYWORD_4K;
    explicit TelecommandCameraMicrophoneSettingsWidget(QWidget *parent = nullptr);
    ~TelecommandCameraMicrophoneSettingsWidget();

    void resetValues();
signals:
    void executed(CommandLog log);

private slots:
    void on_toolButtonClearZoom_clicked();

    void on_toolButtonClearResolution_clicked();

    void on_toolButtonClearEV_clicked();

    void on_toolButtonClearWhiteBalance_clicked();

    void on_toolButtonClearCameraGain_clicked();

    void on_toolButtonClearFrameRate_clicked();

    void on_toolButtonClearBitRate_clicked();

    void on_toolButtonClearMicrophoneGain_clicked();

    void on_doubleSpinBoxZoom_valueChanged(double arg1);

    void on_doubleSpinBoxEV_valueChanged(double arg1);

    void on_comboBoxResolution_currentIndexChanged(int index);

    void on_comboBoxWhiteBalance_currentIndexChanged(int index);

    void on_doubleSpinBoxCameraGain_valueChanged(double arg1);

    void on_comboBoxFrameRate_currentIndexChanged(int index);

    void on_comboBoxBitRate_currentIndexChanged(int index);

    void on_doubleSpinBoxMicrophoneGain_valueChanged(double arg1);

    void on_settingsSendButton_clicked();
private:
    Ui::TelecommandCameraMicrophoneSettingsWidget *ui;
    TelecommandClient* client_;
    QScopedPointer<CameraConfig> cameraConfig_;
    bool isValidInput();
    void setSendButtonStatus();
    void valueChanged(QObject* sender, double arg, QToolButton* clearButton, QLabel* label);
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_CAMERA_MICROPHONE_SETTINGS_WIDGET_H
