#ifndef CAMERA_MICROPHONE_SETTINGS_DIALOG_H
#define CAMERA_MICROPHONE_SETTINGS_DIALOG_H

#include <QDialog>
#include <QLabel>
#include <QScopedPointer>
#include <QToolButton>
#include "camera_config.h"
#include "model/intball_telemetry.h"
#include "telecommand_client.h"

namespace intball
{

namespace Ui {
class CameraMicrophoneSettingsDialog;
}

class CameraMicrophoneSettingsDialog : public QDialog
{
    Q_OBJECT

public:
    static const QString RESOLUTION_KEYWORD_4K;
    explicit CameraMicrophoneSettingsDialog(QWidget *parent = nullptr);
    ~CameraMicrophoneSettingsDialog();

    void initialize(intball::IntBallTelemetry* intballTelemetry, intball::TelecommandClient* telecommandClient);
    void resetValues();

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());

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
    Ui::CameraMicrophoneSettingsDialog *ui;
    QScopedPointer<CameraConfig> cameraConfig_;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::TelecommandClient* telecommandClient_;

    bool isValidInput();
    void setSendButtonStatus();
    void valueChanged(QObject* sender, double arg, QToolButton* clearButton, QLabel* label);
    void checkFrameRateAndResolution();
};

} // namespace intball

#endif // CAMERA_MICROPHONE_SETTINGS_DIALOG_H
