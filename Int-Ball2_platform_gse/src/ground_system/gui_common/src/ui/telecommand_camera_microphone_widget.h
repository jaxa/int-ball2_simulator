#ifndef INTBALL_TELECOMMAND_CAMERA_MICROPHONE_WIDGET_H
#define INTBALL_TELECOMMAND_CAMERA_MICROPHONE_WIDGET_H

#include <QWidget>
#include <QVariant>
#include "telecommand_client.h"
#include "common_log_object.h"

namespace intball {

namespace Ui {
class TelecommandCameraMicrophoneWidget;
}

class TelecommandCameraMicrophoneWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TelecommandCameraMicrophoneWidget(QWidget *parent = nullptr);
    ~TelecommandCameraMicrophoneWidget();

signals:
    void executed(CommandLog log);

private slots:
    void on_pushButtonStreamingON_clicked();

    void on_pushButtonStreamingOFF_clicked();

    void on_pushButtonRecordingON_clicked();

    void on_pushButtonRecordingOFF_clicked();

    void on_pushButtonCameraON_clicked();

    void on_pushButtonCameraOFF_clicked();

    void on_pushButtonMicrophoneON_clicked();

    void on_pushButtonMicrophoneOFF_clicked();

private:
    Ui::TelecommandCameraMicrophoneWidget *ui;
    TelecommandClient* client_;
};


} // namespace intball
#endif // INTBALL_TELECOMMAND_CAMERA_MICROPHONE_WIDGET_H
