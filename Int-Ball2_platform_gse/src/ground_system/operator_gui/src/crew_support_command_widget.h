#ifndef INTBALL_CREW_SUPPORT_COMMAND_WIDGET_H
#define INTBALL_CREW_SUPPORT_COMMAND_WIDGET_H

#include <QMessageBox>
#include <QWidget>

namespace intball {

namespace Ui {
class CrewSupportCommandWidget;
}

class IntBallTelemetry;
class TelecommandClient;
class CrewSupportCommandWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CrewSupportCommandWidget(QWidget *parent = nullptr);
    ~CrewSupportCommandWidget();
    void initialize(IntBallTelemetry* intballTelemetry,
                    TelecommandClient* telecommandClient);

public slots:
    void on_controlStreamingButton_clicked();

    void on_controlRecordMovieButton_clicked();

    void on_controlSettingMainCameraButton_clicked();

    void on_controlSettingLED_clicked();

    void on_controltoggleMainCamera_clicked();

    void on_controltoggleMicrophone_clicked();

    void on_controltoggleLighting_clicked();

private slots:
    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles);

private:
    Ui::CrewSupportCommandWidget *ui;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::TelecommandClient* telecommandClient_;
};


} // namespace intball
#endif // INTBALL_CREW_SUPPORT_COMMAND_WIDGET_H
