#ifndef EXECUTION_PAGE_H
#define EXECUTION_PAGE_H

#include <QDateTime>
#include <QFinalState>
#include <QItemSelectionModel>
#include <QStateMachine>
#include <QTimer>
#include <QWidget>
#include "execution_page_statemachine.h"
#include "model/route_point.h"
#include "operator_gui_common.h"
#include "telemetry_monitor.h"

namespace intball
{

namespace Ui
{
class ExecutionPage;
}

class IntBallTelemetry;
class RouteInformation;
class TelecommandClient;
class ExecutionPage : public QWidget
{
    Q_OBJECT

public:
    /**
     * @brief 移動対象とする経路点のインデックス初期値.
     */
    const static int DEFAULT_TARGET_ROUTE_POINT_INDEX = 1;

    explicit ExecutionPage(QWidget *parent = nullptr);
    virtual ~ExecutionPage();
    void initialize(const QString& pathRvizConfig,
                    intball::RouteInformation* routeInformation,
                    intball::TelecommandClient* telecommandClient,
                    intball::IntBallTelemetry* intballTelemetry);
    void setVideoArea(QWidget* video);
    void setStatusArea(QWidget* status);
    void start();
    void stopTimer();
    bool isStarted();

signals:
    void readyToSwitch(intball::SwitchPageEvent event);

public slots:

    void TelemetryMonitor_detected(intball::TelemetryMonitor::Event event, QVariant value);

    /**
     * @brief 外部ウィジットでエラーを検知した際に呼び出す.
     */
    void anomalyDetected();

private slots:

    void RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());

    void RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last);

    void RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last);

    void IntBall2Telemetry_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());


    void onEntryInitialState();
    void onEntryErrorState();

    void onExitErrorState();

    void onEntryNormalOperation();
    void onEntryNormalOperation_InPreparation();
    void onEntryNormalOperation_PostProcess();
    void onExitNormalOperation_PostProcess();

    void onEntryTargetGuidanceControl_Processing();
    void onExitTargetGuidanceControl_Processing();
    void onEntryTargetGuidanceControl_Waiting();
    void onExitTargetGuidanceControl_Waiting();
    void elapsedTimerHandler();
    void waitingTimerHandler();

    void onEntryTargetGuidanceControl_Goal();
    void onEntryPause_Waiting();
    void onEntryPause_Completed();
    void onEntryCancel_Waiting();
    void onEntryCancel_Completed();
    void onEntryEmergencyStop();

    void on_execPauseButton_clicked();

    void on_execCancelButton_clicked();

    void on_emergencyButton_clicked();

private:
    Ui::ExecutionPage *ui;
    intball::RouteInformation* routeInformation_;
    ExecutionPageStateMachine stateMachine_;

    QTimer elapsedTimer_;
    QTimer waitingTimer_;

    TelecommandClient* telecommandClient_;
    IntBallTelemetry* intballTelemetry_;
    TelemetryMonitor* telemetryMonitor_;
    QDateTime startTime_;
    int targetRoutePointIndex_;
    intball::RoutePoint targetRoutePoint_;
    unsigned long elapsedTimeSecond_;
    unsigned long restOfWaitingTime_;
    unsigned long waitingTimeForCurrentMove_;
    unsigned long waitingTimeForCurrentStopping_;
};


} // namespace intball

#endif // EXECUTION_PAGE_H
