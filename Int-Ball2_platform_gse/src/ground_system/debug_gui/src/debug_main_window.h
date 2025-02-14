#ifndef DEBUG_MAIN_WINDOW_H
#define DEBUG_MAIN_WINDOW_H

#include <QMainWindow>
#include "telemetry_subscriber.h"
#include "telecommand_log_area_widget.h"
#include "common_log_object.h"

namespace intball
{

namespace Ui
{
class DebugMainWindow;
}

class DebugMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit DebugMainWindow(QWidget *parent = nullptr);
    ~DebugMainWindow();

protected:
    void closeEvent(QCloseEvent *event);

private slots:

    void on_displayTargetWidget_itemSelectionChanged();

    void on_displayTargetWidget_2_itemSelectionChanged();

    void on_displayTargetWidget_3_itemSelectionChanged();

    void on_resetViewButton_clicked();

    void on_tabWidget_currentChanged(int index);

private:
    Ui::DebugMainWindow *ui;
    intball::TelemetrySubscriber* subscriber_;
    intball::IntBallTelemetry* intballTelemetry_;
    intball::DockTelemetry* dockTelemetry_;
    TelecommandLogAreaWidget* logTableWidget_;

    void resetView();
};

} // namespace intball
#endif // DEBUG_MAIN_WINDOW_H
