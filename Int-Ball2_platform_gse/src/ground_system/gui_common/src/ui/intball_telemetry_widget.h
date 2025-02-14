#ifndef TELEMETRY_WIDGET_H
#define TELEMETRY_WIDGET_H

#include <QAbstractItemView>
#include "camera_config.h"
#include "model/intball_telemetry.h"
#include "ui/intball_telemetry_header_widget.h"

namespace intball
{

namespace Ui
{
class IntBallTelemetryWidget;
}

enum class TelemetryWidgetGroup {
    HEADER,
    FLIGHT_SOFTWARE,
    ALIVE_STATUS,
    SYSTEM_MONITOR,
    FILE_MONITOR,
    TASK_MANAGER,
    NAVIGATION,
    IMU,
    GUIDANCE_AND_CONTROL,
    WRENCH,
    PROPULSION,
    CAMERA_AND_MICROPHONE,
    LED_DISPLAY,
    RESPONSE_CAMERA_AND_MICROPHONE,
    RESPONSE_PROPULSION,
    RESPONSE_GUIDANCE_AND_CONTROL,
    RESPONSE_TASK_MANAGER,
    RESPONSE_NAVIGATION,
    RESPONSE_IMU,
    RESPONSE_LED_DISPLAY,
    RESPONSE_DISPLAY_MANAGER,
    RESPONSE_ROSPARAMS
};

/**
 * デバッグGUIのウィンドウで、このラベルを用いた表示制御を行う.
 * @see intball::DebugMainWindow::DebugMainWindow
 */
static const QMap<TelemetryWidgetGroup, QString> TELEMETRY_WIDGET_GROUP_LABEL =
{
    {TelemetryWidgetGroup::HEADER,                         "Telemetry header"},
    {TelemetryWidgetGroup::FLIGHT_SOFTWARE,                "Flight Software"},
    {TelemetryWidgetGroup::ALIVE_STATUS,                   "Alive statuses"},
    {TelemetryWidgetGroup::SYSTEM_MONITOR,                 "System monitor"},
    {TelemetryWidgetGroup::FILE_MONITOR,                   "File monitor"},
    {TelemetryWidgetGroup::TASK_MANAGER,                   "Task manager"},
    {TelemetryWidgetGroup::NAVIGATION,                     "Navigation"},
    {TelemetryWidgetGroup::IMU,                            "Imu"},
    {TelemetryWidgetGroup::GUIDANCE_AND_CONTROL,           "Guidance and control"},
    {TelemetryWidgetGroup::WRENCH,                         "Wrench"},
    {TelemetryWidgetGroup::PROPULSION,                     "Propulsion"},
    {TelemetryWidgetGroup::CAMERA_AND_MICROPHONE,          "Camera and microphone"},
    {TelemetryWidgetGroup::LED_DISPLAY,                    "LED Display"},
    {TelemetryWidgetGroup::RESPONSE_CAMERA_AND_MICROPHONE, "Telecommand response - Camera and microphone"},
    {TelemetryWidgetGroup::RESPONSE_PROPULSION,            "Telecommand response - Propulsion"},
    {TelemetryWidgetGroup::RESPONSE_GUIDANCE_AND_CONTROL,  "Telecommand response - Guidance and control"},
    {TelemetryWidgetGroup::RESPONSE_TASK_MANAGER,          "Telecommand response - Task manager"},
    {TelemetryWidgetGroup::RESPONSE_NAVIGATION,            "Telecommand response - Navigation"},
    {TelemetryWidgetGroup::RESPONSE_IMU,                   "Telecommand response - IMU"},
    {TelemetryWidgetGroup::RESPONSE_LED_DISPLAY,           "Telecommand response - LED Display"},
    {TelemetryWidgetGroup::RESPONSE_DISPLAY_MANAGER,       "Telecommand response - Display manager"},
    {TelemetryWidgetGroup::RESPONSE_ROSPARAMS,             "Telecommand response - Get/Set Rosparams"},
};


class IntBallTelemetryWidget : public QAbstractItemView
{
    Q_OBJECT

public:
    explicit IntBallTelemetryWidget(QWidget *parent = nullptr);
    virtual ~IntBallTelemetryWidget() override;

    QRect visualRect(const QModelIndex &index) const override;
    void scrollTo(const QModelIndex &index, ScrollHint hint) override;
    QModelIndex indexAt(const QPoint &point) const override;

    void setVisibility(const TelemetryWidgetGroup& group, const bool on);
protected slots:
    void dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles) override;
    void rowsAboutToBeRemoved(const QModelIndex &parent, int start, int end) override;
    void rowsInserted(const QModelIndex &parent, int start, int end) override;
    void selectionChanged(const QItemSelection &selected, const QItemSelection &deselected) override;
    void currentChanged(const QModelIndex &current, const QModelIndex &previous) override;

protected:
    QModelIndex moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers) override;
    int horizontalOffset() const override;
    int verticalOffset() const override;
    bool isIndexHidden(const QModelIndex &index) const override;
    void setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags command) override;
    QRegion visualRegionForSelection(const QItemSelection &selection) const override;

private:
    Ui::IntBallTelemetryWidget *ui;
    intball::IntBallTelemetry* thisModel();
    QScopedPointer<CameraConfig> cameraConfig_;
    QMap<char, IntBallTelemetryHeaderWidget*> headerWidgets_;
};

}

#endif // TELEMETRY_WIDGET_H
