#ifndef EDITING_PAGE_H
#define EDITING_PAGE_H

#include <boost/shared_ptr.hpp>
#include <QFuture>
#include <QItemSelection>
#include <QQuaternion>
#include <QVector3D>
#include <QWidget>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include "operator_gui_common.h"

namespace tf
{
class TransformListener;
class StampedTransform;
} // namespace tf

namespace intball
{

namespace Ui
{
class EditingPage;
}

class RoutePointList;
class RouteInformation;
class TelecommandClient;
class EditingPage : public QWidget
{
    Q_OBJECT

public:
    explicit EditingPage(QWidget *parent = nullptr);
    ~EditingPage();
    void initialize(const QString& pathRvizConfig,
                    intball::RouteInformation* routeInformation,
                    QItemSelectionModel* routeInformationSelection,
                    intball::TelecommandClient* telecommandClient
                    );
    void setVideoArea(QWidget* video);
    void setStatusArea(QWidget* status);
    void setCamera();

signals:
    void readyToSwitch(intball::SwitchPageEvent event);

private slots:

    void GoalCameraSimulationPanel_changed();

    void RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles = QVector<int>());

    void RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last);

    void RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last);

    void RouteInformation_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

    void on_editPositionUp_clicked();

    void on_editPositionDown_clicked();

    void on_editPositionLeft_clicked();

    void on_editPositionRight_clicked();

    void on_editPositionFront_clicked();

    void on_editPositionBack_clicked();

    void on_editAttitudeRollCounterClock_clicked();

    void on_editAttitudeRollClock_clicked();

    void on_editAttitudePitchCounterClock_clicked();

    void on_editAttitudePitchClock_clicked();

    void on_editAttitudeYawCounterClock_clicked();

    void on_editAttitudeYawClock_clicked();

    void on_editCancelButton_clicked();

    void on_editSaveButton_clicked();

    void on_spinBoxWaitingTime_valueChanged(int arg1);

    void on_emergencyButton_clicked();

private:
    Ui::EditingPage *ui;
    intball::RoutePointList* routePointListWidget_;
    tf::TransformBroadcaster tfBroadcaster_;
    QFuture<bool> futureTelemetry_;
    intball::RouteInformation* routeInformation_;
    QItemSelectionModel* routeInformationSelectionModel_;
    TelecommandClient* telecommandClient_;

    void callbackUpdateTransform(tf::StampedTransform &transform);
    void checkGoal();
    void publishCameraTf(const QVector3D& position, const QQuaternion& orientation);
    void removeCameraTf();
    void setPointValues(const int index);
    void removePointOperation(const int index);
    void editPositionButtonOperation(const QVector3D& diff);
    void editAttitudeButtonOperationRPY(const float roll, const float pitch, const float yaw);
    void removePoint();
};

} // namespace intball


#endif // EDITING_PAGE_H
