#ifndef INTBALL_ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_H
#define INTBALL_ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_H

#include <QWidget>
#include "model/route_information.h"

namespace intball
{

namespace Ui
{
class RouteMovementExecutionStatusList;
}

class RouteMovementExecutionStatusList : public QWidget
{
    Q_OBJECT

public:
    explicit RouteMovementExecutionStatusList(QWidget *parent = nullptr);
    ~RouteMovementExecutionStatusList();
    void initialize(const RouteInformation* routeInformation);
    void setNext(const int routePointIndex);
    void setWaiting(const bool on);
    void updateLatestPoint();

private:
    Ui::RouteMovementExecutionStatusList *ui;

    int listMax_;
    int currentListLength_;
    int nextPointIndex_;
    bool isWaiting_;
};


} // namespace intball
#endif // INTBALL_ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_H
