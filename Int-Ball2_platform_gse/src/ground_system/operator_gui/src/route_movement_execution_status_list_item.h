#ifndef ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ITEM_H
#define ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ITEM_H

#include <QLabel>
#include <QPropertyAnimation>
#include <QWidget>
#include "model/route_information.h"

namespace intball {

class RouteMovementExecutionStatusListIcon;
class RouteMovementExecutionStatusListItem : public QWidget
{
    Q_OBJECT
public:
    static const int LABEL_ANIMATION_DURATION_MSECS = 7000;
    RouteMovementExecutionStatusListItem(QWidget *parent = nullptr);

    void assign(const RouteInformation* routeInformation_, const int index);
    void setIconOnly(bool onGoing);
    void clear();
    void setWaitingState(const bool on);
    void setLatest(const bool on);

private:
    RouteMovementExecutionStatusListIcon* labelIcon_;
    QPropertyAnimation* waitLabelAnimation_;
    QLabel* labelPosition_;
    QLabel* labelAttitude_;
    QLabel* labelWait_;
    bool isEnabled;
    bool isStatusImageOnly_;
    bool isStart_;
    bool isGoal_;
    bool onGoing_;
};

} // namespace intball

#endif // ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ITEM_H
