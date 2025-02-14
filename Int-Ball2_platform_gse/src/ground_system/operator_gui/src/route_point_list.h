#ifndef ROUTE_POINT_LIST_H
#define ROUTE_POINT_LIST_H

#include <QWidget>
#include "route_point_listitem.h"
#include "view/route_information_view.h"

namespace intball
{

namespace Ui
{
class RoutePointList;
}


class RoutePointList : public intball::RouteInformationView
{
    Q_OBJECT
public:
    explicit RoutePointList(QWidget *parent = nullptr);
    ~RoutePointList();

public slots:
    void RoutePointListItem_deleteSelected();
    void RoutePointListItem_selected();
protected:
    virtual void on_dataChanged(const QModelIndex &index, const RoutePoint& data);
    virtual void on_rowsAboutToBeRemoved(int first, int last);
    virtual void on_rowsInserted(int first, int last);
    virtual void on_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected);

private:
    Ui::RoutePointList *ui;
    QVector<intball::RoutePointListItem*> viaPointWidgets_;
    QVector<intball::RoutePointListItem*> visibleListItem_;
    intball::RoutePointListItem* lastSelectedPointListItem_;

    void updateStartPoint();
    bool updateGoalPoint(const int firstToBeRemoved = -1, const int lastToBeRemoved = -1);
    QVector<intball::RoutePointListItem*> updateViaPoints(const int firstToBeRemoved = -1, const int lastToBeRemoved = -1);
    void clearGoal();
    void clearViaPoint(const int viaPointIndex = 0);
    void updateInnerWidgetsSize(const int sizeToBeRemoved = 0);
};
} // namespace intball

#endif // ROUTE_POINT_LIST_H
