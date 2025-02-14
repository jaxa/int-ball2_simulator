#include <QPainter>
#include "qdebug_custom.h"
#include "route_information_view.h"
#include "model/route_point.h"

using namespace intball;

RouteInformationView::RouteInformationView(QWidget *parent) : QAbstractItemView(parent)
{
    setSelectionBehavior(QAbstractItemView::SelectItems);
    setSelectionMode(QAbstractItemView::SingleSelection);
}


QRect RouteInformationView::visualRect(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return QRect(0, 0, width(), height());
}

void RouteInformationView::scrollTo(const QModelIndex &index, ScrollHint hint)
{
    Q_UNUSED(index);
    Q_UNUSED(hint);
}

QModelIndex RouteInformationView::indexAt(const QPoint &point) const
{
    Q_UNUSED(point);
    return QModelIndex();
}

QModelIndex RouteInformationView::moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers)
{
    Q_UNUSED(cursorAction);
    Q_UNUSED(modifiers);
    return QModelIndex();
}

int RouteInformationView::horizontalOffset() const
{
    return 0;
}

int RouteInformationView::verticalOffset() const
{
    return 0;
}

bool RouteInformationView::isIndexHidden(const QModelIndex &index) const
{
    Q_UNUSED(index)
    return false;
}

void RouteInformationView::setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags command)
{
    Q_UNUSED(rect);
    Q_UNUSED(command);
}

QRegion RouteInformationView::visualRegionForSelection(const QItemSelection &selection) const
{
    Q_UNUSED(selection);
    return QRegion();
}

RouteInformation* RouteInformationView::thisModel()
{
    Q_ASSERT(dynamic_cast<RouteInformation*>(model()) != nullptr);
    return dynamic_cast<RouteInformation*>(model());
}

void RouteInformationView::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);
    on_dataChanged(topLeft, model()->data(topLeft).value<RoutePoint>());
}

void RouteInformationView::rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    on_rowsAboutToBeRemoved(first, last);
}

void RouteInformationView::rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    on_rowsInserted(first, last);
}

void RouteInformationView::selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    on_selectionChanged(selected, deselected);
}

void RouteInformationView::currentChanged(const QModelIndex &current, const QModelIndex &previous)
{
    on_currentChanged(current, previous);
}
