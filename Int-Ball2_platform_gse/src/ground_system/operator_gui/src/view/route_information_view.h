#ifndef ROUTE_INFORMATION_VIEW_H
#define ROUTE_INFORMATION_VIEW_H

#include <QAbstractItemView>
#include "model/route_information.h"

namespace intball
{

class RouteInformationView : public QAbstractItemView
{
    Q_OBJECT
public:
    explicit RouteInformationView(QWidget *parent = nullptr);
    virtual ~RouteInformationView() override = default;

    QRect visualRect(const QModelIndex &index) const override;
    void scrollTo(const QModelIndex &index, ScrollHint hint) override;
    QModelIndex indexAt(const QPoint &point) const override;

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
    RouteInformation* thisModel();
    virtual void on_dataChanged(const QModelIndex &index, const intball::RoutePoint& data) = 0;
    virtual void on_rowsAboutToBeRemoved(int first, int last) = 0;
    virtual void on_rowsInserted(int first, int last) = 0;
    virtual void on_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected) {};
    virtual void on_currentChanged(const QModelIndex &current, const QModelIndex &previous) {};
};

}
#endif // ROUTE_INFORMATION_VIEW_H
