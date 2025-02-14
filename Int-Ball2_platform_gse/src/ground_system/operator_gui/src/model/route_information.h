#ifndef ROUTE_INFORMATION_H
#define ROUTE_INFORMATION_H

#include <QAbstractListModel>
#include <QItemSelectionModel>
#include <QMutex>
#include "model/route_point.h"

namespace intball
{

class RouteInformation : public QAbstractListModel
{
    Q_OBJECT
public:
    explicit RouteInformation(QObject* parent);
    virtual ~RouteInformation() override;
    static bool isSelected(const RouteInformation* routeInformation,
                           const QItemSelectionModel* routeInformationSelectionModel,
                           const int row);
    QModelIndex index(int row, int column, const QModelIndex &parent = QModelIndex()) const override;
    QModelIndex parent(const QModelIndex &child) const override;
    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    int columnCount(const QModelIndex &parent = QModelIndex()) const override;
    int count() const;
    QVariant data(const QModelIndex &index, int role = 0) const override;
    RoutePoint data(const int index) const;
    QList<RoutePoint> routeWithoutStartPointAsList() const;
    RoutePoint currentIntBallPose() const;
    RoutePoint goal() const;
    QModelIndex goalModelIndex() const;
    int goalIndex() const;
    bool lastOrLater(const QModelIndex& index);
    bool lastOrLater(const int index);
    bool setData(const QModelIndex &index, const QVariant &value, int role = Qt::EditRole) override;
    bool setData(const int index, const QVariant &value, int role = Qt::EditRole);
    bool setData(const int index, const QVector3D& position, const QQuaternion& orientaion, int role = Qt::EditRole);
    bool setDataPosition(const int index, const QVector3D& position, int role = Qt::EditRole);
    bool setDataPosition(const QModelIndex &index, const QVariant& x, const QVariant& y, const QVariant& z, int role = Qt::EditRole);
    bool setDataPosition(const int index, const QVariant& x, const QVariant& y, const QVariant& z, int role = Qt::EditRole);
    bool setDataOrientation(const QModelIndex &index, const QQuaternion& orientation, int role = Qt::EditRole);
    bool setDataOrientation(const int index, const QQuaternion& orientation, int role = Qt::EditRole);
    bool setDataWaitingTime(const QModelIndex &index, const unsigned int waitingTime, int role = Qt::EditRole);
    bool setDataWaitingTime(const int index, const unsigned int waitingTime, int role = Qt::EditRole);
    bool removeRows(int row, int count, const QModelIndex &parent = QModelIndex()) override;
    bool insertData(int row, const QVector3D& position, const QQuaternion& orientation, const QModelIndex &parent = QModelIndex());
    bool insertData(int row, const QVector3D& position);
    bool pushBack(const RoutePoint& newPoint, const QModelIndex &parent = QModelIndex());

private:
    QMap<QModelIndex, QVariant> data_;
    mutable QMutex* mutex_;
};

}

#endif // ROUTE_INFORMATION_H
