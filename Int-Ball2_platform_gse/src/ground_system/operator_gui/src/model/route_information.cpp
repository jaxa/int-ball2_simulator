#include <QMutexLocker>
#include <QQuaternion>
#include <QVector3D>
#include <QtAlgorithms>
#include "qdebug_custom.h"
#include "route_information.h"
#include "route_point.h"

using namespace intball;

RouteInformation::RouteInformation(QObject* parent) :
    QAbstractListModel(parent), mutex_(new QMutex(QMutex::Recursive))
{
}

RouteInformation::~RouteInformation()
{
    delete mutex_;
}

bool RouteInformation::isSelected(const RouteInformation* routeInformation,
                                  const QItemSelectionModel* routeInformationSelectionModel,
                                  const int row)
{
    return routeInformationSelectionModel->selectedRows().indexOf(routeInformation->index(row, 0)) >= 0;
}

QModelIndex RouteInformation::index(int row, int column, const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return createIndex(row, column);
}

QModelIndex RouteInformation::parent(const QModelIndex &child) const
{
    Q_UNUSED(child);
    return QModelIndex();
}

int RouteInformation::rowCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    QMutexLocker locker(mutex_);
    return data_.size();
}

int RouteInformation::columnCount(const QModelIndex &parent) const
{
    Q_UNUSED(parent);
    return 1;
}

int RouteInformation::count() const
{
    return rowCount();
}

QVariant RouteInformation::data(const QModelIndex &index, int role) const
{
    Q_UNUSED(role);
    QMutexLocker locker(mutex_);

    if (index.isValid())
    {
        return data_[index];
    }
    return QVariant();
}

RoutePoint RouteInformation::data(const int index) const
{
    return data(createIndex(index, 0)).value<RoutePoint>();
}

QList<RoutePoint> RouteInformation::routeWithoutStartPointAsList() const
{
    QList<RoutePoint> returnList;
    for(int i = 1; i < data_.size(); ++i)
    {
        returnList << data_[createIndex(i, 0)].value<RoutePoint>();
    }

    return returnList;
}

RoutePoint RouteInformation::currentIntBallPose() const
{
    QMutexLocker locker(mutex_);
    if(data_.size() > 0)
    {
        return data_.first().value<RoutePoint>();
    }
    else
    {
        return RoutePoint();
    }
}

RoutePoint RouteInformation::goal() const
{
    QMutexLocker locker(mutex_);
    if(data_.size() > 0)
    {
        return data_.last().value<RoutePoint>();
    }
    else
    {
        return RoutePoint();
    }
}

QModelIndex RouteInformation::goalModelIndex() const
{
    QMutexLocker locker(mutex_);
    if(data_.size() > 0)
    {
        return data_.lastKey();
    }
    else
    {
        return QModelIndex();
    }
}

int RouteInformation::goalIndex() const
{
    QMutexLocker locker(mutex_);
    if(data_.size() > 0)
    {
        return data_.lastKey().row();
    }
    else
    {
        return -1;
    }
}

bool RouteInformation::lastOrLater(const QModelIndex& index)
{
    QMutexLocker locker(mutex_);
    return (data_.lastKey() == index) || (data_.lastKey() < index);
}

bool RouteInformation::lastOrLater(const int index)
{
    QMutexLocker locker(mutex_);
    QModelIndex check = createIndex(index, 0);
    return (data_.lastKey() == check) || (data_.lastKey() < check);
}

bool RouteInformation::setData(const QModelIndex &index, const QVariant &value, int role)
{
    Q_UNUSED(role);
    QMutexLocker locker(mutex_);

    if (index.isValid())
    {
        data_[index] = value;
        emit dataChanged(index, index);
        return true;
    }
    return false;
}

bool RouteInformation::setData(const int index, const QVariant &value, int role)
{
    return setData(createIndex(index, 0), value, role);
}

bool RouteInformation::setData(const int index, const QVector3D& position, const QQuaternion& orientaion, int role)
{
    return setData(createIndex(index, 0), QVariant::fromValue(RoutePoint(position, orientaion)), role);
}

bool RouteInformation::setDataPosition(const QModelIndex &index, const QVariant& x, const QVariant& y, const QVariant& z, int role)
{
    Q_UNUSED(role);
    QMutexLocker locker(mutex_);

    if(index.isValid())
    {
        auto target = data_[index].value<RoutePoint>();
        QVector3D targetPosition = target.position();
        if(x.isValid())
        {
            targetPosition.setX(x.toFloat());
        }
        if(y.isValid())
        {
            targetPosition.setY(y.toFloat());
        }
        if(z.isValid())
        {
            targetPosition.setZ(z.toFloat());
        }

        data_[index] = QVariant::fromValue(RoutePoint(targetPosition, target.orientation()));
        emit dataChanged(index, index);
        return true;
    }

    return false;
}

bool RouteInformation::setDataPosition(const int index, const QVariant& x, const QVariant& y, const QVariant& z, int role)
{
    return setDataPosition(createIndex(index, 0), x, y, z, role);
}

bool RouteInformation::setDataPosition(const int index, const QVector3D& position, int role)
{
    return setDataPosition(createIndex(index, 0), position.x(), position.y(), position.z(), role);
}

bool RouteInformation::setDataOrientation(const QModelIndex& index, const QQuaternion& orientation, int role)
{
    Q_UNUSED(role);
    INFO_START_FUNCTION() << "index=" << index << " orientation=" << orientation << " role=" << role;
    QMutexLocker locker(mutex_);

    if(index.isValid())
    {
        auto target = data_[index].value<RoutePoint>();
        target.setData(orientation);
        data_[index] = QVariant::fromValue(target);
        emit dataChanged(index, index);
        return true;
    }

    return false;
}

bool RouteInformation::setDataWaitingTime(const QModelIndex &index, const unsigned int waitingTime, int role)
{
    Q_UNUSED(role);
    INFO_START_FUNCTION() << "index=" << index << " waitingTime=" << waitingTime;
    QMutexLocker locker(mutex_);

    if(index.isValid())
    {
        auto target = data_[index].value<RoutePoint>();
        target.setData(waitingTime);
        data_[index] = QVariant::fromValue(target);
        emit dataChanged(index, index);
        return true;
    }

    return false;
}

bool RouteInformation::setDataWaitingTime(const int index, const unsigned int waitingTime, int role)
{
    return setDataWaitingTime(createIndex(index, 0), waitingTime, role);
}

bool RouteInformation::setDataOrientation(const int index, const QQuaternion& orientation, int role)
{
    return setDataOrientation(createIndex(index, 0), orientation, role);
}

bool RouteInformation::removeRows(int row, int count, const QModelIndex &parent)
{
    INFO_START_FUNCTION() << "row=" << row << " count=" << count << " parent=" << parent;
    QMutexLocker locker(mutex_);

    const int removeStartRow = row;
    const int removeEndRow = row + count - 1;
    if((removeStartRow >= 0) && (removeStartRow < data_.size()) && (removeEndRow >= removeStartRow))
    {
        beginRemoveRows(parent, row,  removeEndRow);

        QMap<QModelIndex, QVariant> newData;
        int newInsertIndex = 0;
        for(auto i = data_.begin(); i != data_.end(); ++i)
        {
            int dataRow = i.key().row();
            if((dataRow >= removeStartRow) && (dataRow <= removeEndRow))
            {
                // 削除対象のデータはスキップ.
                continue;
            }

            // 削除対象row以降のインデックスの場合,インデックス値をつめる.
            newInsertIndex = dataRow;
            if(newInsertIndex >= row)
            {
                newInsertIndex -= count;
            }
            newData.insert(createIndex(newInsertIndex, 0), *i);
        }

        Q_ASSERT(data_.size() == newData.size() + count);
        data_.swap(newData);

        endRemoveRows();

        return true;

    }
    else
    {
        return false;
    }
}

bool RouteInformation::insertData(int row, const QVector3D& position, const QQuaternion& orientation, const QModelIndex &parent)
{
    INFO_START_FUNCTION() << "row=" << row << " position=" << position << " orientation=" << orientation << " parent=" << parent;
    QMutexLocker locker(mutex_);

    int targetRow = row;
    if(targetRow < 0)
    {
        targetRow = 0;
    }
    else if(targetRow > data_.size())
    {
        targetRow = data_.size();
    }

    beginInsertRows(parent, targetRow, targetRow);
    RoutePoint newPoint(position, orientation);

    QMap<QModelIndex, QVariant> newData;
    int newInsertIndex = 0;
    for(auto i = data_.begin(); i != data_.end(); ++i)
    {
        // 追加対象row以降のインデックスの場合,インデックス値を増やす.
        newInsertIndex = i.key().row();
        if(newInsertIndex >= row)
        {
            newInsertIndex++;
        }
        newData.insert(createIndex(newInsertIndex, 0), *i);
    }
    newData.insert(createIndex(targetRow, 0), QVariant::fromValue(newPoint));
    Q_ASSERT(data_.size() + 1 == newData.size());
    data_.swap(newData);

    endInsertRows();

    return true;
}

bool RouteInformation::insertData(int row, const QVector3D& position)
{
    return insertData(row, position, QQuaternion());
}

bool RouteInformation::pushBack(const RoutePoint& newPoint, const QModelIndex &parent)
{
    INFO_START_FUNCTION() << "position=" << newPoint.position() << " orientation=" << newPoint.orientation() << " parent=" << parent;
    QMutexLocker locker(mutex_);

    int targetRow = data_.size();
    beginInsertRows(parent, targetRow, targetRow);
    data_.insert(createIndex(targetRow, 0), QVariant::fromValue(newPoint));
    endInsertRows();

    return true;
}
