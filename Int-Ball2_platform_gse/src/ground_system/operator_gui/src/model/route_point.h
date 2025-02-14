#ifndef ROUTE_POINT_H
#define ROUTE_POINT_H

#include <tuple>
#include <QDataStream>
#include <QQuaternion>
#include <QVector3D>

namespace intball
{

class RoutePoint
{
public:
    RoutePoint();
    RoutePoint(const QVector3D& position, const QQuaternion& orientation, const unsigned int waitSecond = 0);
    RoutePoint(const RoutePoint &obj);
    std::tuple<QVector3D, QQuaternion> data();
    const QVector3D& position() const;
    const QQuaternion& orientation() const;
    unsigned int waitSecond() const;
    void setData(const QVector3D& position, const QQuaternion& orientation, const unsigned int waitSecond = 0);
    void setData(const QVector3D &position);
    void setData(const QQuaternion &orientation);
    void setData(const unsigned int waitSecond);

private:
    QVector3D position_;
    QQuaternion orientation_;
    unsigned int waitSecond_;
};

}
QDataStream & operator<<(QDataStream &ostream, const intball::RoutePoint& data);
QDataStream & operator>>(QDataStream &istream, intball::RoutePoint& data);
Q_DECLARE_METATYPE(intball::RoutePoint);
Q_DECLARE_METATYPE(QList<intball::RoutePoint>)

#endif // ROUTE_POINT_H
