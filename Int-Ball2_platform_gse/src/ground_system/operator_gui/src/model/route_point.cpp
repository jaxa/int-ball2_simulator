#include <QQuaternion>
#include <QVector3D>
#include <tuple>
#include "route_point.h"

using namespace intball;

RoutePoint::RoutePoint() : position_(QVector3D()), orientation_(QQuaternion()), waitSecond_(0)
{

}

RoutePoint::RoutePoint(const QVector3D& position, const QQuaternion& orientation, const unsigned int waitSecond)
{
    position_ = position;
    orientation_ = orientation;
    waitSecond_ = waitSecond;
}

RoutePoint::RoutePoint(const RoutePoint &obj)
{
    position_ = obj.position_;
    orientation_ = obj.orientation_;
    waitSecond_ = obj.waitSecond_;
}

QDataStream & operator<<(QDataStream &ostream, const RoutePoint& data)
{
    ostream << data.position() << data.orientation() << data.waitSecond();
    return ostream;
}

QDataStream & operator>>(QDataStream &istream, RoutePoint& data)
{
    QVector3D position;
    QQuaternion orientation;
    unsigned int waitSecond;
    istream >> position;
    istream >> orientation;
    istream >>waitSecond;
    data.setData(position, orientation, waitSecond);
    return istream;
}

std::tuple<QVector3D, QQuaternion> RoutePoint::data()
{
    return std::make_tuple(position_, orientation_);
}

const QVector3D& RoutePoint::position() const
{
    return position_;
}

const QQuaternion& RoutePoint::orientation() const
{
    return orientation_;
}

unsigned int RoutePoint::waitSecond() const
{
    return waitSecond_;
}

void RoutePoint::setData(const QVector3D &position, const QQuaternion &orientation, const unsigned int waitSecond)
{
    position_ = position;
    orientation_ = orientation;
    waitSecond_ = waitSecond;
}

void RoutePoint::setData(const QVector3D &position)
{
    position_ = position;
}

void RoutePoint::setData(const QQuaternion &orientation)
{
    orientation_ = orientation;
}

void RoutePoint::setData(const unsigned int waitSecond)
{
    waitSecond_ = waitSecond;
}
