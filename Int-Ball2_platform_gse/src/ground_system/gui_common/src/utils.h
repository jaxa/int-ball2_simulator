#ifndef INTBALL_H
#define INTBALL_H
#include <QDateTime>
#include <QFontMetrics>
#include <QItemSelectionModel>
#include <QQuaternion>
#include <QVector3D>
#include <tf/transform_listener.h>
#include "ib2_msgs.h"

namespace intball
{

std::string demangle(const std::type_info &ti);
bool comparisonFloat(const float a, const float b);
QQuaternion tfToQt(const tf::Quaternion &tfQuaternion);
tf::Quaternion qtToTf(const QQuaternion &qtQuaternion);
QVector3D tfToQt(const tf::Vector3 &tfVector);
tf::Vector3 qtToTf(const QVector3D &qtVector);
tf::Vector3 geometryToTf(const geometry_msgs::Point& geometryPoint);
tf::Quaternion geometryToTf(const geometry_msgs::Quaternion& geometryQuaternion);
QVector3D geometryToQt(const geometry_msgs::Point& geometryPoint);
QQuaternion geometryToQt(const geometry_msgs::Quaternion& geometryQuaternion);
void getRPY(const QQuaternion& orientation, qreal& roll, qreal& pitch, qreal& yaw);
QQuaternion fromRPYRadian(const qreal roll, const qreal pitch, const qreal yaw);
QQuaternion fromRPYDegree(const qreal roll, const qreal pitch, const qreal yaw);
QVector3D transformPosition(const std::string& parentFrame, const std::string& childFrame, const QVector3D& childPosition);
QVector3D transformRelative(const QVector3D& basePosition, const QQuaternion& baseQuaternion, const QVector3D& diff);
QQuaternion transformRelative(const QVector3D& basePosition, const QQuaternion& baseQuaternion, const QQuaternion& diff);
QQuaternion transformRelativeRPY(const QVector3D& basePosition, const QQuaternion& baseQuaternion,
                                 const float roll, const float pitch, const float yaw);
qreal roundPositionValue(const qreal value);
qreal roundDegree(const qreal value);
qreal roundPositionValue(const float value);
qreal roundDegree(const float value);

QDateTime rosToQt(const ros::Time& time, const Qt::TimeSpec timeSpec = Qt::LocalTime);
QString dateTimeString(const QDateTime& time);
QString dateTimeString(const ros::Time& time, const Qt::TimeSpec timeSpec = Qt::LocalTime);
QString dateTimeStringWithoutYear(const QDateTime& time);
QString dateTimeStringWithoutYear(const ros::Time& time, const Qt::TimeSpec timeSpec);
QString secondToTimeStringUpToHour(const unsigned int long setSecond);
QString secondToTimeStringUpToMinute(const unsigned long setSecond);
QString secondToRoundedTimeStringUpToHour(const unsigned long setSecond);
QDateTime parseDateTimeString(const QString& dateTimeString);

QString getModeString(const QVariant& qvariant);
QString getAliveStatusResultString(const unsigned char status);
QString getCtlStatusAsString(const QVariant& qvariant);
QString getCtlCommandResultAsString(const QVariant& qvariant);
QString getNavigationStatusAsString(const QVariant& qvariant);
QString getUpdateParameterResponseResultAsString(const QVariant& qvariant);
QString getIntBallModeAsString(const QVariant& qvariant);
QString getColorRGBAAsString(const QVariant& qvariant);
QString getWhiteBalanceModeAsString(const QVariant& qvariant);
QString getWhiteBalanceModeAsString(const unsigned char mode);
QString getBoolAsString(const QVariant& qvariant);
QString getDegreeAsString(const QQuaternion& quaternion);
QString getQuaternionAsString(const QQuaternion& quaternion);
QString getPowerStatusAsString(const QVariant& qvariant);
QString getNavigationStartUpResultAsString(const unsigned char result);
QString getMarkerCorrectionResultString(const QVariant& qvariant);
QStringList splitAsFixedWidthString(const QString& string, const int width, const QFontMetrics& metrics);
QString automaticLineBreak(const QString& string, const int width, const QFontMetrics& metrics);
QString getPlatformModeString(const QVariant& qvariant);
QString getPlatformOperationTypeString(const QVariant& qvariant);
QString getContainerStatusString(const unsigned char status);

QColor fromColorRGBA(const std_msgs::ColorRGBA& color);

template<typename T, typename U>
T getKeyFromValue(const QMap<T, U>& map, const U& value)
{
    for(auto e : map.keys())
    {
        if(map.value(e) == value)
        {
            return e;
        }
    }

    return T();
}

template<typename T>
QString createValueString(const T value, const QMap<T, QString>& labels)
{
    QString resultString;
    if(labels.keys().indexOf(value) != -1)
    {
        resultString = QString("%1(%2)")
                            .arg(labels.value(value))
                            .arg(value);
    }
    else
    {
        resultString = QString("(Invalid status)(%1)")
                            .arg(value);
    }
    return resultString;
}

} // namespace intball

#endif
