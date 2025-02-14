#include <cxxabi.h>
#include <memory>
#include <string>
#include <QDateTime>
#include <QFontMetrics>
#include <QtMath>
#include <tf/transform_listener.h>
#include "ib2_msgs.h"
#include "ros_common.h"
#include "telemetry_telecommand_config.h"
#include "utils.h"

using namespace std;


namespace intball
{

struct delete_demangle
{
    template< typename T >
    void operator ()( T * ptr ) const noexcept
    {
        free(ptr);
    }
};


string demangle(const std::type_info &ti)
{
    int status = 0;
    unique_ptr<char, delete_demangle> p( abi::__cxa_demangle(ti.name(), nullptr, nullptr, &status) );
    if(p)
    {
        return string(p.get());
    }
    else
    {
        return "";
    }

}

bool comparisonFloat(const float a, const float b)
{
    return fabsf(a - b) < FLT_EPSILON;
}

QQuaternion tfToQt(const tf::Quaternion &tfQuaternion)
{
    return QQuaternion(tfQuaternion.getW(), tfQuaternion.getX(), tfQuaternion.getY(), tfQuaternion.getZ());
}

QVector3D tfToQt(const tf::Vector3 &tfVector)
{
    return QVector3D(tfVector.getX(), tfVector.getY(), tfVector.getZ());
}

tf::Quaternion qtToTf(const QQuaternion &qtQuaternion)
{
    return tf::Quaternion(qtQuaternion.x(), qtQuaternion.y(), qtQuaternion.z(), qtQuaternion.scalar());
}

tf::Vector3 qtToTf(const QVector3D &qtVector)
{
    return tf::Vector3(qtVector.x(), qtVector.y(), qtVector.z());
}

tf::Vector3 geometryToTf(const geometry_msgs::Point& geometryPoint)
{
    return tf::Vector3(geometryPoint.x, geometryPoint.y, geometryPoint.z);
}

tf::Quaternion geometryToTf(const geometry_msgs::Quaternion& geometryQuaternion)
{
    return tf::Quaternion(geometryQuaternion.x, geometryQuaternion.y, geometryQuaternion.z, geometryQuaternion.w);
}

QVector3D geometryToQt(const geometry_msgs::Point& geometryPoint)
{
    return QVector3D(geometryPoint.x, geometryPoint.y, geometryPoint.z);
}

QQuaternion geometryToQt(const geometry_msgs::Quaternion& geometryQuaternion)
{
    return QQuaternion(geometryQuaternion.w, geometryQuaternion.x, geometryQuaternion.y, geometryQuaternion.z);
}


void getRPY(const QQuaternion& orientation, qreal& roll, qreal& pitch, qreal& yaw)
{
    tfScalar tmpRoll, tmpPitch, tmpYaw;
    tf::Quaternion tfQuaternion = qtToTf(orientation);
    tf::Matrix3x3 m(tfQuaternion);
    m.getRPY(tmpRoll, tmpPitch, tmpYaw);
    roll = tmpRoll;
    pitch = tmpPitch;
    yaw = tmpYaw;
}

QQuaternion fromRPYRadian(const qreal roll, const qreal pitch, const qreal yaw)
{
    return tfToQt(tf::createQuaternionFromRPY(roll, pitch, yaw));
}

QQuaternion fromRPYDegree(const qreal roll, const qreal pitch, const qreal yaw)
{
    return tfToQt(tf::createQuaternionFromRPY(qDegreesToRadians(roll), qDegreesToRadians(pitch), qDegreesToRadians(yaw)));
}

QVector3D transformPosition(const std::string& parentFrame, const std::string& childFrame, const QVector3D& childPosition)
{
    geometry_msgs::PointStamped pointBody;
    pointBody.point.x = static_cast<double>(childPosition.x());
    pointBody.point.y = static_cast<double>(childPosition.y());
    pointBody.point.z = static_cast<double>(childPosition.z());
    pointBody.header.frame_id = childFrame;
    pointBody.header.stamp = ros::Time(0);

    geometry_msgs::PointStamped result;
    getTransformListener()->transformPoint(parentFrame, pointBody, result);

    return QVector3D(static_cast<float>(result.point.x),
                     static_cast<float>(result.point.y),
                     static_cast<float>(result.point.z));
}

QQuaternion transformQuaternion(const std::string& parentFrame, const std::string& childFrame, const QQuaternion& childQuaternion)
{
    tf::Quaternion tfQuaternion = qtToTf(childQuaternion);

    geometry_msgs::QuaternionStamped childTfQuaternion;
    childTfQuaternion.quaternion.x = tfQuaternion.x();
    childTfQuaternion.quaternion.y = tfQuaternion.y();
    childTfQuaternion.quaternion.z = tfQuaternion.z();
    childTfQuaternion.quaternion.w = tfQuaternion.w();
    childTfQuaternion.header.frame_id = childFrame;
    childTfQuaternion.header.stamp = ros::Time(0);

    geometry_msgs::QuaternionStamped result;
    getTransformListener()->transformQuaternion(parentFrame, childTfQuaternion, result);

    return QQuaternion(static_cast<float>(result.quaternion.w),
                       static_cast<float>(result.quaternion.x),
                       static_cast<float>(result.quaternion.y),
                       static_cast<float>(result.quaternion.z));
}

QVector3D transformRelative(const QVector3D& basePosition, const QQuaternion& baseQuaternion, const QVector3D& diff)
{
    tf::Transform transform;
    transform.setOrigin(qtToTf(basePosition));
    transform.setRotation(qtToTf(baseQuaternion));

    tf::Vector3 resultVector3 = transform * qtToTf(diff);
    return tfToQt(resultVector3);
}

QQuaternion transformRelative(const QVector3D& basePosition, const QQuaternion& baseQuaternion, const QQuaternion& diff)
{
    tf::Transform transform;
    transform.setOrigin(qtToTf(basePosition));
    transform.setRotation(qtToTf(baseQuaternion));

    tf::Quaternion resultQuaternion = transform * qtToTf(diff);
    return tfToQt(resultQuaternion);
}

QQuaternion transformRelativeRPY(const QVector3D& basePosition, const QQuaternion& baseQuaternion,
                                 const float roll, const float pitch, const float yaw)
{
    return transformRelative(basePosition, baseQuaternion,
                             tfToQt(tf::createQuaternionFromRPY(
                                        static_cast<double>(roll),
                                        static_cast<double>(pitch),
                                        static_cast<double>(yaw))));
}

qreal roundPositionValue(const qreal value)
{
    return static_cast<qreal>(std::round(value * 100) / 100);
}

qreal roundDegree(const qreal value)
{
    return static_cast<qreal>(std::round(qRadiansToDegrees(value)));
}

qreal roundPositionValue(const float value)
{
    return static_cast<qreal>(std::round(value * 100) / 100);
}

qreal roundDegree(const float value)
{
    return static_cast<qreal>(std::round(qRadiansToDegrees(value)));
}

QDateTime rosToQt(const ros::Time& time, const Qt::TimeSpec timeSpec)
{
    QDateTime returnDateTime;
    returnDateTime.setMSecsSinceEpoch(static_cast<long long>(time.sec) * 1000 + static_cast<long long>(time.nsec / 1000000));
    returnDateTime.setTimeSpec(timeSpec);
    return returnDateTime;
}

QString dateTimeString(const QDateTime& time)
{
    return time.toUTC().toString("yyyy/MM/dd hh:mm:ss t");
}

QString dateTimeString(const ros::Time& time, const Qt::TimeSpec timeSpec)
{
    return dateTimeString(rosToQt(time, timeSpec));
}

QString dateTimeStringWithoutYear(const QDateTime& time)
{
    return time.toUTC().toString("MM/dd hh:mm:ss t");
}

QString dateTimeStringWithoutYear(const ros::Time& time, const Qt::TimeSpec timeSpec)
{
    return dateTimeStringWithoutYear(rosToQt(time, timeSpec));
}

QString secondToTimeStringUpToHour(const unsigned long setSecond)
{
    unsigned long hour = setSecond / 3600;
    unsigned long minite = setSecond % 3600 / 60;
    unsigned long second = setSecond % 3600 % 60;
    return QString("%1:%2:%3")
           .arg(hour, 2, 10, QChar('0'))
           .arg(minite, 2, 10, QChar('0'))
           .arg(second, 2, 10, QChar('0'));
}

QString secondToTimeStringUpToMinute(const unsigned long setSecond)
{
    unsigned long minite = setSecond / 60;
    unsigned long second = setSecond % 60;
    return QString("%1:%2")
           .arg(minite, 2, 10, QChar('0'))
           .arg(second, 2, 10, QChar('0'));
}

QString secondToRoundedTimeStringUpToHour(const unsigned long setSecond)
{
    unsigned long hour = setSecond / 3600;
    unsigned long minite = setSecond % 3600 / 60;
    return QString("%1:%2")
           .arg(hour, 2, 10, QChar('0'))
           .arg(minite, 2, 10, QChar('0'));
}

QDateTime parseDateTimeString(const QString& dateTimeString)
{
    return QDateTime::fromString(dateTimeString, "yyyy/MM/dd hh:mm:ss t");
}

QString getModeString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto mode = qvariant.value<unsigned char>();

    if(mode == INTBALL_MODE_UNKNOWN)
    {
         // UNKNOWNはGUI特有であるため数値を出力しない.
        return MODE_TYPE_LABEL.value(mode);
    }
    else
    {
        return createValueString(mode, MODE_TYPE_LABEL);
    }
}

QString getAliveStatusResultString(const unsigned char status)
{
    return createValueString(status, telemetry::ALIVE_STATUS_LABEL);
}

QString getCtlStatusAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<int>());
    auto status = qvariant.value<int>();

    if(status == telemetry::GUIDANCE_CONTROL_TYPE_UNKNOWN)
    {
        // UNKNOWNはGUI特有であるため数値を出力しない.
        return telemetry::GUIDANCE_CONTROL_TYPE_LABEL.value(status);
    }
    else
    {
        return createValueString(status, telemetry::GUIDANCE_CONTROL_TYPE_LABEL);
    }
}

QString getCtlCommandResultAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto status = qvariant.value<unsigned char>();

    return createValueString(status, telemetry::GUIDANCE_CONTROL_ACTION_RESULT_LABEL);
}

QString getNavigationStatusAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto status = qvariant.value<unsigned char>();

    return createValueString(status, telemetry::NAVIGATION_STATUS_LABEL);
}

QString getUpdateParameterResponseResultAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto result = qvariant.value<unsigned char>();

    return createValueString(result, telemetry::UPDATE_PARAMETER_RESULT_LABEL);
}

QString getIntBallModeAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto mode = qvariant.value<unsigned char>();

    return createValueString(mode, MODE_TYPE_LABEL);
}

QString getMarkerCorrectionResultString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<short>());
    auto mode = qvariant.value<short>();

    return createValueString(mode, telemetry::MARKER_CORRECTION_STATUS_LABEL);
}

QString getColorRGBAAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<std_msgs::ColorRGBA>());
    auto color = qvariant.value<std_msgs::ColorRGBA>();

    return QString("R:%1   G:%2   B:%3").arg(color.r).arg(color.g).arg(color.b);
}

QString getBoolAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<bool>());
    auto flag = qvariant.value<bool>();

    if(flag)
    {
        return "True";
    }
    else
    {
        return "False";
    }
}

QString getWhiteBalanceModeAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<ib2_msgs::MainCameraWhiteBalanceMode>());
    auto mode = qvariant.value<ib2_msgs::MainCameraWhiteBalanceMode>();

    return createValueString(mode.mode, MAIN_CAMERA_WHITE_BALANCE_MODE_LABEL);
}

QString getWhiteBalanceModeAsString(const unsigned char mode)
{
    return createValueString(mode, MAIN_CAMERA_WHITE_BALANCE_MODE_LABEL);
}

QString getQuaternionAsString(const QQuaternion& quaternion)
{
    return  QString("%1, %2, %3, %4")
            .arg(quaternion.x())
            .arg(quaternion.y())
            .arg(quaternion.z())
            .arg(quaternion.scalar());
}

QString getDegreeAsString(const QQuaternion& quaternion)
{
    qreal roll, pitch, yaw;
    getRPY(quaternion, roll, pitch, yaw);

    return  QString("%5, %6, %7")
            .arg(roundDegree(roll))
            .arg(roundDegree(pitch))
            .arg(roundDegree(yaw));
}

QString getPowerStatusAsString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<ib2_msgs::PowerStatus>());
    auto power = qvariant.value<ib2_msgs::PowerStatus>();

    return createValueString(power.status, POWER_STATUS_LABEL);
}

QString getNavigationStartUpResultAsString(const unsigned char result)
{
    return createValueString(result, telemetry::NAVIGATION_STARTUP_RESULT_LABEL);
}

QStringList splitAsFixedWidthString(const QString& string, const int width, const QFontMetrics& metrics)
{
    QStringList result;

    // 空白文字で分割する.
    QStringList splitWordList = string.split(QRegExp("\\s+"), QString::SkipEmptyParts);

    // 文字列の表示幅算出用のメトリクス.
    QStringList holdingWord;
    for(auto word = splitWordList.begin(); word != splitWordList.end(); ++word)
    {
        // 単語を結合した際の表示幅を確認する.
        int calculateWidth = holdingWord.empty() ?
                    metrics.horizontalAdvance(*word) :
                    metrics.horizontalAdvance(holdingWord.join(" ")) + metrics.horizontalAdvance(" " + (*word));
        if(calculateWidth <= width)
        {
            // 分割した単語を結合しても表示幅しきい値に到達しなかった場合,保持する.
            holdingWord.push_back(*word);
        }
        else
        {
            if(holdingWord.length() > 0)
            {
                /*
                 * 分割した単語を結合した結果表示幅しきい値に達した場合,しきい値を超える前までの単語を結合して
                 * 1文字列として扱う.
                 */
                result.push_back(holdingWord.join(" "));
                holdingWord.clear();
            }


            int startCharIndex = 0;
            for(auto j = 1; j < (*word).length(); ++j)
            {
                if(metrics.horizontalAdvance((*word).mid(startCharIndex, j - startCharIndex)) > width)
                {
                    // 1単語で表示幅しきい値を超える場合,表示幅に収まる文字までを1文字列として扱う.
                    result.push_back((*word).mid(startCharIndex, j - startCharIndex - 1));
                    startCharIndex = j - 1;
                }
            }
            if(startCharIndex < (*word).length())
            {
                // 未処理の文字を1ワードとして保持する.
                holdingWord.push_back((*word).mid(startCharIndex, (*word).length()));
            }
        }
    }
    if(holdingWord.length() > 0)
    {
        result.push_back(holdingWord.join(" "));
    }

    return result;
}

QString automaticLineBreak(const QString& string, const int width, const QFontMetrics& metrics)
{
    return splitAsFixedWidthString(string, width, metrics).join("\n");
}

QColor fromColorRGBA(const std_msgs::ColorRGBA& color)
{
    return QColor(color.r,
                  color.g,
                  color.b,
                  color.a);
}

QString getPlatformModeString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto mode = qvariant.value<unsigned char>();

    if(mode == PLATFORM_MODE_UNKNOWN)
    {
         // UNKNOWNはGUI特有であるため数値を出力しない.
        return PLATFORM_MODE_TYPE_LABEL.value(mode);
    }
    else
    {
        return createValueString(mode, PLATFORM_MODE_TYPE_LABEL);
    }
}

QString getPlatformOperationTypeString(const QVariant& qvariant)
{
    Q_ASSERT(qvariant.canConvert<unsigned char>());
    auto type = qvariant.value<unsigned char>();

    if(type == PLATFORM_OPERATION_TYPE_UNKNOWN)
    {
         // UNKNOWNはGUI特有であるため数値を出力しない.
        return PLATFORM_MODE_TYPE_LABEL.value(type);
    }
    else
    {
        return createValueString(type, PLATFORM_OPERATION_TYPE_LABEL);
    }
}

QString getContainerStatusString(const unsigned char status)
{
    return createValueString(status, telemetry::CONTAINER_STATUS_LABEL);
}

} // namespace intball
