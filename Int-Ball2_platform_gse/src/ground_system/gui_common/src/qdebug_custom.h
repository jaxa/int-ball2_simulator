#ifndef INTBALL_QDEBUG
#define INTBALL_QDEBUG
#include <QDebug>
#include <QQuaternion>

QDebug operator<<(QDebug debug, const std::string &s);
QDebug operator<<(QDebug debug, const QQuaternion& q);

#define INFO_START_FUNCTION() qInfo().noquote().nospace() << "Function start: " <<  __PRETTY_FUNCTION__ << " "

#define LOG_DEBUG() qDebug().noquote().nospace()
#define LOG_INFO() qInfo().noquote().nospace()
#define LOG_WARNING() qWarning().noquote().nospace()
#define LOG_CRITICAL() qCritical().noquote().nospace()

#endif
