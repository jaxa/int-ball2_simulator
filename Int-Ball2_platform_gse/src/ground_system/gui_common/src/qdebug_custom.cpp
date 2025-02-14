#include "qdebug_custom.h"

QDebug operator<<(QDebug debug, const std::string &s)
{
    QDebugStateSaver saver(debug);
    debug.noquote() << QString::fromStdString(s);
    return debug;
}


QDebug operator<<(QDebug debug, const QQuaternion& q)
{
    QDebugStateSaver saver(debug);
    debug.noquote() <<
                    QString("QQuaternion(%1, %2, %3, %4)")
                    .arg(static_cast<double>(q.x())).arg(static_cast<double>(q.y()))
                    .arg(static_cast<double>(q.z())).arg(static_cast<double>(q.scalar()));
    return debug;
}
