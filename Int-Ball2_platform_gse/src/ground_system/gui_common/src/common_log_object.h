#ifndef COMMON_LOG_OBJECT_H
#define COMMON_LOG_OBJECT_H

#include <QMetaType>
#include <QDateTime>
#include <QString>

namespace intball
{

enum class CommandLogLevel
{
    INFO,
    WARN,
    ALERT
};

const static QString LABEL_LOGLEVEL_INFO = "[INFO]";
const static QString LABEL_LOGLEVEL_WARN = "[WARN]";
const static QString LABEL_LOGLEVEL_ALERT = "[ALERT]";

struct CommandLog
{
    QDateTime timestamp;
    CommandLogLevel level;
    QString message;

    CommandLog() :
        timestamp(QDateTime()), level(CommandLogLevel::INFO), message(""){}
    CommandLog(QString setMessage) :
        timestamp(QDateTime()), level(CommandLogLevel::INFO), message(setMessage){}
    CommandLog(CommandLogLevel setLevel,
               QString setMessage) :
        timestamp(QDateTime()), level(setLevel), message(setMessage){}
    CommandLog(QDateTime setTimestamp,
               CommandLogLevel setLevel,
               QString setMessage) :
        timestamp(setTimestamp), level(setLevel), message(setMessage){}
};

namespace message
{
const static QString COMMAND_SUCCESS = "The command was sent successfully.";
const static QString COMMAND_ERROR = "The command could not be sent.";
}

}

Q_DECLARE_METATYPE(intball::CommandLog)

#endif // COMMON_LOG_OBJECT_H
