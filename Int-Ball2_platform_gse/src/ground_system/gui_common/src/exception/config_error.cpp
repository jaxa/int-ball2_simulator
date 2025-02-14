#include <QString>
#include "config_error.h"
#include "qdebug_custom.h"

using namespace intball;

ConfigError::ConfigError(const std::string& what_arg) : std::runtime_error (what_arg)
{
}

[[noreturn]] void intball::throwIntBallConfigError(const std::string& file, const std::string& message)
{
    LOG_CRITICAL() << message;
    throw ConfigError("File[" + file + "] " + message);
}

[[noreturn]] void intball::throwIntBallConfigError(const QString& file, const QString& message)
{
    LOG_CRITICAL() << message;
    throw ConfigError("File[" + file.toStdString() + "] " + message.toStdString());
}
