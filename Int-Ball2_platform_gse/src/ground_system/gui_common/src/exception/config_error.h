#ifndef CONFIG_ERROR_H
#define CONFIG_ERROR_H
#include <stdexcept>
#include <QString>

namespace intball
{
class ConfigError : public std::runtime_error
{
public:
    ConfigError() = delete;
    ConfigError(const std::string& what_arg);
    virtual ~ConfigError() = default;
};

[[noreturn]] void throwIntBallConfigError(const std::string& file, const std::string& message);
[[noreturn]] void throwIntBallConfigError(const QString& file, const QString& message);
}

#endif // CONFIG_ERROR_H
