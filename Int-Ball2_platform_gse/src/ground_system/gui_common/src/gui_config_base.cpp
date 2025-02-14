#include <QFile>
#include <QMutexLocker>
#include <QScopedPointer>
#include <QSettings>
#include <QTextCodec>
#include <ros/package.h>
#include "gui_config_base.h"
#include "exception/config_error.h"
#include "qdebug_custom.h"

using namespace intball;
using namespace intball::qsettings;

QScopedPointer<QSettings> Config::guiConfig_;
QString Config::path_;
QString Config::packagePath_;
QMutex Config::mux_;

QString Config::configFilePath()
{
    Q_ASSERT(!guiConfig_.isNull());
    return path_;
}

QString Config::packagePath()
{
    Q_ASSERT(!packagePath_.isEmpty());
    return packagePath_;
}

void Config::load(const QString& rosPackage)
{
    if(guiConfig_.isNull())
    {
        packagePath_ = QString::fromStdString(ros::package::getPath(rosPackage.toStdString()));
        path_ = packagePath_ + "/config/gui_config.ini";
        LOG_INFO() << "Read the configuration file " << path_;
        QFile configFile(path_);
        if(!configFile.exists())
        {
            throwIntBallConfigError(path_.toStdString(), "File not found.");
        }

        guiConfig_.reset(new QSettings(path_, QSettings::IniFormat));
        guiConfig_->setIniCodec(QTextCodec::codecForName("UTF-8"));
    }
    else
    {
        LOG_INFO() << "Already loaded: " << path_;
    }
}

QVariant Config::value(const QString& key, bool allowEmpty)
{
    QMutexLocker locker(&mux_);
    QVariant targetValue = guiConfig_->value(key);
    if(!targetValue.isValid())
    {
        throwIntBallConfigError(path_, QString::asprintf("Invalid key: %s", key.toStdString().c_str()));
    }

    if(targetValue.isNull() && !allowEmpty)
    {
        throwIntBallConfigError(path_, QString::asprintf("The value is null: key=%s", key.toStdString().c_str()));
    }

    return targetValue;
}

QString Config::valueAsString(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString();
}

std::string Config::valueAsStdString(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString().toStdString();
}

int Config::valueAsInt(const QString& key, bool allowEmpty)
{
    return static_cast<int>(valueAsFloat(key, allowEmpty));
}

unsigned short Config::valueAsUShort(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString().toUShort();
}

unsigned long Config::valueAsULong(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString().toULong();
}


double Config::valueAsDouble(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString().toDouble();
}

float Config::valueAsFloat(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toString().toFloat();
}

QList<QString> Config::valueAsStringList(const QString& key, bool allowEmpty)
{
    return value(key, allowEmpty).toStringList();
}

bool Config::valueOnOffAsBool(const QString& key, bool allowEmpty, bool defaultValue)
{
    QString stringValue = Config::valueAsString(key, allowEmpty).toLower();
    if(stringValue == "on"){
        return true;
    }else if(stringValue == "off"){
        return false;
    }else{
        return defaultValue;
    }
}
