#ifndef GUI_CONFIG_BASE_H
#define GUI_CONFIG_BASE_H
#include <QMetaEnum>
#include <QMutex>
#include <QSettings>
#include "exception/config_error.h"
#include "qdebug_custom.h"

namespace intball
{
namespace qsettings
{
namespace key
{
static const QString KEY_COMMUNICATION_CONFIG_PACKAGE = "Common/CommunicationConfigPackage";
static const QString KEY_COMMUNICATION_CONFIG_FILE_PATH = "Common/CommunicationConfigFilePath";
static const QString KEY_CAMERA_CONFIG_PACKAGE = "Common/CameraConfigPackage";
static const QString KEY_CAMERA_CONFIG_FILE_PATH = "Common/CameraConfigFilePath";
static const QString KEY_PARAMETER_MANAGER_CONFIG_PACKAGE = "Common/ParameterManagerConfigPackage";
static const QString KEY_PARAMETER_MANAGER_CONFIG_FILE_PATH = "Common/ParameterManagerConfigFilePath";
static const QString KEY_LOG_OUTPUT_DIR = "Common/LogOutputDir";
static const QString KEY_LOG_ROTATE_HOURS = "Common/LogRotateHours";
static const QString KEY_LOG_ROTATE_MINUTES = "Common/LogRotateMinutes";
static const QString KEY_LOG_LEVEL = "Common/LogLevel";
static const QString KEY_DEFAULT_ROSPARAM_FILE_PATH = "App/DefaultRosparamFilePath";
static const QString KEY_CAMERA_BITRATE_LIST = "Camera/BitRate";
static const QString KEY_CAMERA_FPS_MAX_4K = "Camera/FpsMax4K";
static const QString KEY_CAMERA_FPS_MAX_HD_FULLHD = "Camera/FpsMaxHdFullHD";
static const QString KEY_LED_MIN_GAIN = "Led/MinGain";
static const QString KEY_TIME_TO_GO_THRESHOLD = "App/TimeToGoThreshold";
static const QString KEY_TIMEOUT_SECOND_INTBALL_TELEMETRY = "Alert/TimeoutSecondIntballTelemetry";
static const QString KEY_TIMEOUT_SECOND_DOCK_TELEMETRY = "Alert/TimeoutSecondDockTelemetry";
static const QString KEY_WARNING_TEMPERATURE = "Alert/WarningTemperature";
static const QString KEY_CRITICAL_TEMPERATURE = "Alert/CriticalTemperature";
static const QString KEY_NORMAL_TEMPERATURE = "Alert/NormalTemperature";
static const QString KEY_WARNING_STORAGE = "Alert/WarningStorage";
static const QString KEY_CRITICAL_STORAGE = "Alert/CriticalStorage";
static const QString KEY_WARNING_BATTERY_REMAIN = "Alert/WarningBatteryRemain";
static const QString KEY_CRITICAL_BATTERY_REMAIN = "Alert/CriticalBatteryRemain";
}
class Config
{
public:

    /**
     * @brief GUI設定情報の読み込み.
     * @param rosPackage 読み込み対象のROSパッケージ名.
     */
    static void load(const QString& rosPackage);

    static QString configFilePath();

    static QString packagePath();

    static QVariant value(const QString& key, bool allowEmpty = false);

    static QString valueAsString(const QString& key, bool allowEmpty = false);

    static std::string valueAsStdString(const QString& key, bool allowEmpty = false);

    static int valueAsInt(const QString& key, bool allowEmpty = false);

    static unsigned short valueAsUShort(const QString& key, bool allowEmpty = false);

    static unsigned long valueAsULong(const QString& key, bool allowEmpty = false);

    static float valueAsFloat(const QString& key, bool allowEmpty = false);

    static double valueAsDouble(const QString& key, bool allowEmpty = false);

    static QList<QString> valueAsStringList(const QString& key, bool allowEmpty = false);

    static bool valueOnOffAsBool(const QString& key, bool allowEmpty = false, bool defaultValue = false);

    template<class T>
    static T valueAsEnum(const T, const QString& key)
    {
        auto targetValue = value(key);

        QMutexLocker locker(&mux_);
        const char* valueString = targetValue.toString().toStdString().c_str();

        auto meta = QMetaEnum::fromType<T>();
        bool isOk = false;
        auto e = static_cast<T>(meta.keyToValue(valueString, &isOk));
        if(isOk)
        {
            return e;
        }
        else
        {
            throwIntBallConfigError(path_,
                                    QString::asprintf("Invalid value: key=%s value=%s",
                                            key.toStdString().c_str(),
                                            valueString));
        }
    }

private:
    /**
     * @brief GUI設定情報.
     */
    static QScopedPointer<QSettings> guiConfig_;

    static QMutex mux_;

    static QString path_;

    static QString packagePath_;
};

} // namespace qsettings

} // namespace intball

#endif
