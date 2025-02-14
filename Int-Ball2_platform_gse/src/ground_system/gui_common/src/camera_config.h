#ifndef CAMERA_CONFIG_H
#define CAMERA_CONFIG_H
#include <string>
#include <QObject>
#include <QMap>
#include <QString>
#include "yaml-cpp/yaml.h"

namespace intball
{
/**
 * @brief 画質設定の構造体.
 */
struct ResolutionSettingsItem
{
    unsigned int id;
    QString name;
    unsigned int width;
    unsigned int height;
};

class CameraConfig
{
public:
    /**
     * @brief カメラ設定ファイルの画質設定のキー.
     */
    static const std::string KEY_RESOLUTION;

    /**
     * @brief CameraConfigコンストラクタ.
     */
    CameraConfig();

    /**
     * @brief CameraConfigデストラクタ.
     */
    virtual ~CameraConfig() = default;

    /**
     * @brief 設定ファイルのパスを取得する.
     * @return 設定ファイルのパス.
     */
    QString getFilePath();


    /**
     * @brief 全ての画質設定を取得する.
     * @return 画質設定.
     */
    const QMap<unsigned int, ResolutionSettingsItem>& getAllQualitySettings();

    /**
     * @brief 指定したIDの画質設定を取得する.
     * @param id 画質設定のID.
     * @return 画質設定構造体.
     */
    ResolutionSettingsItem getQualitySettingById(const unsigned int id);

    /**
     * @brief 指定したIDの画質設定の文字列表記を取得する.
     * @param id 画質設定のID.
     * @return 画質設定の文字列表記.
     */
    QString getQualitySettingAsStringById(const unsigned int id);
private:
    /**
     * @brief 設定ファイル内容.
     */
    YAML::Node config_;

    /**
     * @brief 設定ファイルのパス.
     */
    QString filePath_;


    /**
     * @brief 設定ファイルから読み出した画質設定.
     */
    QMap<unsigned int, ResolutionSettingsItem> resolutionConfig_;
};


} // namespace intball

Q_DECLARE_METATYPE(intball::ResolutionSettingsItem);

#endif
