#ifndef GUI_CommunicationConfig_BASE_H
#define GUI_CommunicationConfig_BASE_H
#include <string>
#include <QString>
#include "yaml-cpp/yaml.h"

namespace intball
{
class CommunicationConfig
{
public:

    /**
     * @brief 通信変換設定ファイルのテレコマンド情報のキー.
     */
    static const std::string KEY_TELECOMMAND;

    /**
     * @brief 通信変換設定ファイルのテレコマンド情報におけるIDのキー.
     */
    static const std::string KEY_TELECOMMAND_ID;

    /**
     * @brief 通信変換設定ファイルのテレコマンド情報における名称のキー.
     */
    static const std::string KEY_TELECOMMAND_NAME;

    /**
     * @brief 通信変換設定ファイルのテレコマンド情報におけるデータ型のキー.
     */
    static const std::string KEY_TELECOMMAND_DATA_CLASS;

    /**
     * @brief 通信変換設定ファイルのテレメトリ情報のキー.
     */
    static const std::string KEY_TELEMETRY;

    /**
    * @brief 通信変換設定ファイルのテレメトリ情報（normal）のキー.
    */
    static const std::string KEY_TELEMETRY_NORMAL;

    /**
    * @brief 通信変換設定ファイルのテレメトリ情報（split）のキー.
    */
    static const std::string KEY_TELEMETRY_SPLIT;

    /**
     * @brief CommunicationConfigコンストラクタ.
     */
    CommunicationConfig();

    /**
     * @brief CommunicationConfigデストラクタ.
     */
    virtual ~CommunicationConfig() = default;

    /**
     * @brief 設定ファイルのパスを取得する.
     * @return 設定ファイルのパス.
     */
    QString getFilePath() const;

    /**
     * @brief 名称（name）値からデータ型（data_class）値を取得する.
     * @param name 名称.
     * @return データ型.
     */
    std::string getDataClassByName(const std::string& name) const;

    QString getTelecommandNameById(const unsigned short name) const;

private:
    /**
     * @brief 設定ファイル内容.
     */
    YAML::Node config_;

    /**
     * @brief 設定ファイルのパス.
     */
    QString filePath_;
};


} // namespace intball

#endif
