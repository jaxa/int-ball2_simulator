#ifndef DIALOG_FACTORY_H
#define DIALOG_FACTORY_H

#include <QLabel>
#include <QMessageBox>
#include "camera_microphone_settings_dialog.h"
#include "led_settings_dialog.h"
#include "ui/dialog.h"

namespace intball
{

class DialogFactory
{
public:
    static intball::MainDialog& getMainInformationDialog();
    static intball::MainDialog& getMainAlertDialog();
    static intball::MainDialog& getMainConfirmDialog();
    static intball::CameraMicrophoneSettingsDialog& getCameraMicrophoneSettingsDialog();
    static intball::LedSettingsDialog& getLedSettingsDialog();

    /**
     * @brief テレコマンドの送信確認.
     * @param command 確認対象コマンドの表記文字列.
     * @return true: OK, false: キャンセル
     */
    static bool telecommandCheck(const QString& command);

    /**
     * @brief テレコマンドの送信確認.
     * @param command 確認対象コマンドの表記文字列.
     * @param callbackOk OK押下時に実行する処理
     */
    static void telecommandCheck(const QString& command, std::function<void()> const& callbackOk);

    /**
     * @brief ステータスが未初期化（現在値不明）のままテレコマンドを送信するかの確認.
     * @return true: OK, false: キャンセル
     */
    static bool telecommandInitCheck();

    /**
     * @brief ステータスが未初期化（現在値不明）のままテレコマンドを送信するかの確認.
     * @param callbackOk OK押下時に実行する処理
     */
    static void telecommandInitCheck(std::function<void()> const& callbackOk);

    /**
     * @brief 前回送信したテレコマンドの応答待ちのまま、次テレコマンドを送信するかの確認.
     * @return true: OK, false: キャンセル
     */
    static bool telecommandWaitCheck();

    /**
     * @brief 前回送信したテレコマンドの応答待ちのまま、次テレコマンドを送信するかの確認.
     * @param callbackOk OK押下時に実行する処理
     */
    static void telecommandWaitCheck(std::function<void()> const& callbackOk);

    static void alert(const QString& message, bool forceModal=true);
    static void showInformation(const QString& message, bool forceModal=true);
    static bool executeConfirm(const QString& message);
    static void activate(QDialog* dialog);

private:
    static intball::MainDialog* operatorInformationDialog;
    static intball::MainDialog* operatorAlertDialog;
    static intball::MainDialog* operatorConfirmDialog;
    static CameraMicrophoneSettingsDialog* cameraMicrophoneSettingsDialog;
    static LedSettingsDialog* ledSettingsDialog;
    static intball::MainDialog* tmpDialog;
};

}



#endif // DIALOG_FACTORY_H
