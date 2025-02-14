#ifndef IB2_VIDEOCONTROLLER_H
#define IB2_VIDEOCONTROLLER_H
#include <QTimer>
#include <QWidget>
#include "vlc/vlc.h"

namespace intball
{
class VideoController : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief 再生種別.
     */
    enum InputType
    {
        FILE,
        URL,
    };
    Q_ENUM(InputType)

    /**
     * @brief VideoControllerコンストラクタ.
     * @param windowId 映像を表示するウィジット/ウィンドウのID.
     * @param parent　親ウィジット.
     */
    explicit VideoController(const unsigned long long windowId, QWidget *parent = nullptr);

    /**
     * @brief VideoControllerデストラクタ.
     */
    virtual ~VideoController();

public slots:

    /**
     * @brief 映像を時計周りに回転する.
     * @param[in] rotateDegree 回転角（度）.
     */
    void setRotate(int rotateDegree);

    /**
     * @brief 映像の再生設定を行い、再生を開始する.
     * @param[in] inputType 再生種別.
     * @param[in] inputString 再生する映像の指定文字列.
     */
    void start(InputType inputType, const char* inputString);

    /**
     * @brief スナップショット画像を保存する.
     */
    void takeSnapshot();

    /**
     * @brief VLCの処理を一旦停止し、再起動する.
     */
    void restart();

private:
    /**
     * @brief VLCの再生処理を呼び出し.
     */
    void callPlay();

    /**
     * @brief VLC media playerエンジンインスタンス.
     */
    libvlc_instance_t *inst_;

    /**
     * @brief VLC media playerの再生処理用インスタンス.
     */
    libvlc_media_player_t *mp_;

    /**
     * @brief 映像を表示するウィジット/ウィンドウのID.
     */
    unsigned long long targetWindowId_;

    /**
     * @brief snapshotDirectoryPath_ スナップショット保存先ディレクトリ.
     */
    std::string snapshotDirectoryPath_;

    /**
     * @brief restartTimer_ リスタート処理を呼び出すタイマー.
     */
    QTimer* restartTimer_;
};
} // namespace intball

#endif
