#include <assert.h>
#include <string>
#include <limits>
#include <vector>
#include <functional>
#include <iostream>
#include <sstream>
#include <QDateTime>
#include <QDir>

#include "vlc/vlc.h"

#include "exception/config_error.h"
#include "operator_gui_config.h"
#include "qdebug_custom.h"
#include "video_controller.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

VideoController::VideoController(const unsigned long long windowId, QWidget *parent)
    : QWidget(parent), inst_(nullptr), mp_(nullptr), targetWindowId_(windowId)
{
    snapshotDirectoryPath_ = Config::valueAsStdString(KEY_SNAPSHOT_DIRECTORY);
    if(QString::fromStdString(snapshotDirectoryPath_).right(1) != "/")
    {
        snapshotDirectoryPath_ = snapshotDirectoryPath_ + "/";
    }

    QFileInfo dirInfo(snapshotDirectoryPath_.c_str());
    if(!dirInfo.exists())
    {
        throwIntBallConfigError(Config::configFilePath(),
                                QString::asprintf("Directory not found: %s", snapshotDirectoryPath_.c_str()));
    }
    if(!dirInfo.isWritable())
    {
        throwIntBallConfigError(Config::configFilePath(),
                                QString::asprintf("Permission denied: : %s", snapshotDirectoryPath_.c_str()));
    }

    // リスタート用タイマー.
    restartTimer_ = new QTimer(this);
    restartTimer_->setInterval(Config::valueAsInt(key::KEY_VIDEO_INPUT) * 1000);
    restartTimer_->setSingleShot(true);
    connect(restartTimer_, &QTimer::timeout, this, &VideoController::callPlay);
}

VideoController::~VideoController()
{
    if(mp_)
    {
        libvlc_media_player_stop(mp_);
        libvlc_media_player_release(mp_);
    }
    if(inst_)
    {
        libvlc_release(inst_);
    }
}

void VideoController::setRotate(int rotateDegree)
{
    /*
     * VLC内部APIを使用せず、公開APIのみでrotateを設定する.
     * 再生中のストリームに対し、libvlc_video_set_adjust等では回転設定ができないため,
     * メディアオプション "--video-filter=rotate" および "--rotate-angle=N" を
     * start()時にメディアに設定する方式を採用する.
     * 再生中の回転角度変更が必要な場合はrestart()を使用する.
     *
     * この関数は後方互換のために残すが、実際の回転設定はstart時に行う.
     */
    Q_UNUSED(rotateDegree);
    LOG_INFO() << "setRotate called with degree=" << rotateDegree
               << " (rotation is applied at media start time)";
}

void VideoController::start(InputType type, const char* inputString)
{
    INFO_START_FUNCTION();

    libvlc_media_t *m;

    /*
     * VLCエンジンの読み込み.
     * libvlc_new関数でvlcコマンドラインオプションを指定可能.
     */
    const char* vlc_args[] = {
        "--avcodec-hw=none",
        "--no-xlib"
    };
    inst_ = libvlc_new(2, vlc_args);

    switch(type)
    {
    case InputType::FILE:
        m = libvlc_media_new_path(inst_, inputString);
        break;
    case InputType::URL:
        m = libvlc_media_new_location(inst_, inputString);
        break;
    default:
        Q_ASSERT_X(false, __FUNCTION__, QString("Invalid type: %1").arg(type).toStdString().c_str());
        return;
    }

    /*
     * メディアオプションを公開APIで設定する.
     * var_Create/var_SetString等の内部APIは使用しない.
     */
    libvlc_media_add_option(m, ":sout-x264-preset=ultrafast");
    libvlc_media_add_option(m, ":sout-x264-tune=film");
    libvlc_media_add_option(m, ":avcodec-threads=0");
    libvlc_media_add_option(m, ":avcodec-fast");

    /*
     * 再生環境の設定.
     */
    mp_ = libvlc_media_player_new_from_media(m);
    libvlc_media_release(m);

    /* X Window IDの設定 (公開API). */
    if(targetWindowId_)
    {
        libvlc_media_player_set_xwindow(mp_, static_cast<uint32_t>(targetWindowId_));
    }

    callPlay();
}

void VideoController::callPlay()
{
    /* 映像再生（待受）. */
    if(libvlc_media_player_play(mp_) != 0)
    {
        LOG_WARNING() << "Could not start video standby.";
    }
}

void VideoController::takeSnapshot()
{
    QDateTime dt = QDateTime::currentDateTime();
    std::string path = snapshotDirectoryPath_ + dt.toString("yyyyMMdd_hhmmss").toStdString() + "_snapshot.png";
    LOG_INFO() << "Take a snapshot: " << path;

    if(libvlc_video_take_snapshot(mp_, 0, path.c_str(), 0, 0) != 0)
    {
        LOG_WARNING() << "Faild to take a snapshot: " << path;
    }
}

void VideoController::restart()
{
    INFO_START_FUNCTION();

    libvlc_media_player_stop(mp_);

    // 一定時間経過した後にリスタートする.
    restartTimer_->start();
}
