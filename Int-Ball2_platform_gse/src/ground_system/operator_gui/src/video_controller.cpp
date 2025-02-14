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
#include "config.h"

// /opt/vlc/include/vlc/plugins
#include "vlc_common.h"
#include "vlc_demux.h"
#include "vlc_input.h"
#include "vlc_vout.h"
#include "vlc_aout.h"
#include "vlc_actions.h"
#include "vlc_http.h"
#include "vlc_fixups.h"

// vlc-3.0.6/lib
#include "libvlc_internal.h"
#include "media_internal.h"
#include "media_player_internal.h"
#include "renderer_discoverer_internal.h"

#include "exception/config_error.h"
#include "operator_gui_config.h"
#include "qdebug_custom.h"
#include "video_controller.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

/**
 * @brief lock_input(from vlc/media_player.c).
 * @param[in] mp libvlc_media_player_t.
 */
static inline void lock_input(libvlc_media_player_t *mp)
{
    vlc_mutex_lock(&mp->input.lock);
}

/**
 * @brief unlock_input(from vlc/media_player.c).
 * @param[in] mp libvlc_media_player_t.
 */
static inline void unlock_input(libvlc_media_player_t *mp)
{
    vlc_mutex_unlock(&mp->input.lock);
}

/**
 * @brief libvlc_get_input_thread(from vlc/media_player.c).
 *        Retrieve the input thread. Be sure to release the object
 *        once you are done with it. (libvlc Internal).
 * @param[in] p_mi libvlc_media_player_t.
 * @return p_input_thread.
 */
input_thread_t *libvlc_get_input_thread(libvlc_media_player_t *p_mi)
{
    input_thread_t *p_input_thread;

    assert(p_mi);

    lock_input(p_mi);
    p_input_thread = p_mi->input.p_thread;
    if (p_input_thread)
        vlc_object_hold(p_input_thread);
    else
        libvlc_printerr("No active input");
    unlock_input(p_mi);

    return p_input_thread;
}

/**
 * @brief GetVouts(from vlc/video.c).
 *        Remember to release the returned vout_thread_t.
 * @param[in] p_mi libvlc_media_player_t.
 * @param[out] n size of vout_thread_t.
 * @return vout_thread_t.
 */
static vout_thread_t **GetVouts(libvlc_media_player_t *p_mi, size_t *n)
{
    input_thread_t *p_input = libvlc_get_input_thread(p_mi);
    if (!p_input)
    {
        *n = 0;
        return NULL;
    }

    vout_thread_t **pp_vouts;
    if (input_Control(p_input, INPUT_GET_VOUTS, &pp_vouts, n))
    {
        *n = 0;
        pp_vouts = NULL;
    }
    vlc_object_release(p_input);
    return pp_vouts;
}

/**
 * @brief GetVout(from vlc/video.c).
 * @param[in] mp libvlc_media_player_t.
 * @param[in] num target index of the vout_thread_t.
 * @return vout_thread_t.
 */
static vout_thread_t *GetVout(libvlc_media_player_t *mp, size_t num)
{
    vout_thread_t *p_vout = NULL;
    size_t n;
    vout_thread_t **pp_vouts = GetVouts(mp, &n);
    if (pp_vouts == NULL)
        goto err;

    if (num < n)
        p_vout = pp_vouts[num];

    for (size_t i = 0; i < n; i++)
        if (i != num)
            vlc_object_release(pp_vouts[i]);
    free(pp_vouts);

    if (p_vout == NULL)
err:
        libvlc_printerr("Video output not active");
    return p_vout;
}

VideoController::VideoController(const unsigned long long windowId, QWidget *parent)
    : QWidget(parent), targetWindowId_(windowId)
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
    libvlc_media_player_stop (mp_);
    libvlc_media_player_release (mp_);
    libvlc_release (inst_);
}

void VideoController::setRotate(int rotateDegree)
{
    if (mp_ != NULL)
    {
        size_t n;
        vout_thread_t **pp_vouts = GetVouts(mp_, &n);
        if (n > 0)
        {
            std::stringstream set;
            set << "rotate{angle=" << rotateDegree << "}";

            /*
             * libvlc_media_player_tにrotateを設定すると
             * 再生開始時にWarning/Errorが出るため,
             * 既存voutにのみrotateを設定する.
             */
            for (size_t i = 0; i < n; i++)
            {
                var_SetString(pp_vouts[i], "video-filter", set.str().c_str());
                vlc_object_release(pp_vouts[i]);
            }
            free(pp_vouts);
        }
    }
}

void VideoController::start(InputType type, const char* inputString)
{
    INFO_START_FUNCTION();

    libvlc_media_t *m;

    /*
     * VLCエンジンの読み込み.
     * libvlc_new関数でvlcコマンドラインオプションを指定可能だが,
     * その他のlibvlc**関数で上書きされるケースがあるため、ここでは処理しない.
     */
    inst_ = libvlc_new(0, NULL);

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
        break;
    }

    /*
     * 再生環境の設定.
     * libvlc_media_player_new_from_media関数内で
     * 一部変数はCreateされる.
     */
    mp_ = libvlc_media_player_new_from_media(m);
    libvlc_media_release (m);

    /* Create済み変数の再設定. */
    var_SetString(mp_, "avcodec-hw", "none");
    var_SetString(mp_, "aout", "any");
    var_SetString(mp_, "vout", "any");
    var_SetString(mp_, "window", targetWindowId_ ? "embed-xid,any" : "");
    var_SetInteger(mp_, "drawable-xid", targetWindowId_);

    /* 新規変数の作成および設定. */
    var_Create(mp_, "sout-x264-preset", VLC_VAR_STRING);
    var_SetString(mp_, "sout-x264-preset", "ultrafast");
    var_Create(mp_, "sout-x264-tune", VLC_VAR_STRING);
    var_SetString(mp_, "sout-x264-tune", "film");
    var_Create(mp_, "avcodec-threads", VLC_VAR_INTEGER);
    var_SetInteger(mp_, "avcodec-threads", 0);
    var_Create(mp_, "avcodec-fast", VLC_VAR_VOID);

    callPlay();
}

void VideoController::callPlay()
{
    /* 映像再生（待受）. */
    if(!libvlc_media_player_play(mp_))
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
