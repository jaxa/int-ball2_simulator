#include <QApplication>
#include <QScreen>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include "ground_system_main_window.h"
#include "gui_color.h"
#include "operator_gui_config.h"
#include "oss/spdlog/spdlog.h"
#include "oss/spdlog/sinks/daily_file_sink.h"
#include "oss/spdlog/async.h"
#include "qdebug_custom.h"
#include "ros_common.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

namespace  {

void logHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    Q_UNUSED(context);
    QByteArray local_msg = msg.toLocal8Bit();
    switch (type)
    {
    case QtDebugMsg:
        spdlog::debug(local_msg.constData());
        break;
    case QtInfoMsg:
        spdlog::info(local_msg.constData());
        break;
    case QtWarningMsg:
        spdlog::warn(local_msg.constData());
        break;
    case QtCriticalMsg:
    case QtFatalMsg:
        spdlog::error(local_msg.constData());
        break;
    default:
        spdlog::info(local_msg.constData());
    }
}

void setLogger()
{
    // ファイルパス設定.
    auto dirPath = Config::valueAsString(KEY_LOG_OUTPUT_DIR);
    if(*dirPath.end() != '/')
        dirPath += '/';
    auto filePath =  QString(dirPath + THIS_PACKAGE_NAME + ".log");

    spdlog::init_thread_pool(8192, 4);
    auto logger = spdlog::daily_logger_mt<spdlog::async_factory>(
                    "main_logger",
                    spdlog::filename_t(filePath.toStdString().c_str()),
                    Config::valueAsInt(KEY_LOG_ROTATE_HOURS), Config::valueAsInt(KEY_LOG_ROTATE_MINUTES),
                    false);
    spdlog::set_default_logger(logger);

    auto level = Config::valueAsString(KEY_LOG_LEVEL);
    if(level.toLower() == "info")
    {
        spdlog::set_level(spdlog::level::info);
    }
    else
    {
        spdlog::set_level(spdlog::level::debug);
    }
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e][%l][%t] %v");
}

}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        THIS_PACKAGE_NAME.toStdString(),
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    intball::setNode(node);

    Config::load(THIS_PACKAGE_NAME);

    setLogger();
    qInstallMessageHandler(logHandler);

    LOG_INFO() << "Launch " << THIS_PACKAGE_NAME << " node";

    QApplication a(argc, argv);

    // UI用フォントを設定する.
    QFont font("Roboto");
    font.setStyleHint(QFont::Monospace);
    a.setFont(font);

    GroundSystemMainWindow w;

    w.setStyleSheet(QString("background-color: %1;color: %2;")
                    .arg(Color::styleSheetRGB(Color::U1))
                    .arg(Color::styleSheetRGB(Color::F1)));

    // ウィンドウはフレームレスで、デスクトップ領域（タスクメニューを除く領域）を埋めるように表示する
    w.setWindowFlags(Qt::WindowType::FramelessWindowHint);
    w.show();
    auto availableGeometry = QGuiApplication::primaryScreen()->availableGeometry();
    w.setGeometry(availableGeometry.x(),
                  availableGeometry.y(),
                  availableGeometry.width(),
                  availableGeometry.height());

    // rviz2パネルの初期化をイベントループ開始後に遅延実行する.
    // rviz2と同様、QApplication::exec()でイベントループを実行し、
    // ROS 2のスピンはQTimerで周期的に行う.
    QTimer rosSpinTimer;
    QObject::connect(&rosSpinTimer, &QTimer::timeout, [&node]() {
        if (rclcpp::ok()) {
            rclcpp::spin_some(node);
        } else {
            QApplication::quit();
        }
    });
    rosSpinTimer.start(16); // ~60Hz

    // rviz2パネルの初期化はイベントループ開始直後にスケジュールする.
    // この時点で全ウィジェットのexpose/paintイベントが処理済みとなり、
    // OgreのSceneManagerが遅延初期化される.
    QTimer::singleShot(0, [&w]() {
        w.initializePages();
    });

    int ret = a.exec();

    spdlog::shutdown();
    rclcpp::shutdown();

    return ret;
}
