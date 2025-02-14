#include <QApplication>
#include <QPushButton>
#include <ros/ros.h>
#include "platform_main_window.h"
#include "platform_gui_config.h"
#include "gui_color.h"
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
    ros::init(argc, argv, THIS_PACKAGE_NAME.toStdString(), ros::init_options::AnonymousName);
    intball::getNodeHandle();

    Config::load(THIS_PACKAGE_NAME);

    setLogger();
    qInstallMessageHandler(logHandler);

    LOG_INFO() << "Launch " << THIS_PACKAGE_NAME << " node";

    QApplication a(argc, argv);

    // UI用フォントを設定する.
    QFont font("Roboto");
    font.setStyleHint(QFont::Monospace);
    a.setFont(font);

    LOG_INFO() << "Set the application window";
    PlatformMainWindow w;
    w.setStyleSheet(QString("background-color: %1;color: %2;")
                    .arg(Color::styleSheetRGB(Color::U1))
                    .arg(Color::styleSheetRGB(Color::F1)));
    w.setWindowFlags(Qt::WindowType::CustomizeWindowHint | Qt::WindowType::WindowCloseButtonHint | Qt::WindowType::WindowMaximizeButtonHint);
    w.showMaximized();

    ros::Rate rate(30);
    while(ros::ok())
    {
        ros::spinOnce();
        a.processEvents();
        rate.sleep();
    }
    //spdlog::shutdown();

    return 0;
}
