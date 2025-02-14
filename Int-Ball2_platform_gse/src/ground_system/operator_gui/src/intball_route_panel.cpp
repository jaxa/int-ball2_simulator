#include <QButtonGroup>
#include <QLabel>
#include <QPushButton>
#include <QQuaternion>
#include <QSpacerItem>
#include <QVector3D>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/render_panel.h>
#include <rviz/view_controller.h>
#include <rviz/visualization_manager.h>
#include <rviz/yaml_config_reader.h>
#include <tf/transform_broadcaster.h>
#include "exception/config_error.h"
#include "gui_color.h"
#include "intball_route_panel.h"
#include "operator_gui_config.h"
#include "qdebug_custom.h"
#include "ros_common.h"
#include "rviz_utils.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

IntBallRoutePanel::IntBallRoutePanel(QWidget *parent)
    : QWidget(parent)
{
    /*
     * RenderPanelの生成はinitializeで実施する.
     */
}

void IntBallRoutePanel::initialize(const QString pathRvizConfig)
{
    // ボタン.
    buttonWidget_ = new QWidget(this);
    buttonWidget_->setAttribute(Qt::WA_TranslucentBackground, true);
    QHBoxLayout* buttonLayout = new QHBoxLayout(buttonWidget_);
    buttonLayout->setMargin(0);
    buttonLayout->setSpacing(0);

    // 両端にスペースを挿入して中央揃えとする.
    buttonLayout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Preferred));

    QLabel* buttonLabel = new QLabel(buttonWidget_);
    buttonLabel->setAttribute(Qt::WA_TranslucentBackground, true);
    buttonLabel->setText("View:  ");
    buttonLayout->addWidget(buttonLabel);

    const QString buttonStyleBegin = QString("*{"
                                     "background-color: %1;"
                                     "color: %2;"
                                     "border: 1px solid %3;"
                                     "border-right: 0;"
                                     "border-top-left-radius: 2px;"
                                     "border-bottom-left-radius: 2px;"
                                     "padding: 5px 15px;"
                                     "}"
                                     "*:pressed,*:checked{"
                                     "background-color: %4;"
                                     "}")
                                     .arg(Color::styleSheetRGB(Color::B1))
                                     .arg(Color::styleSheetRGB(Color::B2))
                                     .arg(Color::styleSheetRGB(Color::B_OUTSET_BORDER))
                                     .arg(Color::styleSheetRGB(Color::B_PRESSED_BACKGROUND));
    const QString buttonStyle = QString("*{"
                                        "background-color: %1;"
                                        "color: %2;"
                                        "border: 1px solid %3;"
                                        "border-right: 0;"
                                        "padding: 5px 15px;"
                                        "}"
                                        "*:pressed,*:checked{"
                                        "background-color: %4;"
                                        "}")
                                .arg(Color::styleSheetRGB(Color::B1))
                                .arg(Color::styleSheetRGB(Color::B2))
                                .arg(Color::styleSheetRGB(Color::B_OUTSET_BORDER))
                                .arg(Color::styleSheetRGB(Color::B_PRESSED_BACKGROUND));
    const QString buttonStyleEnd = QString("*{"
                                           "background-color: %1;"
                                           "color: %2;"
                                           "border: 1px solid %3;"
                                           "border-right: 0;"
                                           "border-top-right-radius: 2px;"
                                           "border-bottom-right-radius: 2px;"
                                           "padding: 5px 15px;"
                                           "}"
                                           "*:pressed,*:checked{"
                                           "background-color: %4;"
                                           "}")
                                   .arg(Color::styleSheetRGB(Color::B1))
                                   .arg(Color::styleSheetRGB(Color::B2))
                                   .arg(Color::styleSheetRGB(Color::B_OUTSET_BORDER))
                                   .arg(Color::styleSheetRGB(Color::B_PRESSED_BACKGROUND));


    QPushButton* button1 = new QPushButton(this);
    button1->setText("1");
    button1->setCheckable(true);
    button1->setStyleSheet(buttonStyleBegin);
    buttonLayout->addWidget(button1);
    QPushButton* button2 = new QPushButton(this);
    button2->setText("2");
    button2->setCheckable(true);
    button2->setStyleSheet(buttonStyle);
    buttonLayout->addWidget(button2);
    QPushButton* button3 = new QPushButton(this);
    button3->setText("3");
    button3->setCheckable(true);
    button3->setStyleSheet(buttonStyle);
    buttonLayout->addWidget(button3);
    QPushButton* button4 = new QPushButton(this);
    button4->setText("4");
    button4->setCheckable(true);
    button4->setStyleSheet(buttonStyle);
    buttonLayout->addWidget(button4);
    QPushButton* button5 = new QPushButton(this);
    button5->setText("5");
    button5->setCheckable(true);
    button5->setStyleSheet(buttonStyle);
    buttonLayout->addWidget(button5);
    QPushButton* button6 = new QPushButton(this);
    button6->setText("6");
    button6->setCheckable(true);
    button6->setStyleSheet(buttonStyleEnd);
    buttonLayout->addWidget(button6);

    // 両端にスペースを挿入して中央揃えとする.
    buttonLayout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Preferred));

    QButtonGroup* viewButtonGroup = new QButtonGroup(this);
    viewButtonGroup->addButton(button1);
    viewButtonGroup->addButton(button2);
    viewButtonGroup->addButton(button3);
    viewButtonGroup->addButton(button4);
    viewButtonGroup->addButton(button5);
    viewButtonGroup->addButton(button6);
    button1->setChecked(true);

    // ボタンのイベント.
    connect(button1, &QAbstractButton::clicked, this, &IntBallRoutePanel::setCamera1);
    connect(button2, &QAbstractButton::clicked, this, &IntBallRoutePanel::setCamera2);
    connect(button3, &QAbstractButton::clicked, this, &IntBallRoutePanel::setCamera3);
    connect(button4, &QAbstractButton::clicked, this, &IntBallRoutePanel::setCamera4);
    connect(button5, &QAbstractButton::clicked, this, &IntBallRoutePanel::setCamera5);
    connect(button6, &QAbstractButton::clicked, this, &IntBallRoutePanel::setFocalPointIntBall2);

    // rvizパネル.
    renderPanel_ = new rviz::RenderPanel();
    renderPanel_->setFixedSize(width(), height() - button1->height() - 10);

    layout_ = new QVBoxLayout();
    layout_->setSpacing(10);
    layout_->setMargin(0);
    layout_->addWidget(renderPanel_);
    layout_->addWidget(buttonWidget_);
    setLayout(layout_);

    // rviz設定ファイルの読み込み.
    rviz::YamlConfigReader rvizConfigReader;
    rviz::Config config;
    rvizConfigReader.readFile(config, pathRvizConfig);
    if(rvizConfigReader.error())
    {
        throwIntBallConfigError(pathRvizConfig, rvizConfigReader.errorMessage());
    }
    manager_.reset(new rviz::VisualizationManager(renderPanel_, nullptr, getTransformListener()));
    manager_->initialize();
    renderPanel_->initialize(manager_->getSceneManager(), manager_.get());
    manager_->load(config);

    // GUI設定ファイルの内容でカメラ表示を初期化する.
    setCamera1();

    //rvizパネルの更新開始.
    manager_->startUpdate();
}

void IntBallRoutePanel::setFocalPointDockingStation()
{
    // Fixed frameはbase
    manager_->setFixedFrame(QString::fromStdString(rosframe::BASE));
    // iss_bodyフレームの原点＝ドッキングステーション
    resetViewController(renderPanel_->getViewController(), QString::fromStdString(rosframe::ISS_BODY));
}

void IntBallRoutePanel::setFocalPointIntBall2()
{
    manager_->setFixedFrame(QString::fromStdString(rosframe::BODY));
    auto controller = renderPanel_->getViewController();
    resetViewController(renderPanel_->getViewController(),  QString::fromStdString(rosframe::BODY));
    controller->subProp("Distance")->setValue(1.5);
    controller->subProp("Pitch")->setValue(0.2);
    controller->subProp("Yaw")->setValue(0.5);
}

void IntBallRoutePanel::setFocalPoint(const QVector3D& position)
{
    // Fixed frameはbase
    manager_->setFixedFrame(QString::fromStdString(rosframe::BASE));
    changeTargetFrame(renderPanel_->getViewController(), QString::fromStdString(rosframe::BASE));
    issBodyValuesToIssBodyFrameCameraFocalPoint(renderPanel_->getViewController(), position);
}

void IntBallRoutePanel::setCameraSettings(const float distance, const float pitchRad, const float yawRad)
{
    auto controller = renderPanel_->getViewController();
    controller->subProp("Distance")->setValue(distance);
    controller->subProp("Pitch")->setValue(pitchRad);
    controller->subProp("Yaw")->setValue(yawRad);
}

void IntBallRoutePanel::setCamera(const QString& keyPosition, const QString& keyDistance, const QString& keyPitchYaw)
{
    try
    {
        QStringList positionValues = Config::valueAsStringList(keyPosition);
        QVector3D position = QVector3D(positionValues.at(0).toFloat(), positionValues.at(1).toFloat(), positionValues.at(2).toFloat());
        float distance = Config::valueAsFloat(keyDistance);
        QStringList pitchYawValues = Config::valueAsStringList(keyPitchYaw);
        float pitch = pitchYawValues.at(0).toFloat();
        float yaw = pitchYawValues.at(1).toFloat();

        setFocalPoint(position);
        setCameraSettings(distance, pitch, yaw);
    }
    catch(...)
    {
        LOG_WARNING() << "Setting value format error: " <<
                         keyPosition << " or " <<
                         keyDistance << " or " <<
                         keyPitchYaw;
    }
}

void IntBallRoutePanel::setCamera1()
{
    setCamera(KEY_RVIZ_PANEL_CAMERA_PANEL_1_POSITION_XYZ,
              KEY_RVIZ_PANEL_CAMERA_PANEL_1_DISTANCE,
              KEY_RVIZ_PANEL_CAMERA_PANEL_1_PITCH_YAW);
}

void IntBallRoutePanel::setCamera2()
{
    setCamera(KEY_RVIZ_PANEL_CAMERA_PANEL_2_POSITION_XYZ,
              KEY_RVIZ_PANEL_CAMERA_PANEL_2_DISTANCE,
              KEY_RVIZ_PANEL_CAMERA_PANEL_2_PITCH_YAW);
}

void IntBallRoutePanel::setCamera3()
{
    setCamera(KEY_RVIZ_PANEL_CAMERA_PANEL_3_POSITION_XYZ,
              KEY_RVIZ_PANEL_CAMERA_PANEL_3_DISTANCE,
              KEY_RVIZ_PANEL_CAMERA_PANEL_3_PITCH_YAW);
}

void IntBallRoutePanel::setCamera4()
{
    setCamera(KEY_RVIZ_PANEL_CAMERA_PANEL_4_POSITION_XYZ,
              KEY_RVIZ_PANEL_CAMERA_PANEL_4_DISTANCE,
              KEY_RVIZ_PANEL_CAMERA_PANEL_4_PITCH_YAW);
}


void IntBallRoutePanel::setCamera5()
{
    setCamera(KEY_RVIZ_PANEL_CAMERA_PANEL_5_POSITION_XYZ,
              KEY_RVIZ_PANEL_CAMERA_PANEL_5_DISTANCE,
              KEY_RVIZ_PANEL_CAMERA_PANEL_5_PITCH_YAW);
}

