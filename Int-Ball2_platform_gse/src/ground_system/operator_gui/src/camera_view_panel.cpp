#include <QVBoxLayout>
#include <rviz/config.h>
#include <rviz/display.h>
#include <rviz/default_plugin/view_controllers/fps_view_controller.h>
#include <rviz/render_panel.h>
#include <rviz/tool_manager.h>
#include <rviz/visualization_manager.h>
#include <rviz/view_manager.h>
#include <rviz/yaml_config_reader.h>
#include <tf/transform_datatypes.h>
#include "camera_view_panel.h"
#include "exception/config_error.h"
#include "main_camera_view_controller.h"
#include "qdebug_custom.h"
#include "rviz_utils.h"
#include "ros_common.h"
#include "set_target_orientation_tool.h"
#include "utils.h"

using namespace intball;

CameraViewPanel::CameraViewPanel(QWidget *parent) : QWidget(parent)
{
    /*
     * ここでrviz::RenderPanelをnewすると,initialize関数内での表示サイズ調整が上手く動作しない場合がある.
     */
}

void CameraViewPanel::initialize(const QString& pathRvizConfig)
{
    renderPanel_ = new rviz::RenderPanel();
    renderPanel_->setFixedSize(width(), height());

    layout_ = new QVBoxLayout(this);
    layout_->setSpacing(0);
    layout_->setMargin(0);
    layout_->addWidget(renderPanel_);


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

    // 独自ViewControllerの定義.
    manager_->getViewManager()->getFactory()->addBuiltInClass(
        "rviz",
        "MainCamera",
        "",
        &rviz::newMainCameraViewController);

    // 独自Toolの定義.
    manager_->getToolManager()->getFactory()->addBuiltInClass(
        "rviz",
        "SetTargetOrientation",
        "",
        &rviz::newSetTargetOrientationTool);

    manager_->load(config);

    // 独自Toolの設定.
    if(dynamic_cast<rviz::MainCameraViewController *>(renderPanel_->getViewController()) == nullptr)
    {
        LOG_CRITICAL() << "Invalid controller type: .rviz file[" << pathRvizConfig << "]";
        throwIntBallConfigError(pathRvizConfig, "Invalid controller type");
    }
    connect(manager_->getToolManager()->getCurrentTool(), SIGNAL(updateCamera()),
            this, SIGNAL(changed()));

    // グリッドを表示.
    auto grindDisplay = manager_->createDisplay("rviz/Grid", "adjustable grid", true);
    grindDisplay->subProp("Plane Cell Count")->setValue(20);
    grindDisplay->subProp("Normal Cell Count")->setValue(0);

    //rvizパネルの更新開始.
    manager_->startUpdate();
}

tf::Quaternion CameraViewPanel::getQuaternion()
{
    rviz::MainCameraViewController* controller = dynamic_cast<rviz::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    // FIXED_FRAME(base)座標系からiss_body座標系に変換する.
    auto returnValue = tf::createQuaternionFromRPY(controller->subProp("Roll")->getValue().toDouble(),
                       controller->subProp("Pitch")->getValue().toDouble(),
                       controller->subProp("Yaw")->getValue().toDouble());
    return getStaticBaseToIssBodyTransform().inverse() * returnValue;
}

void CameraViewPanel::setView(const QVector3D& position, const QQuaternion& quaternion)
{
    rviz::MainCameraViewController* controller = dynamic_cast<rviz::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    issBodyValuesToBaseFrameCameraPosition(controller, position, quaternion);
}

void CameraViewPanel::clear()
{
    rviz::MainCameraViewController* controller = dynamic_cast<rviz::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    controller->subProp("Position")->setValue(QVariant("0;0;0"));
    controller->subProp("Roll")->setValue(0.0);
    controller->subProp("Pitch")->setValue(0.0);
    controller->subProp("Yaw")->setValue(0.0);
}
