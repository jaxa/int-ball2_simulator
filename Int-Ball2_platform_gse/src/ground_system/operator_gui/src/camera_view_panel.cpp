#include <QVBoxLayout>
#include <rviz_common/config.hpp>
#include <rviz_common/display.hpp>
#include <rviz_default_plugins/view_controllers/fps/fps_view_controller.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/tool_manager.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/yaml_config_reader.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
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
     * ここでrviz_common::RenderPanelをnewすると,initialize関数内での表示サイズ調整が上手く動作しない場合がある.
     */
}

void CameraViewPanel::initialize(const QString& pathRvizConfig)
{
    createRenderPanel();
    initializeVisualizationManager(pathRvizConfig);
}

void CameraViewPanel::createRenderPanel()
{
    renderPanel_ = new rviz_common::RenderPanel();
    renderPanel_->setFixedSize(width(), height());

    layout_ = new QVBoxLayout(this);
    layout_->setSpacing(0);
    layout_->setContentsMargins(0, 0, 0, 0);
    layout_->addWidget(renderPanel_);
}

void CameraViewPanel::initializeVisualizationManager(const QString& pathRvizConfig)
{
    // rviz設定ファイルの読み込み.
    rviz_common::YamlConfigReader rvizConfigReader;
    rviz_common::Config config;
    rvizConfigReader.readFile(config, pathRvizConfig);
    if(rvizConfigReader.error())
    {
        throwIntBallConfigError(pathRvizConfig, rvizConfigReader.errorMessage());
    }

    static int nodeCounter = 0;
    auto nodeName = std::string("camera_view_rviz_") + std::to_string(nodeCounter++);
    auto ros_node_abs = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(nodeName);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    // rviz2 (VisualizationFrame) と同じ初期化順序:
    // VisualizationManager生成 → RenderPanel初期化 → Manager初期化
    manager_.reset(new rviz_common::VisualizationManager(renderPanel_, ros_node_abs, nullptr, clock));
    renderPanel_->initialize(manager_.get());
    manager_->initialize();

    // カスタムViewControllerとToolはpluginlibプラグインとして登録済み.
    // plugin_description.xmlで operator_gui/MainCamera, operator_gui/SetTargetOrientation として定義.

    manager_->load(config);

    // 独自Toolの設定.
    if(dynamic_cast<intball::MainCameraViewController *>(renderPanel_->getViewController()) == nullptr)
    {
        LOG_CRITICAL() << "Invalid controller type: .rviz file[" << pathRvizConfig << "]";
        throwIntBallConfigError(pathRvizConfig, "Invalid controller type");
    }
    connect(manager_->getToolManager()->getCurrentTool(), SIGNAL(updateCamera()),
            this, SIGNAL(changed()));

    // グリッドを表示.
    auto grindDisplay = manager_->createDisplay("rviz_default_plugins/Grid", "adjustable grid", true);
    grindDisplay->subProp("Plane Cell Count")->setValue(20);
    grindDisplay->subProp("Normal Cell Count")->setValue(0);

    //rvizパネルの更新開始.
    manager_->startUpdate();
}

void CameraViewPanel::startRendering()
{
    if (manager_) {
        manager_->startUpdate();
    }
}

void CameraViewPanel::stopRendering()
{
    if (manager_) {
        manager_->stopUpdate();
    }
}

QQuaternion CameraViewPanel::getQuaternion()
{
    intball::MainCameraViewController* controller = dynamic_cast<intball::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    // FIXED_FRAME(base)座標系からiss_body座標系に変換する.
    tf2::Quaternion q;
    q.setRPY(controller->subProp("Roll")->getValue().toDouble(),
             controller->subProp("Pitch")->getValue().toDouble(),
             controller->subProp("Yaw")->getValue().toDouble());
    auto result = getStaticBaseToIssBodyTransform().inverse() * q;
    return QQuaternion(result.w(), result.x(), result.y(), result.z());
}

void CameraViewPanel::setView(const QVector3D& position, const QQuaternion& quaternion)
{
    intball::MainCameraViewController* controller = dynamic_cast<intball::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    issBodyValuesToBaseFrameCameraPosition(controller, position, quaternion);
}

void CameraViewPanel::clear()
{
    intball::MainCameraViewController* controller = dynamic_cast<intball::MainCameraViewController *>(renderPanel_->getViewController());
    Q_ASSERT(controller != nullptr);

    controller->subProp("Position")->setValue(QVariant("0;0;0"));
    controller->subProp("Roll")->setValue(0.0);
    controller->subProp("Pitch")->setValue(0.0);
    controller->subProp("Yaw")->setValue(0.0);
}
