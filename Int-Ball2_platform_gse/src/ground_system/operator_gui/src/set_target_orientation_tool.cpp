#include <QQuaternion>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/render_panel.hpp>
#include "main_camera_view_controller.h"
#include "qdebug_custom.h"
#include "set_target_orientation_tool.h"
#include "utils.h"

using namespace std;

namespace intball
{

rviz_common::Tool *newSetTargetOrientationTool()
{
    return new SetTargetOrientationTool();
}

int SetTargetOrientationTool::processMouseEvent(rviz_common::ViewportMouseEvent &event)
{
    // 左クリックのみ処理する.
    if (event.panel->getViewController() && (event.left() || event.leftUp()) && (event.modifiers == Qt::NoModifier))
    {
        // カメラの移動処理.
        event.panel->getViewController()->handleMouseEvent(event);
        setCursor(event.panel->getViewController()->getCursor());

        if (event.leftUp())
        {
            MainCameraViewController *controller = dynamic_cast<MainCameraViewController *>(event.panel->getViewController());
            Q_ASSERT(controller != nullptr);
            emit updateCamera();
        }
    }
    return 0;
}

int SetTargetOrientationTool::processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel)
{
    Q_UNUSED(event);
    Q_UNUSED(panel);
    return 0;
}

} // namespace intball

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(intball::SetTargetOrientationTool, rviz_common::Tool)
