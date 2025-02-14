#include <QQuaternion>
#include <rviz/view_controller.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/render_panel.h>
#include "main_camera_view_controller.h"
#include "qdebug_custom.h"
#include "set_target_orientation_tool.h"
#include "utils.h"

using namespace std;

namespace rviz
{

Tool *newSetTargetOrientationTool()
{
    return new SetTargetOrientationTool();
}

int SetTargetOrientationTool::processMouseEvent(ViewportMouseEvent &event)
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

int SetTargetOrientationTool::processKeyEvent(QKeyEvent *event, RenderPanel *panel)
{
    Q_UNUSED(event);
    Q_UNUSED(panel);
    return 0;
}

} // namespace rviz
