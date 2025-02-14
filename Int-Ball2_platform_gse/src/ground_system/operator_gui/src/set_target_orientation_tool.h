#ifndef IB2_SET_TARGET_ORIENTATION_TOOL_H
#define IB2_SET_TARGET_ORIENTATION_TOOL_H
#include <QObject>
#include <rviz/default_plugin/tools/move_tool.h>

namespace rviz
{
class ViewportMouseEvent;
class RenderPanel;
class FPSViewController;
} // namespace rviz
class QQuaternion;

namespace rviz
{

rviz::Tool *newSetTargetOrientationTool();

class SetTargetOrientationTool : public MoveTool
{
    Q_OBJECT
public:
    SetTargetOrientationTool() {}

    virtual void activate() {}
    virtual void deactivate() {}

    virtual int processMouseEvent(ViewportMouseEvent &event);
    virtual int processKeyEvent(QKeyEvent *event, RenderPanel *panel);
signals:
    void updateCamera();
};

} // namespace rviz

#endif
