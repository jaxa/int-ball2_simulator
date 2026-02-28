#ifndef IB2_SET_TARGET_ORIENTATION_TOOL_H
#define IB2_SET_TARGET_ORIENTATION_TOOL_H
#include <QObject>
#include <rviz_default_plugins/tools/move/move_tool.hpp>

class QQuaternion;

namespace intball
{

rviz_common::Tool *newSetTargetOrientationTool();

class SetTargetOrientationTool : public rviz_default_plugins::tools::MoveTool
{
    Q_OBJECT
public:
    SetTargetOrientationTool() {}

    void activate() override {}
    void deactivate() override {}

    int processMouseEvent(rviz_common::ViewportMouseEvent &event) override;
    int processKeyEvent(QKeyEvent *event, rviz_common::RenderPanel *panel) override;
signals:
    void updateCamera();
};

} // namespace intball

#endif
