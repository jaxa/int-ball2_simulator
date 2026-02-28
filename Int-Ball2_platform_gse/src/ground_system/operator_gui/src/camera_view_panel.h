#ifndef CAMERA_VIEW_PANEL_H
#define CAMERA_VIEW_PANEL_H

#include <QVBoxLayout>
#include <QWidget>
#include <QQuaternion>
#include <rviz_common/visualization_manager.hpp>

namespace rviz_common
{
class Display;
class RenderPanel;
} // namespace rviz_common

namespace intball
{

class CameraViewPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CameraViewPanel(QWidget *parent = nullptr);
    void initialize(const QString& pathRvizConfig);
    void createRenderPanel();
    void initializeVisualizationManager(const QString& pathRvizConfig);
    void startRendering();
    void stopRendering();
    void setView(const QVector3D& position, const QQuaternion& quaternion);
    QQuaternion getQuaternion();
    void clear();
signals:
    void changed();

public slots:
private:
    QVBoxLayout* layout_;
    QScopedPointer<rviz_common::VisualizationManager> manager_;
    rviz_common::RenderPanel* renderPanel_;
};

} // namespace intball

#endif // CAMERA_VIEW_PANEL_H
