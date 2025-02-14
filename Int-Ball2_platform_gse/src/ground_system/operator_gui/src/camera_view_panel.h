#ifndef CAMERA_VIEW_PANEL_H
#define CAMERA_VIEW_PANEL_H

#include <QVBoxLayout>
#include <QWidget>
#include <rviz/visualization_manager.h>

namespace rviz
{
class Display;
class RenderPanel;
} // namespace rviz

namespace tf
{
class Quaternion;
} // namespace tf

namespace intball
{

class CameraViewPanel : public QWidget
{
    Q_OBJECT
public:
    explicit CameraViewPanel(QWidget *parent = nullptr);
    void initialize(const QString& pathRvizConfig);
    void setView(const QVector3D& position, const QQuaternion& quaternion);
    tf::Quaternion getQuaternion();
    void clear();
signals:
    void changed();

public slots:
private:
    QVBoxLayout* layout_;
    QScopedPointer<rviz::VisualizationManager> manager_;
    rviz::RenderPanel* renderPanel_;
};

} // namespace intball

#endif // CAMERA_VIEW_PANEL_H
