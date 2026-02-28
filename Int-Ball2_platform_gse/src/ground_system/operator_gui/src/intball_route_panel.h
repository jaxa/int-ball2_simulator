#ifndef INTBALL_ROUTE_PANEL_H
#define INTBALL_ROUTE_PANEL_H

#include <memory>
#include <QWidget>
#include <QVector3D>
#include <QQuaternion>
#include <QVBoxLayout>
#include <rviz_common/visualization_manager.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rviz_common
{
class Display;
class RenderPanel;
} // namespace rviz_common

namespace intball
{
class IntBallRoutePanel : public QWidget
{
    Q_OBJECT
public:
    explicit IntBallRoutePanel(QWidget *parent);
    virtual ~IntBallRoutePanel() {}
    void initialize(const QString pathRvizConfig);
    void createRenderPanel();
    void initializeVisualizationManager(const QString pathRvizConfig);
    void startRendering();
    void stopRendering();
    void setFocalPoint(const QVector3D& position);
    void setCameraSettings(const float distance, const float pitchRad, const float yawRad);

public slots:
    void setFocalPointDockingStation();
    void setFocalPointIntBall2();
    void setCamera1();
    void setCamera2();
    void setCamera3();
    void setCamera4();
    void setCamera5();

private:
    QWidget* buttonWidget_;
    QVBoxLayout* layout_;
    QScopedPointer<rviz_common::VisualizationManager> manager_;
    rviz_common::RenderPanel* renderPanel_;
    rviz_common::Display *displayRouteMarker_;

    void setCamera(const QString& keyPosition, const QString& keyDistance, const QString& keyPitchYaw);
};
} // namespace intball

#endif // INTBALL_ROUTE_PANEL_H
