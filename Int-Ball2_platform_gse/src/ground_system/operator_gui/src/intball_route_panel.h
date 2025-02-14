#ifndef INTBALL_ROUTE_PANEL_H
#define INTBALL_ROUTE_PANEL_H


#include <boost/shared_ptr.hpp>
#include <QWidget>
#include <QVector3D>
#include <QQuaternion>
#include <QVBoxLayout>
#include <rviz/visualization_manager.h>
#include <ros/ros.h>

namespace rviz
{
class Display;
class RenderPanel;
} // namespace rviz

namespace tf
{
class TransformListener;
class StampedTransform;
} // namespace tf

namespace intball
{
class IntBallRoutePanel : public QWidget
{
    Q_OBJECT
public:
    explicit IntBallRoutePanel(QWidget *parent);
    virtual ~IntBallRoutePanel() {}
    void initialize(const QString pathRvizConfig);
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
    QScopedPointer<rviz::VisualizationManager> manager_;
    rviz::RenderPanel* renderPanel_;
    rviz::Display *displayRouteMarker_;

    void setCamera(const QString& keyPosition, const QString& keyDistance, const QString& keyPitchYaw);
};
} // namespace intball

#endif // INTBALL_ROUTE_PANEL_H
