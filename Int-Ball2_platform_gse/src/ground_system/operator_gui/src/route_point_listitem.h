#ifndef ROUTE_POINT_LISTITEM_H
#define ROUTE_POINT_LISTITEM_H

#include <QAction>
#include <QLabel>
#include <QMenu>
#include <QQuaternion>
#include <QVector3D>
#include <QWidget>

namespace intball
{

class RoutePointListItem : public QWidget
{
    Q_OBJECT
public:
    const static int FIXED_HEIGHT;
    explicit RoutePointListItem(QWidget *parent = nullptr, const QVector3D& position = QVector3D(), const QQuaternion& orientation = QQuaternion());
    explicit RoutePointListItem(const intball::RoutePointListItem& item);
    virtual ~RoutePointListItem() override {}
    void setPoint(const QVector3D& position, const QQuaternion& orientation);
    void setEditable(bool editable);
    void changeHighlight(bool flag);
    bool equals(const QVector3D &position, const QQuaternion &orientation);
    void getValues(QVector3D &position, QQuaternion &orientation);
    bool operator==(const intball::RoutePointListItem& item) const;
    inline bool operator!=(const intball::RoutePointListItem& item) const
    {
        return !(*this == item);
    }
    bool isHighlight();
signals:
    void selected();
    void deleteSelected();
protected:
    void contextMenuEvent(QContextMenuEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
private:
    QMenu* menu_;
    QAction* deleteAction_;
    QLabel* label_;
    QVector3D position_;
    QQuaternion orientation_;
    bool editable_;
    bool highlight_;
    void changeDisplay();
};

} // namespace intball
#endif // ROUTE_POINT_LISTITEM_H
