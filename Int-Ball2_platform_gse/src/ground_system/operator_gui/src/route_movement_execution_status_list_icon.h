#ifndef ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ICON_H
#define ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ICON_H

#include <QWidget>
#include <QSvgRenderer>

namespace intball
{

class RouteMovementExecutionStatusListIcon : public QWidget
{
    Q_OBJECT
public:
    enum class Type
    {
        NONE,
        START,
        VIA,
        GOAL,
        ROUTE_LINE,
        ON_GOING_LINE,
    };

    RouteMovementExecutionStatusListIcon(QWidget* parent = nullptr);
    void setType(Type type);
    void setLatest(const bool on);

protected:
    virtual void paintEvent(QPaintEvent* event) override;

private:
    Type type_;
    bool isLatest_;
    QSvgRenderer* checkMark_;
};

}

#endif // ROUTE_MOVEMENT_EXECUTION_STATUS_LIST_ICON_H
