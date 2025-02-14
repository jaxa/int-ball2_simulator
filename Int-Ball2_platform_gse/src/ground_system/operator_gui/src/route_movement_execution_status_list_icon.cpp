#include "route_movement_execution_status_list_icon.h"
#include <QPainter>
#include <QSvgRenderer>
#include "gui_color.h"

using namespace intball;

RouteMovementExecutionStatusListIcon::RouteMovementExecutionStatusListIcon(QWidget* parent)
    : QWidget(parent), type_(Type::NONE), isLatest_(false)
{
    setFixedSize(40, 40);
    checkMark_ = new QSvgRenderer(QString(":/ground_system/image/check_fill_white.svg"), this);
}

void RouteMovementExecutionStatusListIcon::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

    switch(type_)
    {
    case Type::NONE:
        //DO NOTHING
        break;
    case Type::START:
        // 線.
        painter.setPen(QPen(Color::F1, 0, Qt::DashLine));
        painter.drawLine(width() / 2, height() / 2,
                         width() / 2, height());
        // 円.
        painter.setPen(QPen(Color::F2, 0, Qt::SolidLine));
        painter.setBrush(Color::U2);
        painter.drawEllipse(5, 5, width()-10, height()-10);
        if(isLatest_)
        {
            checkMark_->render(&painter, QRect(5, 0, width()-5, height()-5));
        }
        break;
    case Type::VIA:
        // 線.
        painter.setPen(QPen(Color::F1, 0, Qt::DashLine));
        painter.drawLine(width() / 2, 0,
                         width() / 2, height());
        // 円.
        painter.setPen(QPen(Color::F2, 0, Qt::SolidLine));
        painter.setBrush(Color::U2);
        painter.drawEllipse(10, 10, width() - 20, height() - 20);
        if(isLatest_)
        {
            checkMark_->render(&painter, QRect(5, 0, width()-5, height()-5));
        }
        break;
    case Type::GOAL:
        // 線.
        painter.setPen(QPen(Color::F1, 0, Qt::DashLine));
        painter.drawLine(width() / 2, 0,
                         width() / 2, height() / 2);
        // 円.
        painter.setPen(QPen(Color::F2, 0, Qt::SolidLine));
        painter.setBrush(Color::U2);
        painter.drawEllipse(5, 5, width()-10, height()-10);
        if(isLatest_)
        {
            checkMark_->render(&painter, QRect(5, 0, width()-5, height()-5));
        }
        break;
    case Type::ROUTE_LINE:
        // 線.
        painter.setPen(QPen(Color::F1, 0, Qt::DashLine));
        painter.drawLine(width() / 2, 0,
                         width() / 2, height());
        break;
    case Type::ON_GOING_LINE:
        // 線.
        painter.setPen(QPen(Color::F1, 0, Qt::DashLine));
        painter.drawLine(width() / 2, 0,
                         width() / 2, height());

        // 三角.
        painter.setPen(QPen());
        painter.setBrush(Color::F2);
        QPointF triangle1 = QPointF(5, 10);
        QPointF triangle2 = QPointF(width()-5, 10);
        QPointF triangle3 = QPointF(width() / 2, 28);
        const QPointF trianglePoints[3] = {triangle1, triangle2, triangle3};
        painter.drawPolygon(trianglePoints, 3);

        break;
    }
}

void RouteMovementExecutionStatusListIcon::setType(Type type)
{
    type_ = type;
    update();
}

void RouteMovementExecutionStatusListIcon::setLatest(const bool on)
{
    isLatest_ = on;
    update();
}
