#include <QContextMenuEvent>
#include <QHBoxLayout>
#include <QMenu>
#include <QString>
#include <QVector3D>
#include "gui_color.h"
#include "qdebug_custom.h"
#include "route_point_listitem.h"

using namespace intball;

const int RoutePointListItem::FIXED_HEIGHT = 35;

RoutePointListItem::RoutePointListItem(QWidget *parent, const QVector3D& position, const QQuaternion& orientation) :
    QWidget(parent), position_(position), orientation_(orientation), editable_(true), highlight_(false)
{
    setFixedHeight(FIXED_HEIGHT);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Fixed);
    setAutoFillBackground(true);

    // 右クリックメニュー向けの設定
    menu_ = new QMenu(this);
    setContextMenuPolicy(Qt::DefaultContextMenu);
    menu_->setStyleSheet("QMenu{"
                         "background-color: rgb(255, 255, 255);"
                         "border: 1px solid rgb(200, 200, 200);"
                         "color: rgb(0, 0, 0);"
                         "}"
                         "QMenu:item{"
                         "border: 1px solid transparent;"
                         "padding: 2px 25px 2px 20px; "
                         "}"
                         "QMenu:item:selected{"
                         "background-color: rgba(100, 100, 100, 50);"
                         "}");

    // 削除に関してはRoutePointListで実施するため, シグナルを連結して外部に削除イベントを通知する.
    deleteAction_ = new QAction(tr("&delete"), this);
    connect(deleteAction_, &QAction::triggered, this, &RoutePointListItem::deleteSelected);

    label_ = new QLabel(this);
    label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    label_->setAlignment(Qt::AlignCenter);

    QHBoxLayout* layout = new QHBoxLayout();
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);
    layout->addWidget(label_);
    setLayout(layout);

    changeHighlight(false);
    changeDisplay();
}

void RoutePointListItem::setPoint(const QVector3D& position, const QQuaternion& orientation)
{
    position_ = position;
    orientation_ = orientation;
    changeDisplay();
}

void RoutePointListItem::setEditable(bool editable)
{
    editable_ = editable;
}

inline void RoutePointListItem::changeDisplay()
{
    label_->setText(QString(" %1    %2    %3")
                    .arg(QString::number(static_cast<qreal>(position_.x()), 'f', 2))
                    .arg(QString::number(static_cast<qreal>(position_.y()), 'f', 2))
                    .arg(QString::number(static_cast<qreal>(position_.z()), 'f', 2)));
}

void RoutePointListItem::contextMenuEvent(QContextMenuEvent *event)
{
    if(editable_)
    {
        menu_->addAction(deleteAction_);
        menu_->exec(event->globalPos());
    }
}

void RoutePointListItem::mousePressEvent(QMouseEvent *event)
{
    if(event->button() == Qt::MouseButton::LeftButton)
    {
        emit selected();
    }
}

void RoutePointListItem::changeHighlight(bool flag)
{
    if(flag)
    {
        // ハイライト表示.
        setStyleSheet(QString("background-color: %1")
                      .arg(Color::styleSheetRGB(Color::U_HIGHLIGHT)));
    }
    else
    {
        // ハイライト解除.
        setStyleSheet("background-color: rgba(0, 0, 0, 0)");
    }
    highlight_ = flag;
}

bool RoutePointListItem::operator==(const RoutePointListItem& item) const
{
    return (position_ == item.position_) && (orientation_ == item.orientation_);
}

bool RoutePointListItem::equals(const QVector3D &position, const QQuaternion &orientation)
{
    return (position_ == position) && (orientation_ == orientation);
}

void RoutePointListItem::getValues(QVector3D &position, QQuaternion &orientation)
{
    position.setX(position_.x());
    position.setY(position_.y());
    position.setZ(position_.z());
    orientation.setVector(orientation_.vector());
    orientation.setScalar(orientation_.scalar());
}

bool RoutePointListItem::isHighlight()
{
    return highlight_;
}
