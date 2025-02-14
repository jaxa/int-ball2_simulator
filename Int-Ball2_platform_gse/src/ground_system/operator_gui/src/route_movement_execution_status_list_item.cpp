#include "route_movement_execution_status_list_item.h"
#include <QGridLayout>
#include <QLabel>
#include <QPaintEngine>
#include <QPainter>
#include <QPropertyAnimation>
#include <QString>
#include "additional_property_label.h"
#include "gui_color.h"
#include "model/route_information.h"
#include "qdebug_custom.h"
#include "route_movement_execution_status_list_icon.h"
#include "utils.h"

using namespace intball;

RouteMovementExecutionStatusListItem::RouteMovementExecutionStatusListItem(QWidget *parent) :
    QWidget(parent), isEnabled(false), isStatusImageOnly_(false), isStart_(false), isGoal_(false), onGoing_(false)
{
    setStyleSheet(parent->styleSheet());

    QGridLayout* gridLayout = new QGridLayout(this);
    gridLayout->setContentsMargins(0, 0, 0, 0);
    gridLayout->setSpacing(0);

    labelIcon_ = new RouteMovementExecutionStatusListIcon(this);

    labelPosition_ = new QLabel(this);
    labelPosition_->setStyleSheet(parent->styleSheet());
    labelAttitude_ = new QLabel(this);
    labelAttitude_->setStyleSheet(parent->styleSheet());
    labelWait_ = new AdditionalPropertyLabel(this);
    labelWait_->setStyleSheet(parent->styleSheet());
    labelWait_->setFixedWidth(80);

    gridLayout->addWidget(labelIcon_, 0, 0);
    gridLayout->addWidget(labelPosition_, 0, 1);
    gridLayout->addWidget(labelAttitude_, 0, 2);
    gridLayout->addWidget(labelWait_, 0, 3);

    waitLabelAnimation_ = new QPropertyAnimation(labelWait_, "fontColor");
    waitLabelAnimation_->setLoopCount(-1);
}

void RouteMovementExecutionStatusListItem::clear()
{
    labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::NONE);
    labelPosition_->clear();
    labelAttitude_->clear();
    labelWait_->clear();
}

void RouteMovementExecutionStatusListItem::assign(const RouteInformation* routeInformation_,
                                                  const int index)
{
    auto targetPoint = routeInformation_->data(index);

    isStart_ = (index == 0);
    isGoal_ = (index == routeInformation_->goalIndex());
    Q_ASSERT((isStart_ == false) || (isGoal_ == false));

    labelPosition_->setText(QString(" %1    %2    %3")
                    .arg(QString::number(static_cast<qreal>(targetPoint.position().x()), 'f', 2))
                    .arg(QString::number(static_cast<qreal>(targetPoint.position().y()), 'f', 2))
                    .arg(QString::number(static_cast<qreal>(targetPoint.position().z()), 'f', 2)));

    qreal roll, pitch, yaw;
    getRPY(targetPoint.orientation(), roll, pitch, yaw);
    labelAttitude_->setText(QString(" %1    %2    %3")
                    .arg(QString::number(roundDegree(roll)))
                    .arg(QString::number(roundDegree(pitch)))
                    .arg(QString::number(roundDegree(yaw))));

    if(!isStart_ && !isGoal_){
        labelWait_->setText(QString("%1sec").arg(targetPoint.waitSecond()));
        labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::VIA);
    }else{
        labelWait_->setText("");
    }

    if(isStart_)
    {
        labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::START);
    }
    else if(isGoal_)
    {
        labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::GOAL);
    }

    // 新規の点が設定された場合は表示をリセットする.
    setLatest(false);
    setWaitingState(false);
}

void RouteMovementExecutionStatusListItem::setIconOnly(bool onGoing)
{
    onGoing_ = onGoing;
    if(onGoing)
    {
        labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::ON_GOING_LINE);
    }
    else
    {
        labelIcon_->setType(RouteMovementExecutionStatusListIcon::Type::ROUTE_LINE);
    }
}

void RouteMovementExecutionStatusListItem::setWaitingState(const bool on)
{
    if(on)
    {
        // 待機秒数のアニメーション開始.
        waitLabelAnimation_->setDuration(LABEL_ANIMATION_DURATION_MSECS);
        waitLabelAnimation_->setKeyValueAt(0, Color::F1);
        waitLabelAnimation_->setKeyValueAt(0.5, Color::U_HIGHLIGHT);
        waitLabelAnimation_->setKeyValueAt(1, Color::F1);
        waitLabelAnimation_->start();
    }
    else
    {
        // 待機秒数のアニメーション終了.
        waitLabelAnimation_->stop();
        labelWait_->setStyleSheet(styleSheet());
    }
}

void RouteMovementExecutionStatusListItem::setLatest(const bool on)
{
    labelIcon_->setLatest(on);
}
