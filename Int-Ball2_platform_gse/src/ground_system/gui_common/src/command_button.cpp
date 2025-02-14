#include "command_button.h"
#include <QPainter>
#include "gui_color.h"

using namespace intball;

CommandButton::CommandButton(QWidget* parent)
    :QPushButton(parent), currentStatus_(CommandButton::STATE_OFF),
      isAdditionalStateExists_(false), isStatusInitialized_(false),
      isWaiting_(false), targetState_(false)
{

}

void CommandButton::paintEvent(QPaintEvent *event)
{
    QPushButton::paintEvent(event);

    QPainter painter(this);

    int paddingBottom = height() / 10 * 2;
    if(isEnabled())
    {
        painter.setPen(QPen(QBrush(Color::B4, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
    }
    else
    {
        painter.setPen(QPen(QBrush(Color::B1, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
    }
    painter.drawLine(10, height() - paddingBottom, width() - 10, height() - paddingBottom);
}

void CommandButton::setAdditionalState(const QString text, const QIcon icon)
{
    defaultText_ = this->text();
    defaultIcon_ = this->icon();
    additionalText_ = text;
    additionalIcon_ = icon;
    isAdditionalStateExists_ = true;
    isStatusInitialized_ = false;
}

void CommandButton::setStatus(const bool status)
{
    if(isAdditionalStateExists_)
    {
        if(status == CommandButton::STATE_OFF)
        {
            setText(defaultText_);
            setIcon(defaultIcon_);
        }
        else
        {
            setText(additionalText_);
            setIcon(additionalIcon_);
        }

        currentStatus_ = status;

        isStatusInitialized_ = true;
        if(status == targetState_)
        {
            isWaiting_ = false;
        }
        update();
    }
}

void CommandButton::changeStatus()
{
    setStatus(!getStatus());
}

bool CommandButton::getStatus() const
{
    return currentStatus_;
}

bool CommandButton::isInitialized() const
{
    return isStatusInitialized_;
}

void CommandButton::setWaitingState(const bool status)
{
    isWaiting_ = status;

    if(isWaiting_)
    {
        targetState_ = !getStatus();
    }
}


bool CommandButton::isWaiting() const
{
    return isWaiting_;
}

