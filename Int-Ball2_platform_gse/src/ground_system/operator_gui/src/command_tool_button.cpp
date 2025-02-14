#include "command_tool_button.h"
#include <QPainter>
#include "gui_color.h"

using namespace intball;

CommandToolButton::CommandToolButton(QWidget* parent)
    :QToolButton(parent)
{

}

void CommandToolButton::paintEvent(QPaintEvent *event)
{
    QToolButton::paintEvent(event);

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

void CommandToolButton::setAdditionalState(const QString text, const QIcon icon)
{
    defaultText_ = this->text();
    defaultIcon_ = this->icon();
    additionalText_ = text;
    additionalIcon_ = icon;
    isAdditionalStateExists_ = true;
    isStatusInitialized_ = false;
}

void CommandToolButton::setStatus(const bool status)
{
    if(isAdditionalStateExists_)
    {
        if(status != ADDITIONAL_STATE)
        {
            // 通常表示.
            setText(defaultText_);
            setIcon(defaultIcon_.pixmap(25, 25));
        }
        else
        {
            // 追加ステータス表示.
            setText(additionalText_);
            setIcon(additionalIcon_.pixmap(25, 25));
        }

        // ステータスの初期化完了.
        isStatusInitialized_ = true;
        if(status == targetState_)
        {
            // 目標ステータスに遷移した場合,内部の待ち状態を解除する.
            isWaiting_ = false;
        }
        update();
    }
}

void CommandToolButton::changeStatus()
{
    setStatus(!getStatus());
}

bool CommandToolButton::getStatus() const
{
    return text() == additionalText_;
}

bool CommandToolButton::isInitialized() const
{
    return isStatusInitialized_;
}

void CommandToolButton::setWaitingState(const bool status)
{
    isWaiting_ = status;

    if(isWaiting_)
    {
        targetState_ = !getStatus();
    }
}


bool CommandToolButton::isWaiting() const
{
    return isWaiting_;
}
