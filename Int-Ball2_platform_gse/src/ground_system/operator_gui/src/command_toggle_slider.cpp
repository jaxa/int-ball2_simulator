#include <QFont>
#include <QMouseEvent>
#include <QPainter>
#include "gui_color.h"
#include "qdebug_custom.h"
#include "command_toggle_slider.h"

using namespace intball;

CommandToggleSlider::CommandToggleSlider(QWidget *parent) :
    QSlider(parent), isInitialized_(false), isWaiting_(false)
{
    setFixedSize(85, 30);
    setOrientation(Qt::Orientation::Horizontal);
    setMinimum(0);
    setMaximum(1);
    setStyleSheet(
        QString("QSlider::groove:horizontal {"
                "        height: 30px;"
                "        background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 %1, stop:0.3 %2);"
                "        border-radius: 15px;"
                "}"
                "QSlider::handle:horizontal {"
                "        width: 30px;"
                "        background: %3;"
                "        border-radius: 15px;"
                "}"
                "QSlider::sub-page:horizontal {"
                "        background: rgba(0, 0, 0, 0);"
                " }"
                "QSlider::add-page:horizontal {"
                "        background: rgba(0, 0, 0, 0);"
                "}")
        .arg(Color::styleSheetRGB(Color::B_SHADOW))
        .arg(Color::styleSheetRGB(Color::B1))
        .arg(Color::styleSheetRGB(Color::B2))

    );
    connect(this, &CommandToggleSlider::actionTriggered, this, &CommandToggleSlider::ToggleSlider_actionTriggered);
}

void CommandToggleSlider::paintEvent(QPaintEvent *event)
{
    const int fontSize = 14;
    QSlider::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHints(QPainter::HighQualityAntialiasing);
    painter.setPen(Color::F1);
    painter.setFont(QFont(painter.font().family(), fontSize, QFont::Normal, false));
    QRect rect(10, 0, width()-20, height());

    QLinearGradient ellipseGradient(0, 0, 28, 28);
    ellipseGradient.setColorAt(0.0, QColor(255, 255, 255));
    ellipseGradient.setColorAt(0.5, QColor(220, 220, 220));

    if(value())
    {
        painter.drawText(rect, Qt::AlignLeft | Qt::AlignVCenter, "ON");
        painter.setPen(QPen(QBrush(Color::B4, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
        painter.drawLine(rect.x(), rect.y() + rect.height() - 4,
                         rect.x() + rect.width(), rect.y() + rect.height() - 4);

        painter.setPen(QPen(QBrush(Color::B2, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
        painter.setBrush(Color::B2);
        painter.drawEllipse(width()-29, 1, 28, 28);
    }
    else
    {
        painter.drawText(rect, Qt::AlignRight | Qt::AlignVCenter, "OFF");
        painter.setPen(QPen(QBrush(Color::B4, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
        painter.drawLine(rect.x(), rect.y() + rect.height() - 4,
                         rect.x() + rect.width(), rect.y() + rect.height() - 4);

        painter.setPen(QPen(QBrush(Color::B2, Qt::BrushStyle::SolidPattern), 2, Qt::SolidLine));
        painter.setBrush(Color::B2);
        painter.drawEllipse(1, 1, 28, 28);
    }
}

void CommandToggleSlider::mousePressEvent(QMouseEvent *event)
{
    Q_UNUSED(event);
    emit clicked();
}

void CommandToggleSlider::ToggleSlider_actionTriggered(int action)
{
    Q_UNUSED(action);

    // スライダーとしての動きを全て無効にする.
    setSliderPosition(sliderPosition() == 0 ? 1 : 0);
}

void CommandToggleSlider::setStatus(STATUS value)
{
    if(!isInitialized_)
    {
        isInitialized_ = true;
    }

    if(static_cast<int>(value) != this->value())
    {
        setValue(static_cast<int>(value));
        emit toggled(true);
        isWaiting_ = false;
    }
}

void CommandToggleSlider::setWaiting()
{
    isWaiting_ = true;
}

bool CommandToggleSlider::isInitialized() const
{
    return isInitialized_;
}


bool CommandToggleSlider::isWaiting() const
{
    return isWaiting_;
}

CommandToggleSlider::STATUS CommandToggleSlider::getStatus() const
{
    return static_cast<CommandToggleSlider::STATUS>(this->value());
}
