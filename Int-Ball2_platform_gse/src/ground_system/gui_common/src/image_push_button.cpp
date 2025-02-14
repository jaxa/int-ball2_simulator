#include "image_push_button.h"
#include <QIcon>
#include <QPainter>
#include <QPaintEvent>
#include <QPixmap>
#include "qdebug_custom.h"

using namespace intball;

ImagePushButton::ImagePushButton(QWidget* parent)
    : QPushButton (parent), isPressed_(false)
{
    setImages(iconSize());
    installEventFilter(this);
}

void ImagePushButton::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    if(!isEnabled())
    {
        painter.drawPixmap(drawArea_, disabled_);
    }
    else if(isPressed_)
    {
        painter.drawPixmap(drawArea_, pressed_);
    }
    else
    {
        painter.drawPixmap(drawArea_, normal_);
    }
}

void ImagePushButton::resizeEvent(QResizeEvent *event)
{
    QPushButton::resizeEvent(event);
    setImages(iconSize());
}

void ImagePushButton::setImages(const QSize& baseSize)
{
    QSize targetSize = baseSize.scaled(size(), Qt::KeepAspectRatio);
    drawArea_ = QRect(0, 0, targetSize.width(), targetSize.height());
    drawArea_.moveCenter(QPoint(width() / 2, height() / 2));
    normal_ = icon().pixmap(targetSize, QIcon::Mode::Normal, QIcon::State::Off);
    pressed_ = icon().pixmap(targetSize, QIcon::Mode::Normal, QIcon::State::On);
    disabled_ = icon().pixmap(targetSize, QIcon::Mode::Disabled, QIcon::State::Off);
}

bool ImagePushButton::eventFilter(QObject* obj, QEvent* event)
{
    if(event->type() == QEvent::MouseButtonPress || event->type() == QEvent::MouseButtonDblClick)
    {
        isPressed_ = true;
    }
    else if(event->type() == QEvent::MouseButtonRelease)
    {
        isPressed_ = false;
    }
    return QPushButton::eventFilter(obj, event);
}
