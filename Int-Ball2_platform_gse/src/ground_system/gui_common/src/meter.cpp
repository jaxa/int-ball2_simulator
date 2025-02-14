#include "meter.h"
#include <QPainter>
#include <QResizeEvent>
#include <QDebug>
#include "gui_color.h"
#include "gui_config_base.h"

using namespace intball;
using namespace qsettings;
using namespace qsettings::key;

Meter::Meter(QWidget* parent)
    : QProgressBar(parent), chunkHeight_(4), marginRight_(40), tickValueInterval_(10), tickHeight_(8),
      isWarning_(false), currentColor_(Color::S1)
{
    thresholdAttention_ = Config::valueAsInt(KEY_CRITICAL_TEMPERATURE);
    thresholdWarning_ = Config::valueAsInt(KEY_WARNING_TEMPERATURE);
    thresholdComeUnder_ = Config::valueAsInt(KEY_NORMAL_TEMPERATURE);
}

const QColor& Meter::determineColor()
{
    if(value() >= thresholdAttention_)
    {
        isWarning_ = true;
        return Color::S3;
    }
    else if(value() >= thresholdWarning_)
    {
        isWarning_ = true;
        return Color::S2;
    }
    else if(isWarning_)
    {
        // 一旦警告状態に遷移した場合は,
        // 警告値とは別の正常状態向けしきい値で判定する.
        if(value() <= thresholdComeUnder_)
        {
            isWarning_ = false;
            return Color::S1;
        }
        else
        {
            return Color::S2;
        }
    }
    else
    {
        return Color::S1;
    }
}

void Meter::resizeEvent(QResizeEvent *event)
{
    int marginTopBottom = (event->size().height() - chunkHeight_) / 2;
    setStyles(determineColor(), marginTopBottom);

    QProgressBar::resizeEvent(event);
}

void Meter::setStyles(const QColor& color, const int marginTopBottom)
{
    QString styleSheet = QString(
                             "QProgressBar {"
                             "  background-color: rgba(0, 0, 0, 0);"
                             "  border: 0;"
                             "  margin: %1px %2px %3px 0px;"
                             "  text-align: right;"
                             "}"
                             "QProgressBar::chunk {"
                             "  background-color: %4;"
                             "}"
                             "* {"
                             "color: %4;"
                             "}")
                         .arg(marginTopBottom).arg(marginRight_).arg(marginTopBottom)
                         .arg(Color::styleSheetRGB(color));
    setStyleSheet(styleSheet);
    currentColor_ = color;
}

void Meter::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    painter.setPen(QPen(QBrush(Color::B2), 1, Qt::SolidLine));

    if(determineColor() != currentColor_)
    {
        setStyles(determineColor(), (height() - chunkHeight_) / 2);
    }

    // メーターの背景線.
    int meterBackgroundX = width() - marginRight_;
    painter.drawLine(0, height()/2, meterBackgroundX, height()/2);

    // Tick.
    int tickYStart = height() / 2 - (tickHeight_ / 2);
    int tickYEnd = height() / 2 + (tickHeight_ / 2);

    double tickCount = static_cast<double>(maximum()) / tickValueInterval_;
    double tickDrawInterval = meterBackgroundX / tickCount;
    for(int i = 0; i < tickCount; ++i)
    {
        painter.drawLine(QPointF(i * tickDrawInterval, tickYStart), QPointF(i * tickDrawInterval, tickYEnd));
    }
    painter.drawLine(meterBackgroundX, tickYStart, meterBackgroundX, tickYEnd);

    // QProgressBarの描画処理.
    QProgressBar::paintEvent(event);
}
