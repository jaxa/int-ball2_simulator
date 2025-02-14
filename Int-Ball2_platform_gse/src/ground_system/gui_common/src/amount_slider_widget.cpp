#include "amount_slider_widget.h"
#include <QPaintEvent>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPainter>
#include <QSlider>
#include <QtMath>
#include <QLayout>
#include "gui_color.h"

using namespace intball;

AmountSliderWidget::AmountSliderWidget(QWidget *parent) : QWidget(parent), labelOrientation_(Qt::Orientation::Vertical), sliderValueScale_(1.0)
{
    // スライダー.
    slider_ = new QSlider(this);

    /*
     * groove, sub-page, add-pageの上下marginを揃える.
     * handleの下marginで, handle用画像と軸との距離を調整する.
     * handleの(-上margin - 下margin)が表示サイズとなる.
     */
    slider_->setStyleSheet((QString("QSlider::groove:horizontal {"
                               "        height: 2px;"
                               "        background: %1;"
                               "        margin: 10px 10px -10px 0px;"
                               "}"
                               "QSlider::handle:horizontal {"
                               "        width: 19px;"
                               "        background-image: url(:/ground_system/image/triangle-down_fill_white.svg);"
                               "        background-repeat: no-repeat;"
                               "        margin: -20px -10px 5px 0px;"
                               "}"
                               "QSlider::sub-page:horizontal {"
                               "        background: qlineargradient(x1:0, y1:0, x2:1, y1:0, stop:0 %2, stop:1 %3);"
                               "        margin: 10px 0px -10px 0px"
                               "}"
                               "QSlider::add-page:horizontal {"
                               "        background: %1;"
                               "        margin: 10px 10px -10px 0px;"
                               "}")
                                .arg(Color::styleSheetRGB(Color::B2))
                                .arg(Color::styleSheetRGB(Color::U5))
                                .arg(Color::styleSheetRGBA(Color::U5_TRANSPARENT)))
                            );
    slider_->setOrientation(Qt::Orientation::Horizontal);
    slider_->setTracking(true);
    slider_->setTickPosition(QSlider::TickPosition::NoTicks);
    slider_->setFixedHeight(40);

    valueBox_ = new QDoubleSpinBox(this);
    valueBox_->setStyleSheet(QString("background-color: %1;color: %2;")
                             .arg(Color::styleSheetRGB(Color::U1))
                             .arg(Color::styleSheetRGB(Color::F1)));
    valueBox_->setFixedWidth(70);
    valueBox_->setButtonSymbols(QDoubleSpinBox::ButtonSymbols::NoButtons);
    valueBox_->setDecimals(0);
    valueBox_->setAlignment(Qt::AlignmentFlag::AlignRight);

    amountLabel_ = new QLabel(this);
    amountLabel_->setText("amount: ");

    unitLabel_ = new QLabel(this);

    valueLabelWidget_ = new QWidget(this);
    QHBoxLayout* valueLabelLayout = new QHBoxLayout(valueLabelWidget_);
    valueLabelLayout->setMargin(0);
    valueLabelLayout->addWidget(amountLabel_);
    valueLabelLayout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Preferred));
    valueLabelLayout->addWidget(valueBox_);
    valueLabelLayout->addWidget(unitLabel_);
    valueLabelLayout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Fixed, QSizePolicy::Preferred));

    minLabel_ = new QLabel(this);
    minLabel_->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    minLabel_->setNum(slider_->minimum() / sliderValueScale_);
    maxLabel_ = new QLabel(this);
    maxLabel_->setAlignment(Qt::AlignRight | Qt::AlignTop);
    maxLabel_->setNum(slider_->maximum() / sliderValueScale_);
    maxLabel_->setContentsMargins(0, 0, 10, 0);

    minMaxLabelWidget_ = new QWidget(this);
    QHBoxLayout* minMaxLabelLayout = new QHBoxLayout(minMaxLabelWidget_);
    minMaxLabelLayout->setMargin(0);
    minMaxLabelLayout->addWidget(minLabel_);
    minMaxLabelLayout->addSpacerItem(new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Preferred));
    minMaxLabelLayout->addWidget(maxLabel_);

    mainLayout_ = new QGridLayout(this);
    mainLayout_->setMargin(0);
    mainLayout_->setSpacing(0);
    setLabelLayout(labelOrientation_);

    // スライダー操作のconnect.
    connect(slider_, &QSlider::valueChanged, this, &AmountSliderWidget::sliderValueChanged);
    connect(slider_, &QSlider::rangeChanged, this, &AmountSliderWidget::sliderRangeChanged);

    // 値の直接入力のconnect.
    connect(valueBox_,  QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &AmountSliderWidget::valueBoxChanged);
}

void AmountSliderWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);

    // QSliderのTickを先に描画する.
    painter.setPen(QPen(QBrush(QColor(200, 200, 200)), 5, Qt::SolidLine));
    painter.drawLine(0, 0, 100, 100);

    QWidget::paintEvent(event);

}

void AmountSliderWidget::setLabelLayout(const Qt::Orientation& labelOrientation)
{
    labelOrientation_ = labelOrientation;

    mainLayout_->removeWidget(valueLabelWidget_);
    mainLayout_->removeWidget(slider_);
    mainLayout_->removeWidget(minMaxLabelWidget_);
    switch(labelOrientation_)
    {
    case Qt::Orientation::Horizontal:
        mainLayout_->addWidget(valueLabelWidget_, 0, 0, 2, 1);
        mainLayout_->addWidget(slider_, 0, 1);
        mainLayout_->addWidget(minMaxLabelWidget_, 1, 1);
        break;
    case Qt::Orientation::Vertical:
        mainLayout_->addWidget(valueLabelWidget_, 0, 0);
        mainLayout_->addWidget(slider_, 1, 0);
        mainLayout_->addWidget(minMaxLabelWidget_, 2, 0);
        break;
    default:
        Q_ASSERT(false);
    }
}

void AmountSliderWidget::setUnitLabel(const QString& label)
{
    unitLabel_->setText(label);
}

void AmountSliderWidget::setSliderValueScale(const qreal scale)
{
    sliderValueScale_ = scale;

    // 値の再設定.
    valueBox_->setValue(slider_->value() / sliderValueScale_);
    valueBox_->setRange(slider_->minimum() / sliderValueScale_, slider_->maximum() / sliderValueScale_);
    valueBox_->setSingleStep(1.0 / scale);
    valueBox_->setDecimals(qCeil(std::log10(scale)));
    minLabel_->setNum(slider_->minimum() / sliderValueScale_);
    maxLabel_->setNum(slider_->maximum() / sliderValueScale_);
}

QSlider& AmountSliderWidget::slider()
{
    return *slider_;
}

float AmountSliderWidget::value()
{
    float amountValue = static_cast<float>(valueBox_->value());
    return amountValue;
}

void AmountSliderWidget::sliderValueChanged()
{
    valueBox_->setValue(slider_->value() / sliderValueScale_);
}

void AmountSliderWidget::sliderRangeChanged(int min, int max)
{
    minLabel_->setNum(min / sliderValueScale_);
    maxLabel_->setNum(max / sliderValueScale_);
    valueBox_->setMinimum(min / sliderValueScale_);
    valueBox_->setMaximum(max / sliderValueScale_);
}

void AmountSliderWidget::valueBoxChanged(double value)
{
    slider_->setValue(static_cast<int>(value * sliderValueScale_));
}
