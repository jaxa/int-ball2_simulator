#ifndef AMOUNT_SLIDER_WIDGET_H
#define AMOUNT_SLIDER_WIDGET_H

#include <QDoubleSpinBox>
#include <QLabel>
#include <QLayout>
#include <QSlider>
#include <QWidget>

namespace intball
{

class AmountSliderWidget : public QWidget
{
    Q_OBJECT
public:
    explicit AmountSliderWidget(QWidget *parent = nullptr);
    float value();
    void setLabelLayout(const Qt::Orientation& labelOrientation);
    void setUnitLabel(const QString& label);
    void setSliderValueScale(const qreal scale);
    QSlider& slider();

protected:
    void paintEvent(QPaintEvent *event) override;

private slots:
    void sliderValueChanged();
    void sliderRangeChanged(int min, int max);
    void valueBoxChanged(double value);

private:
    QSlider* slider_;
    QLabel* amountLabel_;
    QLabel* unitLabel_;
    QDoubleSpinBox* valueBox_;
    QLabel* minLabel_;
    QLabel* maxLabel_;
    QGridLayout* mainLayout_;
    QWidget* valueLabelWidget_;
    QWidget* minMaxLabelWidget_;
    Qt::Orientation labelOrientation_;
    qreal sliderValueScale_;
};

}

#endif // AMOUNT_SLIDER_WIDGET_H
