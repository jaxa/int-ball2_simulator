#ifndef METER_H
#define METER_H

#include <QProgressBar>
#include <QWidget>


namespace intball
{

class Meter : public QProgressBar
{
public:
    Meter(QWidget* parent = nullptr);
protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    int chunkHeight_;
    int marginRight_;
    int tickValueInterval_;
    int tickHeight_;
    int thresholdAttention_;
    int thresholdWarning_;
    int thresholdComeUnder_;
    bool isWarning_;
    QColor currentColor_;

    const QColor& determineColor();
    void setStyles(const QColor& determineColor, const int marginTopBottom);
};

} // namespace intball

#endif // METER_H
