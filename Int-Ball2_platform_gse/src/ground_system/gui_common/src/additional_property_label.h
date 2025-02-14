#ifndef ADDITIONAL_PROPERTY_LABEL_H
#define ADDITIONAL_PROPERTY_LABEL_H

#include <QColor>
#include <QLabel>

namespace intball
{

class AdditionalPropertyLabel : public QLabel
{
    Q_OBJECT
    Q_PROPERTY(QColor fontColor READ fontColor WRITE setFontColor)
public:
    AdditionalPropertyLabel(QWidget* parent = nullptr);
    QColor fontColor();
    void setFontColor(const QColor& color);
private:
    QColor backgroundColor_;
    QColor fontColor_;
    QString baseStyleSheetString_;
    bool isBlock_;
};

} // namespace intball


#endif // ADDITIONAL_PROPERTY_LABEL_H
