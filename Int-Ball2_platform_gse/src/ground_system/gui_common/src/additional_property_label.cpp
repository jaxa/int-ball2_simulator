#include "additional_property_label.h"
#include "gui_color.h"
#include "qdebug_custom.h"

using namespace intball;

AdditionalPropertyLabel::AdditionalPropertyLabel(QWidget* parent)
    : QLabel(parent), baseStyleSheetString_(""), isBlock_(true)
{
}

QColor AdditionalPropertyLabel::fontColor()
{
    return fontColor_;
}

void AdditionalPropertyLabel::setFontColor(const QColor& color)
{
    fontColor_ = color;
    if(baseStyleSheetString_.isEmpty())
    {
        baseStyleSheetString_ = styleSheet();
        isBlock_ = baseStyleSheetString_.contains("{");
    }

    // パレットの変更
    QPalette palette = this->palette();
    palette.setColor(QPalette::WindowText, fontColor_);
    this->setAutoFillBackground(true);
    this->setPalette(palette);

    // スタイルシートの変更（文字色設定を上書き）.
    // 既にcolorを設定済みでも,末尾に追加したcolor設定が優先される.
    if(isBlock_)
    {
        setStyleSheet(baseStyleSheetString_ + QString("*{color: %1;}").arg(Color::styleSheetRGB(color)));
    }
    else
    {
        setStyleSheet(baseStyleSheetString_ + QString("color: %1;").arg(Color::styleSheetRGB(color)));
    }
}
