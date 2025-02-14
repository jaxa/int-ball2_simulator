#include "gui_color.h"

const QColor intball::Color::U1 = QColor(53, 51, 53);
const QColor intball::Color::U2 = QColor(29, 31, 34);
const QColor intball::Color::U3 = QColor(94, 85, 82);
const QColor intball::Color::U4 = QColor(134, 124, 119);
const QColor intball::Color::U5 = QColor(219, 54, 141);
const QColor intball::Color::U5_TRANSPARENT = QColor(219, 54, 141, 127);
const QColor intball::Color::U_HIGHLIGHT = QColor(48, 140, 198);

const QColor intball::Color::B1 = QColor(72, 80, 80);
const QColor intball::Color::B2 = QColor(229, 227, 226);
const QColor intball::Color::B3 = QColor(209, 142, 5);
const QColor intball::Color::B4 = QColor(0, 160, 233);
const QColor intball::Color::B_OUTSET_BORDER = QColor(100, 100, 100);
const QColor intball::Color::B_PRESSED_BACKGROUND = QColor(100, 100, 100);
const QColor intball::Color::B_SHADOW = QColor(22, 30, 30);

const QColor intball::Color::F1 = QColor(255, 255, 255);
const QColor intball::Color::F2 = QColor(201, 188, 156);
const QColor intball::Color::F3 = QColor(201, 160, 99);
const QColor intball::Color::F4 = QColor(106, 184, 45);

const QColor intball::Color::S1 = QColor(113, 186, 43);
const QColor intball::Color::S2 = QColor(239, 234, 58);
const QColor intball::Color::S3 = QColor(255, 51, 51);

const QColor intball::Color::TRANSPARENT = QColor(0, 0, 0, 0);

QString intball::Color::styleSheetRGB(const QColor& color) {
    return QString("rgb(%1,%2,%3)")
            .arg(color.red())
            .arg(color.green())
            .arg(color.blue());
}

QString intball::Color::styleSheetRGBA(const QColor& color) {
    return QString("rgba(%1,%2,%3,%4)")
            .arg(color.red())
            .arg(color.green())
            .arg(color.blue())
            .arg(color.alpha());
}


QString intball::Color::htmlRGB(const QColor& color) {
    return QString("#%1%2%3")
            .arg(color.red(), 2, 16, QChar('0'))
            .arg(color.green(), 2, 16, QChar('0'))
            .arg(color.blue(), 2, 16, QChar('0'));
}
