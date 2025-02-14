#ifndef GUI_COLORS_H
#define GUI_COLORS_H

#include <QColor>
#include <QString>

namespace intball {

class Color {

public:
    static const QColor U1;
    static const QColor U2;
    static const QColor U3;
    static const QColor U4;
    static const QColor U5;
    static const QColor U5_TRANSPARENT;
    static const QColor U_HIGHLIGHT;

    static const QColor B1;
    static const QColor B2;
    static const QColor B3;
    static const QColor B4;
    static const QColor B_OUTSET_BORDER;
    static const QColor B_PRESSED_BACKGROUND;
    static const QColor B_SHADOW;

    static const QColor F1;
    static const QColor F2;
    static const QColor F3;
    static const QColor F4;

    static const QColor S1;
    static const QColor S2;
    static const QColor S3;

    static const QColor TRANSPARENT;

    static QString styleSheetRGB(const QColor& color);
    static QString styleSheetRGBA(const QColor& color);
    static QString htmlRGB(const QColor& color);
};

} // namespace intball

#endif // GUI_COLORS_H
