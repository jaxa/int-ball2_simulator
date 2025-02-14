#ifndef IMAGE_PUSH_BUTTON_H
#define IMAGE_PUSH_BUTTON_H

#include <QPushButton>

namespace intball
{

class ImagePushButton : public QPushButton
{
    Q_OBJECT
public:
    ImagePushButton(QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    bool eventFilter(QObject* obj, QEvent* event) override;

private:
    void setImages(const QSize& baseSize);

    QPixmap normal_;
    QPixmap pressed_;
    QPixmap disabled_;
    QRect drawArea_;
    bool isPressed_;

};

} // namespace intball

#endif // IMAGE_PUSH_BUTTON_H
