#ifndef TOGGLE_SLIDER_H
#define TOGGLE_SLIDER_H

#include <QMessageBox>
#include <QSlider>
#include <QWidget>

namespace intball
{

class CommandToggleSlider : public QSlider
{
    Q_OBJECT
public:
    enum class STATUS {
        OFF = 0,
        ON = 1
    };

    explicit CommandToggleSlider(QWidget *parent = nullptr);
    void setWaiting();
    bool isWaiting() const;
    bool isInitialized() const;
    STATUS getStatus() const;
public slots:
    void setStatus(STATUS value);
signals:
    void toggled(bool checked);
    void clicked();
protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
private slots:
    void ToggleSlider_actionTriggered(int action);
private:
    bool isInitialized_;
    bool isWaiting_;
};

}

#endif // TOGGLE_SLIDER_H
