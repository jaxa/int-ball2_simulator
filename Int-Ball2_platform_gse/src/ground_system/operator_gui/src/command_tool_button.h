#ifndef COMMAND_TOOL_BUTTON_H
#define COMMAND_TOOL_BUTTON_H

#include <QToolButton>

namespace intball
{

class CommandToolButton : public QToolButton
{
    Q_OBJECT
public:
    static const bool ADDITIONAL_STATE = true;

    CommandToolButton(QWidget* parent = nullptr);

    void setAdditionalState(const QString text, const QIcon icon = QIcon());
    void setStatus(const bool status);
    void changeStatus();
    void setWaitingState(const bool status);
    bool getStatus() const;
    bool isInitialized() const;
    bool isWaiting() const;
protected:
    void paintEvent(QPaintEvent *event) override;
    bool isAdditionalStateExists_;
    bool isStatusInitialized_;
    bool isWaiting_;
    bool targetState_;
    QString defaultText_;
    QIcon defaultIcon_;
    QString additionalText_;
    QIcon additionalIcon_;
};

} // namespace intball

#endif // COMMAND_TOOL_BUTTON_H
