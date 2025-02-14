#ifndef COMMAND_BUTTON_H
#define COMMAND_BUTTON_H

#include <QPushButton>

namespace intball
{

/**
 * @brief QPushButtonにテレコマンド用の状態値を追加したクラス
 */
class CommandButton : public QPushButton
{
    Q_OBJECT
public:
    static const bool STATE_OFF = false;
    static const bool STATE_ON = true;

    CommandButton(QWidget* parent = nullptr);

    /**
     * @brief ON時のテキストとアイコンを設定する.
     * @param text 追加テキスト
     * @param icon 追加アイコン
     */
    void setAdditionalState(const QString text, const QIcon icon = QIcon());
    void setStatus(const bool status);
    void changeStatus();
    void setWaitingState(const bool status);
    bool getStatus() const;
    bool isInitialized() const;
    bool isWaiting() const;
protected:
    void paintEvent(QPaintEvent *event) override;
    bool currentStatus_;
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

#endif // COMMAND_BUTTON_H
