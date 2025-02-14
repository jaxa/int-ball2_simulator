#ifndef COMMON_MAIN_DIALOG_H
#define COMMON_MAIN_DIALOG_H

#include <QDialog>

namespace intball {

namespace Ui {
class MainDialog;
}

class MainDialog : public QDialog
{
    Q_OBJECT

public:
    enum class Type {
        INFO,
        ALERT,
        CONFIRM,
    };
    explicit MainDialog(QWidget *parent = nullptr);
    ~MainDialog();
    void setText(const QString& message);
    void setType(const Type& type);
private:
    Ui::MainDialog *ui;
    Type type_;
};

} // namespace intball

#endif // COMMON_MAIN_DIALOG_H
