#ifndef TELECOMMAND_WIDGET_H
#define TELECOMMAND_WIDGET_H

#include <QDateTime>
#include <QTableWidget>
#include <QWidget>
#include "telecommand_client.h"
#include "telecommand_log_area_widget.h"
#include "common_log_object.h"

namespace intball
{
namespace Ui
{
class TelecommandWidget;
}

class TelecommandWidget : public QWidget
{
    Q_OBJECT
public:

    explicit TelecommandWidget(QWidget *parent = nullptr);
    virtual ~TelecommandWidget();
    void enableControlsOnly();
    void enableParameterSettingsOnly();
    void setLogAreaWidget(TelecommandLogAreaWidget* widget);
    void showLogAreaWidget();

private:
    Ui::TelecommandWidget *ui;
    TelecommandLogAreaWidget *logAreaWidget_;
};

}

#endif // TELECOMMAND_WIDGET_H
