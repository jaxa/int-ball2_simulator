#ifndef TELECOMMAND_LOG_AREA_WIDGET_H
#define TELECOMMAND_LOG_AREA_WIDGET_H

#include <QTableWidget>
#include "common_log_object.h"

namespace intball
{

class TelecommandLogAreaWidget : public QTableWidget
{
    Q_OBJECT

public:
    /**
     * @brief ログメッセージの表示幅最大値.
     *        表示幅が最大値を超える場合は改行される.
     */
    static const int MAX_MESSAGE_WIDTH = 550;

    /**
     * @brief 表示するログ行の最大値.
     */
    static const int MAX_LOG_LINE = 50;
    explicit TelecommandLogAreaWidget(QWidget *parent = nullptr);

public slots:
    void setLog(CommandLog log);
};

}

#endif // TELECOMMAND_LOG_AREA_WIDGET_H
