#include "telecommand_log_area_widget.h"
#include <QTableWidgetItem>
#include "utils.h"
#include "qdebug_custom.h"

using namespace intball;

TelecommandLogAreaWidget::TelecommandLogAreaWidget(QWidget *parent) :
    QTableWidget(parent)
{
    // setItemでQTableWidgetItemを設定する前に、列数を設定しておく必要がある
    setColumnCount(3);
}

void TelecommandLogAreaWidget::setLog(CommandLog log)
{
    insertRow(0);
    setItem(0, 0, new QTableWidgetItem(dateTimeStringWithoutYear(log.timestamp)));
    QString levelString = "";
    switch(log.level)
    {
    case CommandLogLevel::INFO:
        levelString = "INFO";
        break;
    default:
        levelString = "ERROR";
        break;
    }
    setItem(0, 1, new QTableWidgetItem(levelString));
    setItem(0, 2, new QTableWidgetItem(automaticLineBreak(log.message, MAX_MESSAGE_WIDTH, fontMetrics())));

    if(rowCount() > MAX_LOG_LINE)
    {
        removeRow(rowCount() - 1);
    }
}
