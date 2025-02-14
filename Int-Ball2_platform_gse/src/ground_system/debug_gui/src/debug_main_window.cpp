#include "debug_main_window.h"
#include "ui_debug_main_window.h"
#include <QGridLayout>
#include <QHeaderView>
#include <QListWidgetItem>
#include <QScrollArea>
#include <QSplitter>
#include "ui/intball_telemetry_widget.h"
#include "common_log_object.h"
#include "ros_common.h"
#include "telemetry_subscriber.h"

using namespace intball;

namespace  {

/*
 * 各テレメトリ項目（ウィジットのグループ）が、テレメトリ表示領域0,1,2（左、真ん中、右）の
 * どの領域にデフォルトで表示されるかを定義する.
 * NOTE: わデフォルト表示の後の切り替え処理を動作させるには、IntBallTelemetryWidget::setVisibilityの修正が必要
 */
static const QMap<TelemetryWidgetGroup, int> DEFAULT_DISPLAY_GROUP_INDEX = {
    {TelemetryWidgetGroup::HEADER,                            0},
    {TelemetryWidgetGroup::FLIGHT_SOFTWARE,                   0},
    {TelemetryWidgetGroup::ALIVE_STATUS,                      0},
    {TelemetryWidgetGroup::SYSTEM_MONITOR,                    0},
    {TelemetryWidgetGroup::FILE_MONITOR,                      0},
    {TelemetryWidgetGroup::TASK_MANAGER,                      1},
    {TelemetryWidgetGroup::NAVIGATION,                        1},
    {TelemetryWidgetGroup::IMU,                               1},
    {TelemetryWidgetGroup::GUIDANCE_AND_CONTROL,              1},
    {TelemetryWidgetGroup::WRENCH,                            1},
    {TelemetryWidgetGroup::PROPULSION,                        1},
    {TelemetryWidgetGroup::CAMERA_AND_MICROPHONE,             1},
    {TelemetryWidgetGroup::LED_DISPLAY,                       1},
    {TelemetryWidgetGroup::RESPONSE_CAMERA_AND_MICROPHONE,    2},
    {TelemetryWidgetGroup::RESPONSE_PROPULSION,               2},
    {TelemetryWidgetGroup::RESPONSE_GUIDANCE_AND_CONTROL,     2},
    {TelemetryWidgetGroup::RESPONSE_TASK_MANAGER,             2},
    {TelemetryWidgetGroup::RESPONSE_NAVIGATION,               2},
    {TelemetryWidgetGroup::RESPONSE_IMU,                      2},
    {TelemetryWidgetGroup::RESPONSE_LED_DISPLAY,              2},
    {TelemetryWidgetGroup::RESPONSE_DISPLAY_MANAGER,          2},
    {TelemetryWidgetGroup::RESPONSE_ROSPARAMS,                2},
};

void itemSelectionChanged(QListWidget* listWidget, IntBallTelemetryWidget* widget)
{
    for(auto i: TELEMETRY_WIDGET_GROUP_LABEL.keys())
    {
        auto listItems = listWidget->findItems(TELEMETRY_WIDGET_GROUP_LABEL.value(i), Qt::MatchFlag::MatchFixedString);
        if(listItems.count() > 0)
        {
            for(auto item: listItems)
            {
                widget->setVisibility(i, item->isSelected());
            }
        }
    }
}

}

DebugMainWindow::DebugMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::DebugMainWindow)
{
    ui->setupUi(this);

    qRegisterMetaType<CommandLog>();

    // テレメトリタブ.
    intballTelemetry_ = new IntBallTelemetry(this);
    ui->telemetryScrollAreaWidgetContents->setModel(intballTelemetry_);
    ui->telemetryScrollAreaWidgetContents_2->setModel(intballTelemetry_);
    ui->telemetryScrollAreaWidgetContents_3->setModel(intballTelemetry_);

    dockTelemetry_ = new DockTelemetry(this);
    ui->dockTelemetryWidget->setModel(dockTelemetry_);

    // テレコマンドタブ.
    ui->telecommandControlsWidget->enableControlsOnly();
    ui->telecommandParameterWidget->enableParameterSettingsOnly();

    // ログ表示用ウィジット設定
    logTableWidget_ = new TelecommandLogAreaWidget();
    logTableWidget_->setSizeAdjustPolicy(QAbstractScrollArea::SizeAdjustPolicy::AdjustToContents);
    logTableWidget_->setEditTriggers(QAbstractItemView::EditTrigger::NoEditTriggers);
    logTableWidget_->setTabKeyNavigation(false);
    logTableWidget_->setDropIndicatorShown(false);
    logTableWidget_->setDropIndicatorShown(false);
    logTableWidget_->setSelectionMode(QAbstractItemView::SelectionMode::NoSelection);
    logTableWidget_->setTextElideMode(Qt::TextElideMode::ElideNone);
    logTableWidget_->setVerticalScrollMode(QAbstractItemView::ScrollMode::ScrollPerPixel);
    logTableWidget_->setWordWrap(false);
    logTableWidget_->setCornerButtonEnabled(false);
    logTableWidget_->horizontalHeader()->setVisible(false);
    logTableWidget_->horizontalHeader()->setHighlightSections(false);
    logTableWidget_->horizontalHeader()->setMinimumSectionSize(80);
    logTableWidget_->horizontalHeader()->setStretchLastSection(true);
    logTableWidget_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    logTableWidget_->verticalHeader()->setVisible(false);
    logTableWidget_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    ui->telecommandControlsWidget->setLogAreaWidget(logTableWidget_);
    ui->telecommandParameterWidget->setLogAreaWidget(logTableWidget_);

    // テレメトリの受信を開始.
    subscriber_ = new TelemetrySubscriber(this);
    subscriber_->start(*getNodeHandle(), intballTelemetry_, dockTelemetry_);

    // 表示制御用のウィジット初期化.
    for(auto i: TELEMETRY_WIDGET_GROUP_LABEL.keys())
    {
        // index = 0.
        ui->displayTargetWidget->addItem(TELEMETRY_WIDGET_GROUP_LABEL.value(i));

        // index = 1.
        ui->displayTargetWidget_2->addItem(TELEMETRY_WIDGET_GROUP_LABEL.value(i));

        // index = 2.
        ui->displayTargetWidget_3->addItem(TELEMETRY_WIDGET_GROUP_LABEL.value(i));
    }
    resetView();

    // 初期表示タブを指定
    ui->tabWidget->setCurrentIndex(0);
}

DebugMainWindow::~DebugMainWindow()
{
    delete ui;
}

void DebugMainWindow::closeEvent(QCloseEvent *event)
{
    Q_UNUSED(event);
    ros::shutdown();
}

void DebugMainWindow::resetView()
{
    Q_ASSERT(ui->displayTargetWidget->count() == ui->displayTargetWidget_2->count());
    Q_ASSERT(ui->displayTargetWidget_2->count() == ui->displayTargetWidget_3->count());

    // 表示設定用ウィジットの選択状態を更新.
    for(auto i: TELEMETRY_WIDGET_GROUP_LABEL.keys())
    {
        // index = 0.
        ui->displayTargetWidget->item(static_cast<int>(i))->setSelected(DEFAULT_DISPLAY_GROUP_INDEX.value(i) == 0);

        // index = 1.
        ui->displayTargetWidget_2->item(static_cast<int>(i))->setSelected(DEFAULT_DISPLAY_GROUP_INDEX.value(i) == 1);

        // index = 2.
        ui->displayTargetWidget_3->item(static_cast<int>(i))->setSelected(DEFAULT_DISPLAY_GROUP_INDEX.value(i) == 2);
    }

    // スロットを呼び出して表示を更新する.
    ui->displayTargetWidget->itemSelectionChanged();
    ui->displayTargetWidget_2->itemSelectionChanged();
    ui->displayTargetWidget_3->itemSelectionChanged();
}

void DebugMainWindow::on_displayTargetWidget_itemSelectionChanged()
{
    itemSelectionChanged(ui->displayTargetWidget, ui->telemetryScrollAreaWidgetContents);
}

void DebugMainWindow::on_displayTargetWidget_2_itemSelectionChanged()
{
    itemSelectionChanged(ui->displayTargetWidget_2, ui->telemetryScrollAreaWidgetContents_2);
}

void DebugMainWindow::on_displayTargetWidget_3_itemSelectionChanged()
{
    itemSelectionChanged(ui->displayTargetWidget_3, ui->telemetryScrollAreaWidgetContents_3);
}

void DebugMainWindow::on_resetViewButton_clicked()
{
    resetView();
}

void DebugMainWindow::on_tabWidget_currentChanged(int index)
{
    switch(index)
    {
    case 0: // テレメトリ
        break;
    case 1: // 制御用テレコマンドを表示
        // ログ用ウィジットの設定
        ui->telecommandControlsWidget->showLogAreaWidget();
        break;
    case 2: // パラメータ設定用テレコマンドを表示
        // ログ用ウィジットの設定
        ui->telecommandParameterWidget->showLogAreaWidget();
        break;
    }
}
