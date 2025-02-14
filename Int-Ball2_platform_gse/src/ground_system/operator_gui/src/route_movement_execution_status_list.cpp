#include "route_movement_execution_status_list.h"
#include "ui_route_movement_execution_status_list.h"
#include "gui_color.h"
#include "operator_gui_config.h"
#include "route_movement_execution_status_list_item.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

RouteMovementExecutionStatusList::RouteMovementExecutionStatusList(QWidget *parent) :
    QWidget(parent), ui(new Ui::RouteMovementExecutionStatusList),
    listMax_(0), currentListLength_(0), nextPointIndex_(-1), isWaiting_(false)
{
    ui->setupUi(this);

    setStyleSheet(QString("*{"
                          "border: 0;"
                          "background-color: %1;"
                          "color: %2;"
                          "}").arg(Color::styleSheetRGB(Color::U2)).arg(Color::styleSheetRGB(Color::F1)));

    listMax_ = Config::valueAsInt(KEY_ROUTE_POINT_SIZE_MAX) * 2;
    for(int i = 0; i < listMax_; ++i)
    {
        auto testItem = new QListWidgetItem();
        auto testItemWidget = new RouteMovementExecutionStatusListItem(this);
        testItem->setSizeHint(testItemWidget->sizeHint());
        ui->statusList->addItem(testItem);
        ui->statusList->setItemWidget(testItem, testItemWidget);
    }
}

RouteMovementExecutionStatusList::~RouteMovementExecutionStatusList()
{
    delete ui;
}

void RouteMovementExecutionStatusList::initialize(const RouteInformation* routeInformation)
{
    // 経由点のリストを初期化する.
    int listTargetIndex = 0;
    for(int i = 0; i < routeInformation->count(); ++i)
    {
        auto target = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(listTargetIndex)));
        Q_ASSERT(target != nullptr);
        ui->statusList->item(listTargetIndex)->setHidden(false);
        target->assign(routeInformation, i);
        listTargetIndex++;

        if(i != routeInformation->goalIndex())
        {
            auto target = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(listTargetIndex)));
            Q_ASSERT(target != nullptr);
            ui->statusList->item(listTargetIndex)->setHidden(false);
            target->setIconOnly(false);
            listTargetIndex++;
        }
    }
    currentListLength_ = listTargetIndex;

    for(; listTargetIndex < listMax_; ++listTargetIndex)
    {
        auto target = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(listTargetIndex)));
        Q_ASSERT(target != nullptr);
        target->clear();
        ui->statusList->item(listTargetIndex)->setHidden(true);
    }

    // 最新座標の表記.
    dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(0)))->setLatest(true);

    // 目標点のインデックスを初期化.
    nextPointIndex_ = -1;
}

void RouteMovementExecutionStatusList::setNext(const int routePointIndex)
{
    INFO_START_FUNCTION() << " routePointIndex=" << routePointIndex;
    Q_ASSERT(routePointIndex * 2 - 1 <= currentListLength_);

    if(nextPointIndex_ >= 0)
    {
        // setNext呼び出し済みだった場合は,直前の移動中表示を消去する.
        // statusListは　[初期座標] -> [アイコン1] -> [経由点1] -> [アイコン2] .... の順で並んでいる/
        // 経由点1の移動を完了した場合は,アイコン1(=経由点1のインデックス-1)の移動中表示を消去する.
        auto beforeIcon = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(nextPointIndex_ * 2 - 1)));
        beforeIcon->setIconOnly(false);

        // 待機中の表示を消去する.
        setWaiting(false);
    }
    auto next = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(routePointIndex * 2 - 1)));
    next->setIconOnly(true);

    // 次目標インデックスの更新.
    nextPointIndex_ = routePointIndex;

    // 現在地点の表記を更新.
    updateLatestPoint();
}

void RouteMovementExecutionStatusList::setWaiting(const bool on)
{
    if(nextPointIndex_ >= 0)
    {
        auto target = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(nextPointIndex_ * 2)));
        if(on)
        {
            // 移動中表示を消去する.
            auto beforeIcon = dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(nextPointIndex_ * 2 - 1)));
            beforeIcon->setIconOnly(false);

            // 待機中の表示とする.
            target->setWaitingState(true);
        }
        else
        {
            // 待機中の表示を消去する.
            target->setWaitingState(false);
        }
        isWaiting_ = on;

        // 現在地点の表記を更新.
        updateLatestPoint();
    }
}

void RouteMovementExecutionStatusList::updateLatestPoint()
{
    INFO_START_FUNCTION() << "isWaiting_=" << isWaiting_ << " nextPointIndex_=" << nextPointIndex_;
    if(isWaiting_)
    {
        // 目標地点に到達し, 待機状態.

        // 次目標の1つ前を最新座標表示を解除する.
        if(nextPointIndex_ > 0)
        {
            dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item((nextPointIndex_-1) * 2)))->setLatest(false);
        }

        // 現在目標となっている点を最新座標として表示する.
        // 現在目標の点にて待機状態となっている想定.
        dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item(nextPointIndex_ * 2)))->setLatest(true);
    }
    else
    {
        // 次の目標地点に移動中.

        // 次目標の2つ前の最新座標表示を解除する.
        if(nextPointIndex_ > 1)
        {
            dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item((nextPointIndex_-2) * 2)))->setLatest(false);
        }

        // 次目標の1つ前を最新座標として表示する.
        if(nextPointIndex_ > 0)
        {
            dynamic_cast<RouteMovementExecutionStatusListItem*>(ui->statusList->itemWidget(ui->statusList->item((nextPointIndex_-1) * 2)))->setLatest(true);
        }
    }
}
