#include <float.h>
#include <QObject>
#include <QVector3D>
#include "operator_gui_config.h"
#include "qdebug_custom.h"
#include "route_point_list.h"
#include "ui_route_point_list.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

RoutePointList::RoutePointList(QWidget *parent) :
    RouteInformationView(parent),
    ui(new Ui::RoutePointList), lastSelectedPointListItem_(nullptr)
{
    ui->setupUi(this);
    clearGoal();
    clearViaPoint();

    // 経由点表示用のウィジットを初期化.
    int viaPointsMax = Config::valueAsInt(KEY_ROUTE_POINT_SIZE_MAX) - 2;
    for(int i = 0; i < viaPointsMax; ++i)
    {
        viaPointWidgets_.push_back(new RoutePointListItem(this));
        viaPointWidgets_.last()->setPoint(QVector3D(), QQuaternion());
        viaPointWidgets_.last()->hide();
        connect(viaPointWidgets_.last(), &RoutePointListItem::deleteSelected,
                this, &RoutePointList::RoutePointListItem_deleteSelected);
        connect(viaPointWidgets_.last(), &RoutePointListItem::selected,
                this, &RoutePointList::RoutePointListItem_selected);
        ui->verticalLayout->insertWidget(ui->verticalLayout->count() - 3, viaPointWidgets_.last());
    }

    ui->routePointListItemStart->setEditable(false);
    connect(ui->routePointListItemGoal, &RoutePointListItem::selected,
            this, &RoutePointList::RoutePointListItem_selected);
    connect(ui->routePointListItemGoal, &RoutePointListItem::deleteSelected,
            this, &RoutePointList::RoutePointListItem_deleteSelected);


    visibleListItem_ << ui->routePointListItemStart;
}

RoutePointList::~RoutePointList()
{
    viaPointWidgets_.clear();
    delete ui;
}

void RoutePointList::updateStartPoint()
{
    ui->routePointListItemStart->setPoint(thisModel()->currentIntBallPose().position(),
                                          thisModel()->currentIntBallPose().orientation());

}

bool RoutePointList::updateGoalPoint(const int firstToBeRemoved, const int lastToBeRemoved)
{
    int nextSize = thisModel()->rowCount();
    Q_ASSERT(nextSize > 0);

    if(lastToBeRemoved >= 0)
    {
        // 削除が実施される場合は,削除後の数をカウントする.
        nextSize -= (lastToBeRemoved + 1 - firstToBeRemoved);
    }

    // 2点目以上存在する場合は最終点をゴールとして描画する.
    if(nextSize > 1)
    {

        int targetIndex = thisModel()->goalIndex();
        if((lastToBeRemoved >= 0) && (thisModel()->lastOrLater(lastToBeRemoved)))
        {
            targetIndex = firstToBeRemoved-1;

        }
        ui->routePointListItemGoal->setPoint(
            thisModel()->data(targetIndex).position(),
            thisModel()->data(targetIndex).orientation());

        // ゴールが選択済みの場合はハイライト表示.
        if(selectionModel()->isSelected(thisModel()->index(targetIndex, 0)))
        {
            ui->routePointListItemGoal->changeHighlight(true);
        }
        else
        {
            ui->routePointListItemGoal->changeHighlight(false);
        }

        ui->routePointListTitleGoal->show();
        ui->routePointListItemGoal->show();

        return true;
    }
    else
    {
        /*
         * ゴールの非表示.
         * 選択済みゴールが削除された場合はselectionChangedシグナルが呼ばれるため
         * on_selectionChanged内で選択解除する.
         */
        clearGoal();
        return false;
    }
}


QVector<RoutePointListItem*> RoutePointList::updateViaPoints(const int firstToBeRemoved, const int lastToBeRemoved)
{

    int nextSize = thisModel()->rowCount();
    if(lastToBeRemoved >= 0)
    {
        // 削除が実施される場合は,削除後の数をカウントする.
        nextSize -= (lastToBeRemoved + 1 - firstToBeRemoved);
    }

    // 3点目以降が存在する場合は経由点として描画する.
    int visibleViaPointIndex = 0;
    QVector<RoutePointListItem*> setVector;
    if(nextSize > 2)
    {
        ui->routePointListTitleVia->show();

        // ゴール（最終点）を除く点までが対象.
        int nextLast = thisModel()->rowCount() - 1;
        if(thisModel()->lastOrLater(lastToBeRemoved))
        {
            nextLast = firstToBeRemoved - 1;
        }

        for(int i = 1; i < nextLast; ++i)
        {
            if((i >= firstToBeRemoved) && (i <= lastToBeRemoved))
            {
                continue;
            }
            setVector << viaPointWidgets_.at(visibleViaPointIndex);
            viaPointWidgets_.at(visibleViaPointIndex)->setPoint(
                thisModel()->data(i).position(), thisModel()->data(i).orientation());
            viaPointWidgets_.at(visibleViaPointIndex)->setVisible(true);

            // 選択済みの点はハイライト表示
            if(selectionModel()->isSelected(thisModel()->index(i, 0)))
            {
                viaPointWidgets_.at(visibleViaPointIndex)->changeHighlight(true);
            }
            else
            {
                viaPointWidgets_.at(visibleViaPointIndex)->changeHighlight(false);
            }
            visibleViaPointIndex++;
        }
    }
    else
    {
        ui->routePointListTitleVia->hide();
    }

    // 対象ではない経由点ウィジットのクリア.
    clearViaPoint(visibleViaPointIndex);

    return setVector;
}

void RoutePointList::clearGoal()
{
    ui->routePointListTitleGoal->hide();
    ui->routePointListItemGoal->hide();
    ui->routePointListItemGoal->setPoint(QVector3D(), QQuaternion());
    ui->routePointListItemGoal->changeHighlight(false);
}

void RoutePointList::clearViaPoint(const int viaPointIndex)
{
    if(viaPointIndex == 0)
    {
        ui->routePointListTitleVia->hide();
    }

    for(int i = viaPointIndex; i < viaPointWidgets_.size(); ++i)
    {
        // 表示OFF.
        viaPointWidgets_.at(i)->hide();
        viaPointWidgets_.at(i)->setPoint(QVector3D(), QQuaternion());
        viaPointWidgets_.at(i)->changeHighlight(false);
    }
}

void RoutePointList::RoutePointListItem_deleteSelected()
{
    INFO_START_FUNCTION();
    auto target = dynamic_cast<RoutePointListItem*>(sender());
    Q_ASSERT_X(target != nullptr, __FUNCTION__, "Invalid sender");
    int deleteIndex = -1;
    for(int i = 0; i < visibleListItem_.size(); i++)
    {
        if(visibleListItem_.at(i) == target)
        {
            deleteIndex = i;
            break;
        }
    }

    Q_ASSERT(deleteIndex >= 0);
    thisModel()->removeRows(deleteIndex, 1);

}

void RoutePointList::RoutePointListItem_selected()
{
    INFO_START_FUNCTION();

    auto target = dynamic_cast<RoutePointListItem*>(sender());
    Q_ASSERT_X(target != nullptr, __FUNCTION__, "Invalid sender");

    int index = visibleListItem_.indexOf(target);
    if(index >= 0)
    {
        selectionModel()->select(thisModel()->index(index, 0), QItemSelectionModel::ClearAndSelect);
    }
    else
    {
        Q_ASSERT_X(false, __FUNCTION__, "The target widget not found in visibleListItem_");
    }
}

void RoutePointList::updateInnerWidgetsSize(const int sizeToBeRemoved)
{
    int contentsHeight = (RoutePointListItem::FIXED_HEIGHT * (thisModel()->rowCount() - sizeToBeRemoved)) +
                         ui->routePointListTitleStart->height() +
                         ui->routePointListTitleVia->height() +
                         ui->routePointListTitleGoal->height();
    if(contentsHeight> height())
    {
        setFixedHeight(height() + RoutePointListItem::FIXED_HEIGHT);
    }
    else if(height() - contentsHeight > RoutePointListItem::FIXED_HEIGHT)
    {
        setFixedHeight(height() - RoutePointListItem::FIXED_HEIGHT);
    }
}

void RoutePointList::on_dataChanged(const QModelIndex &index, const RoutePoint& data)
{
    auto target = visibleListItem_.at(index.row());
    target->setPoint(data.position(), data.orientation());
}

void RoutePointList::on_rowsAboutToBeRemoved(int first, int last)
{
    INFO_START_FUNCTION() << "first=" << first << " last=" << last;
    Q_ASSERT(first > 0);
    Q_ASSERT(first <= last);

    // 表示中のリストアイテムへのポインタを保持するバッファをクリア.
    visibleListItem_.clear();
    visibleListItem_.squeeze();

    // Int-Ball現在位置.
    // モデルの先頭データは削除されないため値の変更は必要無し.
    visibleListItem_ << ui->routePointListItemStart;

    // 経由点.
    QVector<RoutePointListItem*> setViaPoints = updateViaPoints(first, last);
    if(setViaPoints.length() > 0)
    {
        visibleListItem_ << setViaPoints;
    }

    // ゴール.
    if(updateGoalPoint(first, last))
    {
        visibleListItem_ << ui->routePointListItemGoal;
    }

    updateInnerWidgetsSize(last + 1 - first);

}

void RoutePointList::on_rowsInserted(int first, int last)
{
    INFO_START_FUNCTION() << "first=" << first << " last=" << last;
    Q_UNUSED(first);
    Q_UNUSED(last);

    // 表示中のリストアイテムへのポインタを保持するバッファをクリア.
    visibleListItem_.clear();
    visibleListItem_.squeeze();

    // Int-Ball現在位置.
    visibleListItem_ << ui->routePointListItemStart;
    updateStartPoint();

    // 経由点.
    QVector<RoutePointListItem*> setViaPoints = updateViaPoints();
    if(setViaPoints.length() > 0)
    {
        visibleListItem_ << setViaPoints;
    }

    // ゴール.
    if(updateGoalPoint())
    {
        visibleListItem_ << ui->routePointListItemGoal;
    }

    updateInnerWidgetsSize();
}

void RoutePointList::on_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    INFO_START_FUNCTION() << "selected=" << selected << " deselected=" << deselected;
    Q_ASSERT(selected.length() <= 1);
    Q_ASSERT(deselected.length() <= 1);

    if(!selected.empty())
    {
        visibleListItem_.at(selected.at(0).top())->changeHighlight(true);
    }

    if(!deselected.empty())
    {
        visibleListItem_.at(deselected.at(0).top())->changeHighlight(false);
    }
}

