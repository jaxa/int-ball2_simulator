#include <QWidget>
#include "monitor_status_model.h"
#include "platform_gui_config.h"
#include "utils.h"

#include "qdebug_custom.h"

using namespace intball;
using namespace qsettings;
using namespace qsettings::key;
using namespace platform_msgs;

const QString MonitorStatusModel::KEY_PUBLICATIONS = "publications";
const QString MonitorStatusModel::KEY_SUBSCRIPTIONS = "subscriptions";
const QString MonitorStatusModel::KEY_SERVICES = "services";

MonitorStatusModel::MonitorStatusModel(QObject *parent)
    : QStandardItemModel(parent)
{
    rootItem_ = QStandardItemModel::invisibleRootItem();

    // 2列目はタイムスタンプ
    QStandardItemModel::setColumnCount(2);
}

QStandardItem* MonitorStatusModel::rosNodeItem(const QString& nodeName) const
{
    QStandardItem* returnValue = nullptr;

    auto nodeList = QStandardItemModel::findItems(nodeName,
                                                  Qt::MatchFlag::MatchFixedString | Qt::MatchFlag::MatchCaseSensitive);
    if(nodeList.size() > 0)
    {
        // ノード名の値は重複はない想定
        returnValue = nodeList.at(0);
    }

    return returnValue;
}

QStandardItem* MonitorStatusModel::keyItem(const QString& nodeName, const KeyType key) const
{
    auto node = rosNodeItem(nodeName);
    if(node != nullptr)
    {
        QString keyText;
        switch(key)
        {
        case KeyType::PUBLICATIONS:
            keyText = KEY_PUBLICATIONS;
            break;
        case KeyType::SUBSCRIPTIONS:
            keyText = KEY_SUBSCRIPTIONS;
            break;
        case KeyType::SERVICES:
            keyText = KEY_SERVICES;
            break;
        }

        // nodeの子要素（ツリー構造の直下）にkey文字列が存在するため,
        // nodeの子要素を探索する.。
        // (invisibleRootItem) -> rosnode -> key(publications, subscriptions, services) -> value
        for(int i = 0; i < node->rowCount(); ++i)
        {
            if(node->child(i)->text() == keyText)
            {
                return node->child(i);
            }
        }
    }

    return nullptr;
}

void MonitorStatusModel::updateNode(const KeyType key,
                                    const std::vector<NodeStatus>& values)
{
    // NodeStatusValueのデータを, 後続処理用にQMapに入れ込む
    QMap<QString, QList<NodeStatus>> setupValues; // 取得した監視結果の値（トピック名など）
    for(NodeStatus value: values)
    {
        auto targetNode = QString::fromStdString(value.node);
        if (targetNode != "")
        {
            if(!setupValues.contains(targetNode))
            {
                setupValues[targetNode] = QList<NodeStatus>();
            }

            // 既存リスト内に同じ値が無いかチェックする
            auto newValue = QString::fromStdString(value.value);
            bool isNotContains = true;
            for(NodeStatus nodeStatus : setupValues[targetNode])
            {
                if(nodeStatus.value == value.value)
                {
                    isNotContains = false;
                    break;
                }
            }
            if(isNotContains)
            {
                auto targetList = setupValues.value(targetNode);
                targetList.push_back(value);
                setupValues[targetNode] = targetList;
            }
        }
    }

    QMap<QString, QList<NodeStatus>>::const_iterator i = setupValues.constBegin();
    while (i != setupValues.constEnd()) {
        // ツリー内のノードの存在チェック
        auto nodeName = i.key();
        if(rosNodeItem(nodeName) == nullptr)
        {
            // 新規ノードのツリーを追加する
            auto newItem = new QStandardItem(nodeName);
            rootItem_->appendRow(newItem);
            newItem->appendRow(new QStandardItem(KEY_PUBLICATIONS));
            newItem->appendRow(new QStandardItem(KEY_SUBSCRIPTIONS));
            newItem->appendRow(new QStandardItem(KEY_SERVICES));
        }

        // 対象のキー列（publications, subscriptions, services）の抽出
        auto targetItem = keyItem(nodeName, key);
        if(targetItem != nullptr)
        {
            targetItem->removeColumn(0);
            targetItem->removeColumn(1);
            targetItem->setRowCount(0);
            for(NodeStatus newChildValue : i.value())
            {
                // FIXME: このnewがメモリリークを引き起こさないか、APIの仕様確認が必要
                QList<QStandardItem*> test;
                test.push_back(new QStandardItem(QString::fromStdString(newChildValue.value)));
                test.push_back(new QStandardItem(dateTimeString(newChildValue.stamp)));
                targetItem->appendRow(test);
            }
        }

        i++;
    }


//    if(deleteOld)
//    {
//        deleteOldData();
//    }
}

void MonitorStatusModel::deleteOldData(const QDateTime& targetTimestamp, const int clearInterval)
{
//    // 削除処理で比較対象とするタイムスタンプ値
//    QDateTime targetTimestamp;

//    if(lastTimestamp.isValid())
//    {
//        targetTimestamp = lastTimestamp;
//    }
//    else
//    {
//        // 表示中の内容から最新のタイムスタンプ値を抽出する
//        for(int rosNodeCount = 0; rosNodeCount < QStandardItemModel::rowCount(); ++rosNodeCount)
//        {
//            LOG_INFO() << "TEST A " << rosNodeCount;
//            auto rosNodeItem = QStandardItemModel::item(rosNodeCount, 0); // ノード名の表示箇所
//            for(int keyCount = 0; keyCount < rosNodeItem->rowCount(); ++keyCount)
//            {
//                LOG_INFO() << "TEST B " << keyCount;
//                auto keyItem = rosNodeItem->child(keyCount, 0); // publications, subscriptions, servicesの箇所
//                for(int valueCount = 0; valueCount < keyItem->rowCount(); ++valueCount)
//                {
//                    LOG_INFO() << "TEST C " << valueCount;
//                    auto dateTimeStringItem = keyItem->child(valueCount, 1);
//                    if(dateTimeStringItem != nullptr)
//                    {
//                        auto checkTimestamp = parseDateTimeString(dateTimeStringItem->text());
//                        LOG_INFO() << "TEST D " << dateTimeStringItem->text();
//                        LOG_INFO() << "TEST D " << checkTimestamp;
//                        LOG_INFO() << "TEST D " << targetTimestamp;
//                        if(!targetTimestamp.isValid() || ( checkTimestamp.isValid() && checkTimestamp > targetTimestamp))
//                        {
//                            LOG_INFO() << "TEST E";
//                            targetTimestamp = checkTimestamp;
//                        }
//                    }
//                }
//            }
//        }
//    }

//    if(!targetTimestamp.isValid())
//    {
//        // タイムスタンプ設定済みの行が無ければ処理無し.
//        return;
//    }

//    LOG_INFO() << "TEST X " << targetTimestamp;

    QList<QString> deleteRosNodeName;
    for(int rosNodeIndex = 0; rosNodeIndex < QStandardItemModel::rowCount(); ++rosNodeIndex)
    {
        auto rosNodeItem = QStandardItemModel::item(rosNodeIndex, 0); // ノード名の表示箇所

        int lastValueCount = 0;
        for(int keyIndex = 0; keyIndex < rosNodeItem->rowCount(); ++keyIndex)
        {
            auto keyItem = rosNodeItem->child(keyIndex, 0); // publications, subscriptions, servicesの箇所

            int checkValueIndex = 0;
            int checkValueCount = keyItem->rowCount();
            while(checkValueIndex < checkValueCount)
            {
                auto dateTimeStringItem = keyItem->child(checkValueIndex, 1);

                // 先にインデックス値をインクリメントし、削除によりインデックス値がずれる場合は追加処理を行う
                checkValueIndex++;

                if(dateTimeStringItem != nullptr)
                {
                    auto checkTimestamp = parseDateTimeString(dateTimeStringItem->text());
                    // 古いタイムスタンプのアイテムを消去する
                    if(checkTimestamp.isValid() && (targetTimestamp.toSecsSinceEpoch() - checkTimestamp.toSecsSinceEpoch() > clearInterval))
                    {
                        checkValueIndex--; // 削除により既存データ（rosNodeItem->child）のインデックスがずれる

                        keyItem->removeRow(checkValueIndex);
                    }
                }
            }

            // publications/subscriptions/services配下のアイテム数を保持しておく
            lastValueCount += keyItem->rowCount();
        }

        if(lastValueCount == 0)
        {
            // 削除対象ノード名の保持
            // 全ての値（key=publications/subscriptions/services配下のアイテム）が削除されたら、ノードごと表示を消去する
            deleteRosNodeName.push_back(rosNodeItem->text());
        }
    }

    // ノード表示全体を消去
    for(QString nodeName : deleteRosNodeName)
    {
        auto deleteTarget = rosNodeItem(nodeName);
        if(deleteTarget != nullptr)
        {
            rootItem_->removeRow(deleteTarget->row());
        }
    }
}

void MonitorStatusModel::deleteAll()
{
    rootItem_->removeRows(0, rootItem_->rowCount());
}

//QVariant MonitorStatusModel::headerData(int section, Qt::Orientation orientation, int role) const
//{
//    // FIXME: Implement me!
//    return QVariant("TEST HEADER");
//}

//QModelIndex MonitorStatusModel::index(int row, int column, const QModelIndex &parent) const
//{
//    // FIXME: Implement me!
//    if(!hasIndex(row, column, parent))
//    {
//        return QModelIndex();
//    }

//    return createIndex(row, column);
//}

//QModelIndex MonitorStatusModel::parent(const QModelIndex &index) const
//{
//    // FIXME: Implement me!
//    return QModelIndex();
//}

//int MonitorStatusModel::rowCount(const QModelIndex &parent) const
//{
//    if (!parent.isValid()) // 親がない＝トップレベルの要素
//        return 0;

//    // FIXME: Implement me!
//    return static_cast<QWidget*>(parent.internalPointer())->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly).size();
//}

//int MonitorStatusModel::columnCount(const QModelIndex &parent) const
//{
//    if (!parent.isValid()) // 親がない＝トップレベルの要素
//        return 0;

//    // FIXME: Implement me!
//    return 1;
//}

////bool MonitorStatusModel::hasChildren(const QModelIndex &parent) const
////{
////    // FIXME: Implement me!
////}

////bool MonitorStatusModel::canFetchMore(const QModelIndex &parent) const
////{
////    // FIXME: Implement me!
////    return false;
////}

////void MonitorStatusModel::fetchMore(const QModelIndex &parent)
////{
////    // FIXME: Implement me!
////}

//QVariant MonitorStatusModel::data(const QModelIndex &index, int role) const
//{
//    if (!index.isValid())
//        return QVariant();

//    // FIXME: Implement me!
//    return QVariant("TEST");
//}

