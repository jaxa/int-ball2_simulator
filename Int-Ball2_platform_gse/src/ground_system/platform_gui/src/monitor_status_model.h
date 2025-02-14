#ifndef INTBALL_MONITORSTATUSMODEL_H
#define INTBALL_MONITORSTATUSMODEL_H

#include <QDateTime>
#include <QStandardItemModel>
#include "ib2_msgs.h"
#include "ros_related_type_definitions.h"

namespace intball {

class MonitorStatusModel : public QStandardItemModel
{
    Q_OBJECT

public:

    enum class KeyType {
        PUBLICATIONS,
        SUBSCRIPTIONS,
        SERVICES,
    };

    static const QString KEY_PUBLICATIONS;
    static const QString KEY_SUBSCRIPTIONS;
    static const QString KEY_SERVICES;

    explicit MonitorStatusModel(QObject *parent = nullptr);

    QStandardItem* rosNodeItem(const QString& nodeName) const;
    QStandardItem* keyItem(const QString& nodeName, const KeyType key) const;

    void updateNode(const KeyType key, const std::vector<NodeStatus>& values);
    void deleteOldData(const QDateTime& targetTimestamp, const int clearInterval);
    void deleteAll();

private:
   QStandardItem* rootItem_;

};

} // namespace intball

#endif // INTBALL_MONITORSTATUSMODEL_H
