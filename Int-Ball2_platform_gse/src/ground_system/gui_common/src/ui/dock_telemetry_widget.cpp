#include <ros/ros.h>
#include "dock_telemetry_widget.h"
#include "ui_dock_telemetry_widget.h"
#include "model/dock_telemetry.h"

using namespace intball;
using namespace intball::dock::telemetry;

DockTelemetryWidget::DockTelemetryWidget(QWidget *parent) :
    QAbstractItemView(parent),
    ui(new Ui::DockTelemetryWidget)
{
    ui->setupUi(this);
}

DockTelemetryWidget::~DockTelemetryWidget()
{
    delete ui;
}

QRect DockTelemetryWidget::visualRect(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return QRect(0, 0, width(), height());
}

void DockTelemetryWidget::scrollTo(const QModelIndex &index, ScrollHint hint)
{
    Q_UNUSED(index);
    Q_UNUSED(hint);
}

QModelIndex DockTelemetryWidget::indexAt(const QPoint &point) const
{
    Q_UNUSED(point);
    return QModelIndex();
}

DockTelemetry* DockTelemetryWidget::thisModel()
{
    Q_ASSERT(dynamic_cast<DockTelemetry*>(model()) != nullptr);
    return dynamic_cast<DockTelemetry*>(model());
}

void DockTelemetryWidget::dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(topLeft);
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    ui->labelDockTelemetryCounterValue->setNum(thisModel()->data<unsigned char>(Index::TELEMETRY_COUNTER));
    ui->labelDockCommandCounterValue->setNum(thisModel()->data<unsigned char>(Index::COMMAND_COUNTER));
    ui->labelDockChargeStateValue->setText(thisModel()->getChargeStateAsString());
    ui->labelDockMotorStateValue->setText(thisModel()->getMotorStateAsString());
    ui->labelDockSwitchStateValue->setText(thisModel()->getSwitchStateAsString());
    ui->labelDockCommandStateValue->setText(thisModel()->getCommandStateAsString());
    ui->labelDockVersionValue->setNum(thisModel()->data<unsigned char>(Index::VERSION));
    ui->labelDockMotorTempValue->setNum(static_cast<double>(thisModel()->data<float>(Index::MOTOR_TEMP)));
    ui->labelDockDCDCTempValue->setNum(static_cast<double>(thisModel()->data<float>(Index::DCDC_TEMP)));
    ui->labelDockPower1CurrentValue->setNum(static_cast<double>(thisModel()->data<float>(Index::POWER1_CURRENT)));
    ui->labelDockPower2CurrentValue->setNum(static_cast<double>(thisModel()->data<float>(Index::POWER2_CURRENT)));
    ui->labelDockTempAlertValue->setText(thisModel()->getTempAlertAsString());
    ui->labelDockChargeTimeValue->setNum(thisModel()->data<unsigned short>(Index::CHARGE_TIME));
}

QModelIndex DockTelemetryWidget::moveCursor(CursorAction cursorAction, Qt::KeyboardModifiers modifiers)
{
    Q_UNUSED(cursorAction);
    Q_UNUSED(modifiers);
    return QModelIndex();
}

int DockTelemetryWidget::horizontalOffset() const
{
    return 0;
}

int DockTelemetryWidget::verticalOffset() const
{
    return 0;
}

bool DockTelemetryWidget::isIndexHidden(const QModelIndex &index) const
{
    Q_UNUSED(index);
    return false;
}

void DockTelemetryWidget::setSelection(const QRect &rect, QItemSelectionModel::SelectionFlags command)
{
    Q_UNUSED(rect);
    Q_UNUSED(command);
}

QRegion DockTelemetryWidget::visualRegionForSelection(const QItemSelection &selection) const
{
    Q_UNUSED(selection);
    return QRegion();
}

void DockTelemetryWidget::rowsAboutToBeRemoved(const QModelIndex &parent, int start, int end)
{
    Q_UNUSED(parent);
    Q_UNUSED(start);
    Q_UNUSED(end);
}

void DockTelemetryWidget::rowsInserted(const QModelIndex &parent, int start, int end)
{
    Q_UNUSED(parent);
    Q_UNUSED(start);
    Q_UNUSED(end);
}

void DockTelemetryWidget::selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    Q_UNUSED(selected);
    Q_UNUSED(deselected);
}

void DockTelemetryWidget::currentChanged(const QModelIndex &current, const QModelIndex &previous)
{
    Q_UNUSED(current);
    Q_UNUSED(previous);
}
