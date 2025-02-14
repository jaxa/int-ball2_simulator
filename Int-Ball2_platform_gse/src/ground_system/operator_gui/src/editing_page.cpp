#include "editing_page.h"
#include "ui_editing_page.h"
#include <QDoubleValidator>
#include <QQuaternion>
#include <QString>
#include <QtMath>
#include <QVector3D>
#include <sensor_msgs/Range.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include "amount_slider_widget.h"
#include "camera_view_panel.h"
#include "gui_common.h"
#include "operator_gui_config.h"
#include "dialog_factory.h"
#include "qdebug_custom.h"
#include "ros_common.h"
#include "route_point_list.h"
#include "telecommand_client.h"
#include "utils.h"


using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;
using namespace intball::message;

EditingPage::EditingPage(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::EditingPage)
{
    ui->setupUi(this);
    ui->editTopDownViewWidget->setAxisType(RouteEditor::AxisType::X_Y);

    routePointListWidget_ = new RoutePointList(this);
    routePointListWidget_->setFixedWidth(ui->editRoutePointListArea->width());
    ui->editRoutePointListArea->setWidget(routePointListWidget_);

    // 入力フォームは数値のみ.
    QDoubleValidator* validator = new QDoubleValidator(this);
    ui->editInputX->setValidator(validator);
    ui->editInputY->setValidator(validator);
    ui->editInputZ->setValidator(validator);
    ui->editInputYaw->setValidator(validator);
    ui->editInputPitch->setValidator(validator);
    ui->editInputRoll->setValidator(validator);

    // スライダー.
    ui->editAmountPositionSlider->setUnitLabel("[m]");
    ui->editAmountPositionSlider->setSliderValueScale(100.0);
    ui->editAmountPositionSlider->slider().setSingleStep(10);
    ui->editAmountPositionSlider->slider().setPageStep(10);
    ui->editAmountPositionSlider->slider().setMinimum(1);
    ui->editAmountPositionSlider->slider().setMaximum(50);
    ui->editAmountPositionSlider->setLabelLayout(Qt::Orientation::Horizontal);

    ui->editAmountAttitudeSlider->setUnitLabel("[deg]");
    ui->editAmountAttitudeSlider->slider().setSingleStep(10);
    ui->editAmountAttitudeSlider->slider().setPageStep(10);
    ui->editAmountAttitudeSlider->slider().setMinimum(5);
    ui->editAmountAttitudeSlider->slider().setMaximum(180);
    ui->editAmountAttitudeSlider->setLabelLayout(Qt::Orientation::Horizontal);

    //GoalCameraView -> EditPage
    connect(ui->editCameraSimulationPanel, &CameraViewPanel::changed, this, &EditingPage::GoalCameraSimulationPanel_changed);

    // rvizの座標系をQtの座標系に変換する設定.
    QTransform transform;
    // NOTE: 現状設定パラメータ化は無し
//    transform.translate(ui->editTopDownViewWidget->width() / 2, ui->editTopDownViewWidget->height() / 2);
//    transform.scale(10, -10);
    ui->editTopDownViewWidget->initialize(transform);
    ui->editSideViewWidget->initialize(transform);

    // 初期状態で待機秒数は入力不可.
    ui->spinBoxWaitingTime->setValue(0);
    ui->spinBoxWaitingTime->setDisabled(true);
}

EditingPage::~EditingPage()
{
//    publisherRoute_.shutdown();
    delete ui;
}

void EditingPage::initialize(const QString& pathRvizConfig, RouteInformation* routeInformation, QItemSelectionModel* routeInformationSelectionModel,
                             TelecommandClient* telecommandClient)
{
    routeInformation_ = routeInformation;
    routeInformationSelectionModel_ = routeInformationSelectionModel;
    telecommandClient_ = telecommandClient;

    ui->editBirdEyeViewPanel->initialize(pathRvizConfig);

    /*
     * QtのViewクラスでは,setModelが呼ばれた際に内部でSelectionModelが自動生成されるため
     * 必ずsetModelを呼び出した後にsetSelectionModelを呼び出す必要がある.
     */
    ui->editTopDownViewWidget->setModel(routeInformation_);
    ui->editSideViewWidget->setModel(routeInformation_);
    routePointListWidget_->setModel(routeInformation_);
    ui->editTopDownViewWidget->setSelectionModel(routeInformationSelectionModel_);
    ui->editSideViewWidget->setSelectionModel(routeInformationSelectionModel_);
    routePointListWidget_->setSelectionModel(routeInformationSelectionModel_);

    connect(routeInformation_, &RouteInformation::rowsInserted, this, &EditingPage::RouteInformation_rowsInserted);
    connect(routeInformation_, &RouteInformation::rowsAboutToBeRemoved, this, &EditingPage::RouteInformation_rowsAboutToBeRemoved);
    connect(routeInformation_, &RouteInformation::dataChanged, this, &EditingPage::RouteInformation_dataChanged);
    connect(routeInformationSelectionModel_, &QItemSelectionModel::selectionChanged,
            this, &EditingPage::RouteInformation_selectionChanged);

    ui->editCameraSimulationPanel->initialize(Config::packagePath() + Config::valueAsString(KEY_RVIZ_CONFIG_CAMERA));
    ui->editCameraSimulationPanel->hide();
}

void EditingPage::removeCameraTf()
{
    // カメラのtfをリセットする.
    tf::Vector3 v(0, 0, 0);
    tf::Quaternion q(0, 0, 0, 1);
    tf::Transform transform(q, v);
    ros::Time now = ros::Time::now();
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform, now, rosframe::ISS_BODY, rosframe::CAMERA));

    // カメラ表示パネルをリセットする.
    ui->editCameraSimulationPanel->clear();
}

void EditingPage::publishCameraTf(const QVector3D& position, const QQuaternion& orientation)
{
    tf::Vector3 v = qtToTf(position);
    tf::Transform transform_goal(qtToTf(orientation), v);
    ros::Time now = ros::Time::now();
    tfBroadcaster_.sendTransform(tf::StampedTransform(transform_goal, now,rosframe::ISS_BODY, rosframe::CAMERA));
}

void EditingPage::GoalCameraSimulationPanel_changed()
{
    INFO_START_FUNCTION();
    if(!routeInformationSelectionModel_->selectedRows().empty())
    {
        auto selectedIndex = routeInformationSelectionModel_->selectedRows().first();
        qDebug() << __FUNCTION__ << " Index of the target point=" << selectedIndex;

        // rvizのパネル用に/tfを送信する.
        tf::StampedTransform transform;
        try
        {
            getTransformListener()->lookupTransform(rosframe::ISS_BODY, rosframe::CAMERA, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            LOG_WARNING() << "Can't lookup transform: " <<  ex.what();
            return;
        }

        ros::Time now = ros::Time::now();
        transform.setRotation(ui->editCameraSimulationPanel->getQuaternion());

        tfBroadcaster_.sendTransform(tf::StampedTransform(transform, now, rosframe::ISS_BODY, rosframe::CAMERA));

        // モデルのデータを更新する.
        routeInformation_->setDataOrientation(selectedIndex, tfToQt(ui->editCameraSimulationPanel->getQuaternion()));
    }
}

void EditingPage::setPointValues(const int index)
{
    const QVector3D position = routeInformation_->data(index).position();
    const QQuaternion orientation = routeInformation_->data(index).orientation();
    const unsigned int waitingTime = routeInformation_->data(index).waitSecond();

    // 入力フォームに,選択した点の値を反映.
    ui->editInputX->setText(QString::number(roundPositionValue(position.x())));
    ui->editInputY->setText(QString::number(roundPositionValue(position.y())));
    ui->editInputZ->setText(QString::number(roundPositionValue(position.z())));

    // オイラー角の順序はros(tf)基準.
    qreal roll, pitch, yaw;
    getRPY(orientation, roll, pitch, yaw);
    // 角度（Degree）表記.
    ui->editInputRoll->setText(QString::number(roundDegree(roll)));
    ui->editInputPitch->setText(QString::number(roundDegree(pitch)));
    ui->editInputYaw->setText(QString::number(roundDegree(yaw)));

    // 待機秒数.
    ui->spinBoxWaitingTime->setValue(static_cast<int>(waitingTime));

    // 向き指定用カメラの向きを変更する.
    ui->editCameraSimulationPanel->setView(position, orientation);
}

void EditingPage::editPositionButtonOperation(const QVector3D& diff)
{
    INFO_START_FUNCTION() << "diff=" << diff;

    if(!routeInformationSelectionModel_->selectedIndexes().isEmpty())
    {
        int selectedIndex = routeInformationSelectionModel_->selectedIndexes().first().row();
        QVector3D newPosition = transformRelative(
                                    routeInformation_->data(selectedIndex).position(),
                                    routeInformation_->data(selectedIndex).orientation(),
                                    diff);

        // JEM船内に収まる値に丸める.
        newPosition = ui->editSideViewWidget->roundToValidRoutePoint(newPosition);
        newPosition = ui->editTopDownViewWidget->roundToValidRoutePoint(newPosition);
        routeInformation_->setDataPosition(selectedIndex, newPosition);
    }
}


void EditingPage::editAttitudeButtonOperationRPY(const float roll, const float pitch, const float yaw)
{
    INFO_START_FUNCTION() << "roll=" << roll << " pitch=" << pitch << " yaw=" << yaw;

    if(!routeInformationSelectionModel_->selectedIndexes().isEmpty())
    {
        int selectedIndex = routeInformationSelectionModel_->selectedIndexes().first().row();
        QQuaternion newOrientation = transformRelativeRPY(
                                         routeInformation_->data(selectedIndex).position(),
                                         routeInformation_->data(selectedIndex).orientation(),
                                         roll, pitch, yaw);
        routeInformation_->setDataOrientation(selectedIndex, newOrientation);
    }
}

void EditingPage::on_editPositionUp_clicked()
{
    editPositionButtonOperation(QVector3D(0.0, 0.0, ui->editAmountPositionSlider->value()));
}

void EditingPage::on_editPositionDown_clicked()
{
    editPositionButtonOperation(QVector3D(0.0, 0.0, -ui->editAmountPositionSlider->value()));
}

void EditingPage::on_editPositionLeft_clicked()
{
    editPositionButtonOperation(QVector3D(0.0, ui->editAmountPositionSlider->value(), 0.0));
}

void EditingPage::on_editPositionRight_clicked()
{
    editPositionButtonOperation(QVector3D(0.0, -ui->editAmountPositionSlider->value(), 0.0));
}

void EditingPage::on_editPositionFront_clicked()
{
    editPositionButtonOperation(QVector3D(ui->editAmountPositionSlider->value(), 0, 0));
}

void EditingPage::on_editPositionBack_clicked()
{
    editPositionButtonOperation(QVector3D(-ui->editAmountPositionSlider->value(), 0.0, 0.0));
}

void EditingPage::on_editAttitudeRollCounterClock_clicked()
{
    editAttitudeButtonOperationRPY(qDegreesToRadians(ui->editAmountAttitudeSlider->value()), 0.0, 0.0);
}

void EditingPage::on_editAttitudeRollClock_clicked()
{
    editAttitudeButtonOperationRPY(qDegreesToRadians(360 - ui->editAmountAttitudeSlider->value()), 0.0, 0.0);
}

void EditingPage::on_editAttitudePitchCounterClock_clicked()
{
    editAttitudeButtonOperationRPY(0.0, qDegreesToRadians(ui->editAmountAttitudeSlider->value()), 0.0);
}

void EditingPage::on_editAttitudePitchClock_clicked()
{
    editAttitudeButtonOperationRPY(0.0, qDegreesToRadians(360 - ui->editAmountAttitudeSlider->value()), 0.0);
}

void EditingPage::on_editAttitudeYawCounterClock_clicked()
{
    editAttitudeButtonOperationRPY(0.0, 0.0, qDegreesToRadians(ui->editAmountAttitudeSlider->value()));
}

void EditingPage::on_editAttitudeYawClock_clicked()
{
    editAttitudeButtonOperationRPY(0.0, 0.0, qDegreesToRadians(360 - ui->editAmountAttitudeSlider->value()));
}

void EditingPage::RouteInformation_dataChanged(const QModelIndex &topLeft, const QModelIndex &bottomRight, const QVector<int> &roles)
{
    Q_UNUSED(bottomRight);
    Q_UNUSED(roles);

    int dataListIndex = topLeft.row();
    if(routeInformation_->lastOrLater(dataListIndex))
    {
        // カメラtfの変更.
        publishCameraTf(routeInformation_->goal().position(), routeInformation_->goal().orientation());
    }

    // 選択済みの点が更新された場合, 値を更新する.
    if(routeInformationSelectionModel_->isSelected(topLeft))
    {
        setPointValues(dataListIndex);
        publishCameraTf(routeInformation_->data(dataListIndex).position(),
                        routeInformation_->data(dataListIndex).orientation());
    }
}

void EditingPage::RouteInformation_rowsAboutToBeRemoved(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);

    Q_ASSERT(first > 0);

    // 選択済みの点が削除される場合はカメラtfを非表示とする.
    bool isSelectedPoint = false;
    for(int i = first; i <= last; ++i)
    {
        isSelectedPoint = RouteInformation::isSelected(routeInformation_, routeInformationSelectionModel_, i);
        if(isSelectedPoint)
        {
            removeCameraTf();
            break;
        }
    }

    // ゴールが削除され, 新しいゴール地点が設定される場合は新しいゴールの待機秒数をゼロとする.
    if(first > 1 && routeInformation_->rowCount() - 1 <= last) {
        auto newGoalIndex = first - 1;
        auto newGoal = routeInformation_->data(newGoalIndex);
        newGoal.setData(0);
        routeInformation_->setData(newGoalIndex, QVariant::fromValue(newGoal));

        // 未選択,またはゴール以外が選択されている場合は待機秒数を設定可能とする.
        if(!routeInformationSelectionModel_->currentIndex().isValid() ||
                RouteInformation::isSelected(routeInformation_, routeInformationSelectionModel_, newGoalIndex))
        {
            // 待機秒数を編集不可.
            ui->spinBoxWaitingTime->setDisabled(true);
        }
        else
        {
            // ゴール以外の点を選択中.
            ui->spinBoxWaitingTime->setEnabled(true);
        }
    }
}

void EditingPage::RouteInformation_rowsInserted(const QModelIndex &parent, int first, int last)
{
    Q_UNUSED(parent);
    Q_UNUSED(first);
    Q_UNUSED(last);

    // 未選択,またはゴール以外が選択されている場合は待機秒数を設定可能とする.
    if(!routeInformationSelectionModel_->currentIndex().isValid() ||
            RouteInformation::isSelected(routeInformation_, routeInformationSelectionModel_, routeInformation_->goalIndex()))
    {
        ui->spinBoxWaitingTime->setEnabled(true);
    }
    else
    {
        ui->spinBoxWaitingTime->setDisabled(true);
    }
}

void EditingPage::RouteInformation_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    INFO_START_FUNCTION() << "selected=" << selected << " deselected" << deselected;
    Q_ASSERT(selected.length() <= 1);
    Q_ASSERT(deselected.length() <= 1);

    if(!selected.empty())
    {
        int selectedIndex = selected.first().top();

        // 点が選択された場合,カメラのシミュレーションを表示する.
        if(!ui->editCameraSimulationPanel->isVisible())
        {
            ui->editCameraSimulationPanel->show();
        }

        // rviz上のtfを選択点に移動.
        publishCameraTf(routeInformation_->data(selectedIndex).position(),
                        routeInformation_->data(selectedIndex).orientation());

        // ゴール以外が選択された場合は待機秒数を設定可能とする.
        if(selectedIndex != routeInformation_->goalIndex())
        {
            ui->spinBoxWaitingTime->setEnabled(true);
        }
        else
        {
            ui->spinBoxWaitingTime->setDisabled(true);
        }


        // 選択した点の値を画面に反映.
        setPointValues(selectedIndex);
    }
    else if(!deselected.empty())
    {
        // 選択の解除のみ行われた場合.

        // カメラtfを削除
        removeCameraTf();

        // 入力フォームの表示をクリア.
        ui->editInputX->clear();
        ui->editInputY->clear();
        ui->editInputZ->clear();
        ui->editInputPitch->clear();
        ui->editInputYaw->clear();
        ui->editInputRoll->clear();
        ui->spinBoxWaitingTime->setValue(0);
        ui->spinBoxWaitingTime->setDisabled(false);

        // カメラのシミュレーションを非表示にする.
        ui->editCameraSimulationPanel->hide();

        // 待機秒数は入力不可.
        ui->spinBoxWaitingTime->setValue(0);
        ui->spinBoxWaitingTime->setDisabled(true);
    }
}

void EditingPage::setVideoArea(QWidget* video)
{
    video->setParent(ui->videoArea);
    video->setFixedSize(ui->videoArea->size());
    video->show();
}

void EditingPage::setStatusArea(QWidget* status)
{
    ui->statusGroup->layout()->addWidget(status);
}

void EditingPage::setCamera()
{
    ui->editBirdEyeViewPanel->setFocalPointDockingStation();
    ui->editBirdEyeViewPanel->setFocalPoint(routeInformation_->currentIntBallPose().position());
}

void intball::EditingPage::on_editCancelButton_clicked()
{
    // 選択状態を解除する.
    routeInformationSelectionModel_->clear();

    // メインの画面クラスにキャンセルを通知.
    emit readyToSwitch(SwitchPageEvent::CANCEL);
}

void intball::EditingPage::on_editSaveButton_clicked()
{
    // 選択状態を解除する.
    routeInformationSelectionModel_->clear();

    // メインの画面クラスに完了を通知.
    emit readyToSwitch(SwitchPageEvent::DONE);
}

void EditingPage::on_spinBoxWaitingTime_valueChanged(int arg1)
{
    if(!routeInformationSelectionModel_->selectedIndexes().isEmpty())
    {
        int selectedIndex = routeInformationSelectionModel_->selectedIndexes().first().row();
        routeInformation_->setDataWaitingTime(selectedIndex, static_cast<unsigned int>(arg1));
    }
}

void EditingPage::on_emergencyButton_clicked()
{
    if(DialogFactory::telecommandCheck(COMMAND_NAME_EMERGENCY_STOP))
    {
        if(!telecommandClient_->sendCtlCommandCancel())
        {
            // 送信失敗.
            DialogFactory::alert(DIALOG_MSG_COMMAND_SEND_ERROR);
        }
    }
}
