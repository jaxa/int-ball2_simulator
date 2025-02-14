#include <QPaintEngine>
#include <QPainter>
#include <QMouseEvent>
#include <QTimerEvent>
#include <QEvent>
#include <QSvgRenderer>
#include <QTouchEvent>
#include <cfloat>
#include "operator_gui_config.h"
#include "gui_color.h"
#include "model/route_information.h"
#include "model/route_point.h"
#include "qdebug_custom.h"
#include "route_editor.h"

using namespace intball;
using namespace intball::qsettings;
using namespace intball::qsettings::key;

inline void RouteEditor::insertData(const int index, const QPointF &point)
{
    switch (axis_)
    {
    case AxisType::X_Z:
        thisModel()->insertData(index, QVector3D(point.x(), 0, point.y()));
        break;
    case AxisType::X_Y:
        thisModel()->insertData(index, QVector3D(point.x(), point.y(), 0));
        break;
    default:
        Q_ASSERT_X(false, __FUNCTION__, QString::asprintf("Invalid axis_: %d", axis_).toStdString().c_str());
        break;
    }
}

inline void RouteEditor::setData(const int index, const QPointF &point)
{
    switch (axis_)
    {
    case AxisType::X_Z:
        thisModel()->setDataPosition(index, QVariant(point.x()), QVariant(), QVariant(point.y()));
        break;
    case AxisType::X_Y:
        thisModel()->setDataPosition(index, QVariant(point.x()), QVariant(point.y()), QVariant());
        break;
    default:
        Q_ASSERT_X(false, __FUNCTION__, QString::asprintf("Invalid axis_: %d", axis_).toStdString().c_str());
        break;
    }
}

inline bool RouteEditor::isInViewport(const QPointF &point)
{
    if (point.x() < 0 || point.x() > width() || point.y() < 0 || point.y() > height())
    {
        return false;
    }
    else
    {
        return true;
    }
}

RouteEditor::RouteEditor(QWidget *parent)
    : RouteInformationView(parent), axis_(AxisType::X_Z), pointRadius_(6), activePoint_(-1),
      jemBodyRect_(QRectF(QPointF(0.0, 0.0), QPointF(36.454, 8.875))),
      detectMouseEvent_(true), fixedStart_(false), jemScale_(1)
{
    maxPointSize_ = Config::valueAsInt(KEY_ROUTE_POINT_SIZE_MAX);

    setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    QPalette pal(palette());
    pal.setColor(QPalette::Background, Qt::white);
    setAutoFillBackground(true);
    setPalette(pal);

    backgroundRendererSide_ = new QSvgRenderer(QString(":/ground_system/image/img_jem_side.svg"));
    backgroundRendererTop_ = new QSvgRenderer(QString(":/ground_system/image/img_jem_top.svg"));
}

void RouteEditor::initialize(const QTransform& transform)
{
    paintTransform_ = transform;
    qreal widthScale = width() / jemBodyRect_.width();
    qreal heightScale = height() / jemBodyRect_.height();
    jemScale_ = widthScale > heightScale ? heightScale * 0.8 : widthScale * 0.8;

    jemBodyRect_.setWidth(jemBodyRect_.width() * jemScale_);
    jemBodyRect_.setHeight(jemBodyRect_.height() * jemScale_);
    jemBodyRect_.moveCenter(QPointF(static_cast<double>(width()) / 2, static_cast<double>(height()) / 2));

    switch(axis_)
    {
    case X_Z:
        paintTransform_.translate(jemBodyRect_.x() + 0.2 * jemScale_, jemBodyRect_.y() + 1 * jemScale_);
        break;
    case X_Y:
        paintTransform_.translate(jemBodyRect_.x() + 0.2 * jemScale_, jemBodyRect_.y() + (jemBodyRect_.height() / 2));
        break;
    }
    // NOTE: 現状設定パラメータ化は無し
    paintTransform_.scale(4.4 * jemScale_, -4.4 * jemScale_);

    angleBase_ = QVector3D(1, 0, 0);
}

void RouteEditor::setAxisType(const AxisType type)
{
    INFO_START_FUNCTION() << "type=" << type;
    axis_ = type;
}

void RouteEditor::setDetectMouseEvent(const bool flag)
{
    INFO_START_FUNCTION() << "flag=" << flag;
    detectMouseEvent_ = flag;
}

void RouteEditor::setFixedStart(const bool flag)
{
    INFO_START_FUNCTION() << "flag=" << flag;
    fixedStart_ = flag;
    if(flag && rendererPoints_.size() > 0)
    {
        fixedStartPoint_ = rendererPoints_.first();
    }
}

void RouteEditor::paintEvent(QPaintEvent *event)
{
    if (rendererPoints_.isEmpty())
    {
        return;
    }

    /*
     * Viewではviewportに紐づくQPainterを取得する.
     * QPainterはbegin済み状態となる.
     */
    QPainter painter(viewport());
    painter.setClipRect(event->rect());
    painter.setRenderHint(QPainter::Antialiasing);

    QPointF origin = paintTransform_.map(QPointF(0, 0));
    QPointF xAxis = paintTransform_.map(QPointF(0.5, 0));
    QPointF yAxis = paintTransform_.map(QPointF(0, 0.5));
    switch(axis_)
    {
    case X_Z:
        // 軸の描画.
        painter.setPen(QPen(Qt::red, 2));
        painter.drawLine(origin, xAxis);
        painter.setPen(QPen(Qt::blue, 2));
        painter.drawLine(origin, yAxis);

        // 背景の描画
        backgroundRendererTop_->render(&painter, jemBodyRect_);
        break;
    case X_Y:
        // 軸の描画.
        painter.setPen(QPen(Qt::red, 2));
        painter.drawLine(origin, xAxis);
        painter.setPen(QPen(Qt::green, 2));
        painter.drawLine(origin, yAxis);

        // 背景の描画
        backgroundRendererSide_->render(&painter, jemBodyRect_);
        break;
    }

    // JEM方向ラベルを描画する.
    painter.setPen(Color::F2);
    switch(axis_)
    {
    case X_Z:
        painter.drawText(QRectF(QPointF(0.0, 0.0), QPointF(width(), jemBodyRect_.top())),
                         Qt::AlignCenter, "Ovhd");
        painter.drawText(QRectF(QPointF(0.0, jemBodyRect_.bottom()),
                                QPointF(width(), height())),
                         Qt::AlignCenter, "Deck");
        break;
    case X_Y:
        painter.drawText(QRectF(QPointF(0.0, 0.0),
                                QPointF(width(),jemBodyRect_.top())),
                         Qt::AlignCenter, "Aft");
        painter.drawText(QRectF(QPointF(0.0, jemBodyRect_.bottom()),
                                QPointF(width(), height())),
                         Qt::AlignCenter, "Fwd");
        break;
    }
    painter.drawText(QRectF(QPointF(0.0, 0.0),QPointF(jemBodyRect_.left(), height())), Qt::AlignCenter, "Stbd");
    painter.drawText(QRectF(QPointF(jemBodyRect_.right(), 0.0),
                            QPointF(width(), height())), Qt::AlignCenter, "Port");

    // 経路線を描画する.
    painter.setPen(QPen(Color::F4, 0, Qt::DashLine));
    painter.setBrush(Qt::NoBrush);
    if(fixedStart_)
    {
        // 経路線の開始地点を固定する.
        QVector<QPointF> drawPoints;
        drawPoints << fixedStartPoint_;
        drawPoints.append(rendererPoints_.mid(1));
        painter.drawPolyline(drawPoints);

        // 固定した経路の開始地点を描画する.
        painter.setPen(QPen(Color::U4, 0, Qt::DotLine));
        painter.setBrush(Color::F3);
        painter.drawEllipse(QRectF(fixedStartPoint_.x() - pointRadius_,
                                   fixedStartPoint_.y() - pointRadius_,
                                   pointRadius_ * 2, pointRadius_ * 2));
    }
    else
    {
        // 通常の経路線の描画.
        painter.drawPolyline(rendererPoints_);
    }

    // 経路点を描画する.
    painter.setPen(Color::TRANSPARENT);
    for (int i = 0; i < rendererPoints_.size(); ++i)
    {
        if (i == 0)
        {
            // Int-Ball現在位置.
            painter.setBrush(Color::F2);
        }
        else if (i == rendererPoints_.size() - 1)
        {
            // ゴール.
            painter.setBrush(Color::F4);
        }
        else if (i == 1)
        {
            painter.setBrush(Color::F3);
        }

        QPointF pos = rendererPoints_.at(i);

        // 点の描画.
        painter.drawEllipse(QRectF(pos.x() - pointRadius_,
                                   pos.y() - pointRadius_,
                                   pointRadius_ * 2, pointRadius_ * 2));

        // 角度の描画.
        auto angle = rendererAngles_.at(i);

        painter.translate(pos);

        QPointF triangle1 = QPointF(pointRadius_ + 15, 0);
        QPointF triangle2 = QPointF(triangle1.x() - 10, triangle1.y() + 5);
        QPointF triangle3 = QPointF(triangle1.x() - 10, triangle1.y() - 5);
        const QPointF trianglePoints[3] = {triangle1, triangle2, triangle3};

        painter.rotate(angle);
        painter.drawPolygon(trianglePoints, 3);
        painter.rotate(-angle);

        painter.translate(-pos);
    }

    // 選択されている点の描画.
    auto selected = selectedIndexes();
    if(selected.size() > 0) {
        for(auto i = selected.begin(); i != selected.end(); ++i) {
            auto target = rendererPoints_.at((*i).row());

            painter.setPen(QPen(Color::U_HIGHLIGHT, 3, Qt::DashLine));
            painter.setBrush(Color::TRANSPARENT);
            painter.drawEllipse(QRectF(target.x() - pointRadius_ - 10,
                                       target.y() - pointRadius_ - 10,
                                       (pointRadius_ + 10) * 2, (pointRadius_ + 10) * 2));
        }
    }
}

int RouteEditor::detectNewPointIndex(const QPointF &pos)
{
    const int pointsSize = rendererPoints_.size();
    if (pointsSize < 2)
    {
        return pointsSize;
    }
    int next_index = -1;
    float sum_distance = FLT_MAX;
    float sum_distance_tmp = 0.0f;

    // 最も近い点を検出する.
    for (int i = 0; i < pointsSize - 1; ++i)
    {
        sum_distance_tmp = QVector2D(pos).distanceToPoint(QVector2D(QLineF(rendererPoints_.at(i), rendererPoints_.at(i + 1)).center()));
        if (sum_distance_tmp < sum_distance)
        {
            sum_distance = sum_distance_tmp;
            next_index = i + 1;
        }
    }

    // 最も近い点が最終地点（ゴール)であった場合,
    // 途中点の追加かゴールの追加かを判定する.
    if (next_index == pointsSize - 1)
    {
        qreal distance_new = QLineF(rendererPoints_.at(pointsSize - 2), pos).length();
        qreal distance_exist = QLineF(rendererPoints_.at(pointsSize - 2), rendererPoints_.at(pointsSize - 1)).length();
        if (distance_new > distance_exist)
        {
            next_index = pointsSize;
        }
    }

    return next_index;
}

void RouteEditor::mousePressEvent(QMouseEvent *e)
{
    if (!detectMouseEvent_)
    {
        return;
    }

    if (activePoint_ != -1)
    {
        return;
    }

    qreal distance = -1;
    for (int i = 0; i < rendererPoints_.size(); ++i)
    {
        qreal d = QLineF(e->pos(), rendererPoints_.at(i)).length();
        if (distance < 0 || d < distance)
        {
            distance = d;
            activePoint_ = i;
        }
    }

    LOG_INFO() << "mousePressEvent: activePoint_=" << activePoint_ << " action=" << e->button()
               << " x=" << e->pos().x() << " y=" << e->pos().y() << " distance=" << distance;

    if (e->button() == Qt::MiddleButton && isEffectiveMousePoint(e->pos()))
    {
        if(thisModel()->rowCount() >= maxPointSize_)
        {
            LOG_INFO() << "Reached the maximum size of points.";
            return;
        }

        /*
         * 初期追加,または既存点から離れた位置の指定時のみ
         * 処理を実行する.
         */
        if((distance < 0) || (distance > pointRadius_* 2))
        {
            int index = detectNewPointIndex(e->pos());
            if (index > 0)
            {
                // Int-Ballの現在位置(i=0)以外であれば経路の点を追加する.
                insertData(index, paintTransform_.inverted().map(QPointF(e->pos().x(), e->pos().y())));
                activePoint_ = index;
            }
        }
    }
    else
    {
        // Int-Ballの現在位置(i=0)の点に対する操作, 点の中心から離れた操作は無視する.
        if (activePoint_ > 0 && distance < 10)
        {
            if (e->button() == Qt::RightButton)
            {
                thisModel()->removeRows(activePoint_, 1);
            }
            else
            {
                mouseMoveEvent(e);
            }
        }
    }

    mousePress_ = e->pos();
}

void RouteEditor::mouseMoveEvent(QMouseEvent *e)
{
    if (!detectMouseEvent_)
    {
        return;
    }

    // 点の半径以上の距離を移動した場合のみマウスドラッグ（移動）と判定する.
    if (QPoint(mousePress_ - e->pos()).manhattanLength() > pointRadius_)
    {
        mouseDrag_ = true;
    }
    else
    {
        mouseDrag_ = false;
    }

    /*
     * Int-Ballの現在位置(i=0)に対する操作,
     * JEMの枠外に対する操作は無視する.
     */
    if (mouseDrag_ && activePoint_ > 0 && activePoint_ < rendererPoints_.size() && isEffectiveMousePoint(e->pos()))
    {
        setData(activePoint_, paintTransform_.inverted().map(QPointF(e->pos().x(), e->pos().y())));
    }
}

void RouteEditor::mouseReleaseEvent(QMouseEvent *e)
{
    if (!detectMouseEvent_)
    {
        return;
    }
    if (activePoint_ != -1 && !isInViewport(e->pos()))
    {
        setData(activePoint_, paintTransform_.inverted().map(QPointF(mousePress_.x(), mousePress_.y())));
    }
    activePoint_ = -1;
}

void RouteEditor::updateView(const int firstToBeRemoved, const int lastToBeRemoved)
{
    Q_ASSERT(firstToBeRemoved <= lastToBeRemoved);
    if(thisModel()->rowCount() > 0)
    {
        QVector<QPointF>().swap(rendererPoints_);
        QVector<int>().swap(rendererAngles_);
        for(int i = 0; i < thisModel()->rowCount(); ++i)
        {
            if((i >= firstToBeRemoved) && (i <= lastToBeRemoved))
            {
                // 削除対象の点は無視する.
                continue;
            }

            auto data = thisModel()->data(i);
            auto xAxisVector = data.orientation().rotatedVector(angleBase_);

            // QPainterでカメラの向きを描画する際の角度.
            qreal angle = 0;
            QPointF point;

            switch (axis_)
            {
            case X_Z:
                point.setX(static_cast<qreal>(data.position().x()));
                point.setY(static_cast<qreal>(data.position().z()));
                // QLineFのangleは反時計周りだが、angleを利用するPainterでは時計周り指定なので符号を反転する.
                angle = -QLineF(0, 0, static_cast<qreal>(xAxisVector.x()), static_cast<qreal>(-xAxisVector.z())).angle();
                break;
            case X_Y:
                point.setX(static_cast<qreal>(data.position().x()));
                point.setY(static_cast<qreal>(data.position().y()));
                // QLineFのangleは反時計周りだが、angleを利用するPainterでは時計周り指定なので符号を反転する.
                angle = -QLineF(0, 0, static_cast<qreal>(xAxisVector.x()), static_cast<qreal>(-xAxisVector.y())).angle();
                break;
            default:
                Q_ASSERT_X(false, __FUNCTION__, QString::asprintf("Invalid axis_: %d", axis_).toStdString().c_str());
                break;
            }
            rendererPoints_ << paintTransform_.map(point);
            rendererAngles_ << static_cast<int>(angle);
        }
    }

    update();
}

void RouteEditor::on_dataChanged(const QModelIndex &index, const RoutePoint& data)
{
    Q_UNUSED(index);
    Q_UNUSED(data);
    updateView();
}

void RouteEditor::on_rowsAboutToBeRemoved(int firstToBeRemoved, int lastToBeRemoved)
{
    INFO_START_FUNCTION() << "firstToBeRemoved=" << firstToBeRemoved << " lastToBeRemoved=" << lastToBeRemoved;
    updateView(firstToBeRemoved, lastToBeRemoved);
}

void RouteEditor::on_rowsInserted(int first, int last)
{
    INFO_START_FUNCTION() << "first=" << first << " last=" << last;
    updateView();

}

void RouteEditor::on_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected)
{
    INFO_START_FUNCTION() << "selected=" << selected << " last=" << deselected;
    updateView();
}

bool RouteEditor::isEffectiveMousePoint(const QPointF &point)
{
    return jemBodyRect_.contains(point.x(), point.y());
}

QPointF RouteEditor::getTransformedPoint(const QVector3D& data)
{
    QPointF point;

    switch (axis_)
    {
    case X_Z:
        point.setX(static_cast<qreal>(data.x()));
        point.setY(static_cast<qreal>(data.z()));
        break;
    case X_Y:
        point.setX(static_cast<qreal>(data.x()));
        point.setY(static_cast<qreal>(data.y()));
        break;
    default:
        Q_ASSERT_X(false, __FUNCTION__, QString::asprintf("Invalid axis_: %d", axis_).toStdString().c_str());
        break;
    }

    return paintTransform_.map(point);
}

QPointF RouteEditor::roundPoint(const QPointF& point)
{
    float x, y;
    x = static_cast<float>(point.x());
    if(x < static_cast<float>(jemBodyRect_.left()))
    {
        x = static_cast<float>(jemBodyRect_.left() + 1);
    }
    else if(x > static_cast<float>(jemBodyRect_.right()))
    {
        x = static_cast<float>(jemBodyRect_.right() - 1);
    }
    y = static_cast<float>(point.y());
    if(y < static_cast<float>(jemBodyRect_.top()))
    {
        y = static_cast<float>(jemBodyRect_.top() + 1);
    }
    else if(y > static_cast<float>(jemBodyRect_.bottom()))
    {
        y = static_cast<float>(jemBodyRect_.bottom() - 1);
    }

    return QPointF(static_cast<qreal>(x), static_cast<qreal>(y));
}

QVector3D RouteEditor::roundToValidRoutePoint(const QVector3D& data)
{
    QVector3D returnPoint(data);
    QPointF rounded = paintTransform_.inverted().map(roundPoint(getTransformedPoint(data)));

    switch (axis_)
    {
    case X_Z:
        returnPoint.setX(static_cast<float>(rounded.x()));
        returnPoint.setZ(static_cast<float>(rounded.y()));
        break;
    case X_Y:
        returnPoint.setX(static_cast<float>(rounded.x()));
        returnPoint.setY(static_cast<float>(rounded.y()));
        break;
    default:
        Q_ASSERT_X(false, __FUNCTION__, QString::asprintf("Invalid axis_: %d", axis_).toStdString().c_str());
        break;
    }

    return returnPoint;
}
