#ifndef IB2_PATH_RENDER_H
#define IB2_PATH_RENDER_H

#include <QWidget>
#include <QPainter>
#include <QVector3D>
#include <QQuaternion>
#include <QSharedPointer>
#include <QSvgRenderer>
#include "view/route_information_view.h"

namespace intball
{

class RouteEditor : public intball::RouteInformationView
{
    Q_OBJECT
public:
    /**
     * @brief 軸の種別.
     */
    enum AxisType
    {
        /** @brief 左上原点, 横軸X, 縦軸Z, 横からの視点（Side view）. */
        X_Z,

        /** @brief 左上原点, 横軸X, 縦軸Y, 上からの視点（Top-down view）.*/
        X_Y
    };

    /**
     * @brief RouteEditorコンストラクタ.
     * @param parent 親ウィジット.
     */
    explicit RouteEditor(QWidget *parent);

    void mousePressEvent(QMouseEvent *e) override;
    void mouseMoveEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;

    /**
     * @brief 軸の種別を設定する.
     * @param type 軸の種別.
     */
    void setAxisType(const AxisType type);

    /**
     * @brief マウスイベントの検知有無を設定する.
     * @param flag マウスイベントの検知有無（true=検知有り）
     */
    void setDetectMouseEvent(const bool flag);

    /**
     * @brief Int-Ball2位置を固定表示有無を指定する.
     * @param flag 固定表示有無（true=固定表示）
     * @detailes 固定表示trueを指定したタイミングのInt-Ball2位置が、固定表示される.
     */
    void setFixedStart(const bool flag);

    /**
     * @brief 初期化処理.
     * @param transform Int-Ball2位置情報を表示するためのTransform.
     */
    void initialize(const QTransform& transform);

    /**
     * @brief JEM船内に収まる座標値に丸める.
     * @param data 座標.
     * @return 丸めた座標値.
     */
    QVector3D roundToValidRoutePoint(const QVector3D& data);

protected:
    virtual void paintEvent(QPaintEvent *) override;
    virtual void on_dataChanged(const QModelIndex &index, const RoutePoint& data) override;
    virtual void on_rowsAboutToBeRemoved(int firstToBeRemoved, int lastToBeRemoved) override;
    virtual void on_rowsInserted(int first, int last) override;
    virtual void on_selectionChanged(const QItemSelection &selected, const QItemSelection &deselected) override;
private:
    int detectNewPointIndex(const QPointF &pos);
    bool isInViewport(const QPointF &point);
    void insertData(const int index, const QPointF &point);
    void setData(const int index, const QPointF &point);
    void updateView(const int removeStart = -1, const int removeEnd = -1);
    bool isEffectiveMousePoint(const QPointF &point);
    QPointF getTransformedPoint(const QVector3D& data);
    QPointF roundPoint(const QPointF& point);

    int maxPointSize_;
    AxisType axis_;
    int pointRadius_;
    int activePoint_;
    QRectF jemBodyRect_;
    bool mouseDrag_;
    bool detectMouseEvent_;
    bool fixedStart_;
    qreal jemScale_;
    QVector3D angleBase_;
    QTransform paintTransform_;
    QVector<QPointF> rendererPoints_;
    QVector<int> rendererAngles_;
    QPoint mousePress_;
    QPointF fixedStartPoint_;
    QSvgRenderer* backgroundRendererSide_;
    QSvgRenderer* backgroundRendererTop_;
};

} // namespace intball

#endif // IB2_PATH_RENDER_H
