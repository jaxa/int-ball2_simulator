#include <ignition/math/Pose3.hh>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ib2_msgs/CtlProfile.h"
#include "ib2_msgs/CtlStatus.h"
#include "ib2_msgs/CtlStatusType.h"
#include "tf/transform_listener.h"

namespace gazebo
{
class Ib2RouteDisplayPlugin : public WorldPlugin
{
  public:
    explicit Ib2RouteDisplayPlugin();

  protected:
    virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf) override;

  private:

		/** タイマーイベントのコールバック.
		 * @param [in] _event タイマーイベント.
		 */
    void timerCallback(const ros::TimerEvent& _event);

		/** 移動履歴マーカーの制御処理.
		 * @param [in] pose 移動履歴（過去のある時点でのInt-Ball2位置姿勢）.
		 */
    void controlHistoryMarkers(const geometry_msgs::Pose& pose);

		/** 移動経路マーカーの制御処理.
		 */
    void controlRouteMarkers();

		/** 誘導制御ステータスのSubscriberのコールバック.
		 * @param [in] msg 誘導制御ステータス.
		 */
    void ctlStatusCallback(const ib2_msgs::CtlStatus& msg);

		/** 誘導制御プロファイルのSubscriberのコールバック.
		 * @param [in] msg 誘導制御プロファイル.
		 */
    void ctlProfileCallback(const ib2_msgs::CtlProfile& msg);

    /** 移動履歴マーカーのID最小値. */
    static constexpr unsigned int HISTORY_INDEX_MIN = 1;

    /** シミュレーション中の移動経路マーカーの最大表示数　デフォルト値. */
    static constexpr int DEFAULT_HISTORY_MAX = 50;

    /** 移動経路マーカー間の最小距離　デフォルト値. */
    static constexpr double DEFAULT_HISTORY_MIN_DISTANCE = 0.1;

    /** 移動履歴マーカーの出力を強制するフラグ.1回出力するたびにフラグがOFFになる. */
    bool force_publish_history_;

    /** Int-Ball2が移動中であるかを示すフラグ. */
    bool is_ib2_moving_;

    /** 移動経路マーカーが削除済み（非表示）であるかどうかを示すフラグ. */
    bool is_route_marker_deleted_;

    /** 移動経路マーカー間の最小距離. */
    double history_min_distance_;

    /** 最後に受信した誘導制御プロファイル値. */
    ib2_msgs::CtlProfile ctl_profile_;

    /** 移動中に受信した誘導制御ステータス値の最新値（更新するのは移動中のみ）. */
    ib2_msgs::CtlStatus ctl_status_;

    /** ROS関連処理用NodeHandle. */
    ros::NodeHandle nh_;

    /** 移動履歴マーカーのPublisher. */
    ros::Publisher pub_history_;

    /** 移動経路マーカーのPublisher. */
    ros::Publisher pub_route_;

    /** 誘導制御プロファイルのSubscriber. */
    ros::Subscriber sub_ctl_profile_;

    /** 誘導制御ステータスのSubscriber. */
    ros::Subscriber sub_ctl_status_;

    /** 最後にInt-Ball2の移動停止を検知した時刻. */
    ros::Time last_ib2_stop_timestamp_;

    /** メイン処理ループ用のタイマー. */
    ros::Timer timer_;

    /** 移動経路マーカーのFrame ID. */
    std::string history_frame_id_;

    /** TFのリスナー. */
    std::unique_ptr<tf::TransformListener> tf_listener_ptr_;

    /** シミュレーション中の移動経路マーカーの最大表示数. */
    int history_max_;

    /** 最後にPublishされた移動履歴マーカー（visualization_msgs::Marker）のID. */
    unsigned int last_published_history_id_;

    /** 移動履歴マーカーのメッセージ. */
    visualization_msgs::Marker msg_history_;

    /** 移動経路マーカーのメッセージ. */
    visualization_msgs::MarkerArray msg_route_;

};

GZ_REGISTER_WORLD_PLUGIN(Ib2RouteDisplayPlugin)
}
