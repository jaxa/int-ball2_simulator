
#pragma once

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <ros/service_server.h>
#include <ros/service_client.h>
//#include "ctl/fsm.h"    // Modification for platform packages
#include "ctl/dtc.h"

// Standard messages
#include <std_msgs/Int8.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h> 
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "ib2_msgs/Navigation.h"
#include "ib2_msgs/CtlStatusType.h"
#include "ib2_msgs/CtlStatus.h"
#include "ib2_msgs/CtlProfile.h"
#include "ib2_msgs/CtlCommandAction.h"
#include "ib2_msgs/UpdateParameter.h"

#include <memory>

// マクロ宣言、後でどこか共通の*.hに移動したほうがいいかも
#define TOPIC_CTL_WRENCH  "/ctl/wrench"
#define TOPIC_CTL_PROFILE "/ctl/profile"
#define TOPIC_CTL_STATUS  "/ctl/status"
#define TOPIC_NAV_POSE    "/sensor_fusion/navigation"
#define TOPIC_UP_WRENCH_MODE  "/ib2_user/guidance_control/wrench_mode"
#define TOPIC_UP_TORQUE_GAIN  "/ib2_user/guidance_control/torque_gain"


namespace ib2
{
	// classの前方宣言
	class CtlBody;
	class PosAttController;
	class PosAttProfiler;
}
	
/**
* @brief 制御ノードクラス
*/
class Ctl
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
private:
	/** デフォルトコンストラクタ */
	Ctl() = delete;

public:
	/** コンストラクタ */
	explicit Ctl(const ros::NodeHandle& nh);

	/** デストラクタ */
	~Ctl();

	/** Wrench mode (User Program) */
	enum class UP_WRENCH_MODE : unsigned int
	{
		NORMAL=0,	///< Normal
		USE_TORQUE_GAIN_FORWARD_IB2=1 ///< Add torque gain forward to IB2.
	};

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	Ctl(const Ctl&) = delete;

	/** コピー代入演算子. */
	Ctl& operator=(const Ctl&) = delete;

	/** ムーブコンストラクタ. */
	Ctl(Ctl&&) = delete;

	/** ムーブ代入演算子. */
	Ctl& operator=(Ctl&&) = delete;
    
	//----------------------------------------------------------------------
	// 操作(Setter)
private:
	/** メンバ設定
	 * @retval true 設定成功
	 * @retval false 設定失敗
	 */
	bool setMember();
	
	/** 位置姿勢保持設定 */
	void setKeepPose();

	//----------------------------------------------------------------------
	// 実装
private:
	/** 制御目標への誘導
	 * @param [in] goalType 制御目標種別
	 * @param [in] tolp 位置誤差許容値[m]
	 * @param [in] tola 姿勢誤差許容値[rad]
	 * @retval true 目標到達
	 * @retval false 中断
	 */
	bool guidance(int32_t goal_type, double tolp, double tola);
	
	/** ターゲットモードの処理
	 * @param [in] goal 制御目標
	 */
	void target(const ib2_msgs::CtlCommandGoalConstPtr& goal);
	
	/** リリースモードの処理 */
	void release();
	
	/** ドッキングモードの処理
	 * @param [in] correction マーカー補正フラグ(true更新あり/false補正なし)
	 */
	void docking(bool correction);
	
	/** ドッキングスタンバイモードの処理 */
	void dockingStandBy();
	
	/** スキャンモードの処理 */
	void scan();
	
	/** 停止誘導モードの処理 */
	void stopping();
	
	/** アクション中止
	 * @param [in] reult_type アクション結果種別
	 */
	void abortAction(uint8_t reult_type);
	
	/** 制御目標キャンセル時の処理 */
	void cancelTarget();

	/** 制御目標到達時の処理 */
	void goalTarget();

	/** 航法メッセージタイムアウト処理 */
	void timeoutNavigation();

	/** 制御目標到達判定
	 * @param [in, out] stay 制御目標周辺継続判定結果
	 * @param [in, out] tin 制御目標周辺到達時刻
	 * @param [in] tnav 最新の航法時刻
	 * @param [in] fb アクションフィードバック
	 * @param [in] tolp 位置誤差許容値[m]
	 * @param [in] tola 姿勢誤差許容値[rad]
	 * @return 制御目標到達判定結果
	 */
	bool reachGoal
	(bool& stay, ros::Time& tin, const ros::Time& tnav,
	 const ib2_msgs::CtlCommandFeedback& fb, double tolp, double tola);

	/** 制御目標到達判定(SCAN)
	 * @param [in, out] stay 制御目標周辺継続判定結果
	 * @param [in, out] tin 制御目標周辺到達時刻
	 * @param [in] tnav 最新の航法時刻
	 * @param [in] fb アクションフィードバック
	 * @param [in] tola 姿勢誤差許容値[rad]
	 * @return 制御目標到達判定結果
	 */
	bool reachGoalScan
	(bool& stay, ros::Time& tin, const ros::Time& tnav,
	 const ib2_msgs::CtlCommandFeedback& fb, double tola);

	/** 制御目標到達判定(DOCK)
	 * @return 制御目標到達判定結果
	 */
	bool reachGoalDock();

	/** 制御目標妥当性確認
	 * @param [in] goal 制御目標
	 * @retval true 妥当
	 * @retval false 不正
	 */
	bool validCommand(const ib2_msgs::CtlCommandGoalConstPtr& goal) const;

	/** 航法メッセージ妥当性確認
	 * @param [in] nav 判定対象航法メッセージ
	 * @param [in] first 初期フラグ(true:初めて / false : 初めてでない)
	 * @retval true 妥当
	 * @retval false 不正
	 */
	bool validNavigation(const ib2_msgs::Navigation& nav, bool first) const;

	//--------------------------------------------------------------------------
	// 実装（コールバック関数）
public:
	/** 制御目標アクション受信時の処理
	 * @param [in] goal 制御目標値メッセージ
	 */
	void commandCallback(const ib2_msgs::CtlCommandGoalConstPtr& goal);

	/** パラメータ更新サービス受信時の処理
	 * @param [in] パラメータ更新サービスリクエスト
	 * @param [in] res パラメータ更新サービス実行結果
	 * @retval true 更新成功
	 * @retval false 更新失敗
	 */
	bool updateCallback
	(ib2_msgs::UpdateParameter::Request&,
	 ib2_msgs::UpdateParameter::Response& res);

	/** 航法値のサブスクライバのコールバック関数
	 * @param [in] nav_stamp 航法値
	 */
	void navinfoCallback(const ib2_msgs::Navigation& nav_stamp);

	/** Callback method for Wrench mode. (User Program)
	 * @param [in] msg: Wrench mode id.
	 *                  1: Normal mode [force, torque]
	 *                  2: Forward to IB2 mode [force, torque - torque_gain.cross(torque)]
	 * @ref upTorqueGainCallback method.
	 */
	void upWrenchModeCallback(const std_msgs::Int8ConstPtr& msg);

	/** Callback method for Toruqe gain. (User Program)
	 * @param [in] msg: Toruqe gain [x,y,z]
	 * @ref upWrenchModeCallback method.
	 */
	void upTorqueGainCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

	/** 定期的な処理
	 * @param [in] ev タイマーイベント
	 */
	void timerCallback(const ros::TimerEvent& ev);

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ROSノードハンドラ */
	ros::NodeHandle nh_;

	/** 制御目標アクションサーバ */
	actionlib::SimpleActionServer<ib2_msgs::CtlCommandAction> command_as_;
	
	/** パラメータ更新サービスサーバ */
	ros::ServiceServer update_ss_;

	/** マーカー補正サービスクライアント */
	ros::ServiceClient marker_sc_;
	
	/** 誘導制御ステータス出力間隔 */
	ros::Duration interval_status_;
	
	/** フィードバック間隔 */
	ros::Duration interval_feedback_;

	/** 目標到達継続時間 */
	ros::Duration duration_goal_;
	
	/** 制御目標位置到達判定値[m] */
	double tolerance_pos_;

	/** 制御目標姿勢到達判定値[rad] */
	double tolerance_att_;
	
	/** 位置停止判定値[m] */
	double tolerance_pos_stop_;
	
	/** 姿勢停止判定値[rad] */
	double tolerance_att_stop_;
	
	/** 航法異常連続上限 */
	size_t nav_counter_;
	
	/** 航法位置変動量上限[m/s] */
	double nav_dr_;
	
	/** 航法速度変動量上限[m/s2] */
	double nav_dv_;
	
	/** 航法加速度変動量上限[m/s3] */
	double nav_da_;

	/** 航法姿勢変動量上限[rad/s] */
	double nav_dq_;
	
	/** 航法角速度変動量上限[rad/s2] */
	double nav_dw_;
	
	/** ターゲットキャンセルの待ち時間 */
	ros::Duration waitCancel_;
	
	/** リリース開始からAIP移動開始までの待ち時間 */
	ros::Duration waitRelease_;
	
	/** ホーミング時VisualSLAM較正の待ち時間 */
	ros::Duration waitCalibration_;
	
	/** ドッキング開始からスタンバイまでの待ち時間 */
	ros::Duration waitDocking_;
	
    // Subscriber
	/** 航法値のサブスクライバ */
    ros::Subscriber navinfo_sub_;

	/** 目標値のサブスクライバ */
    ros::Subscriber target_sub_;

	/** Wrench mode subscriber. (User Program) */
    ros::Subscriber up_wrench_mode_sub_;

	/** Torque gain subscriber. (User Program) */
	ros::Subscriber up_torque_gain_sub_;

	// Publisher
	/** 誘導制御モードパブリッシャ */
	ros::Publisher status_pub_;

	/** 力トルクのパブリッシャ */
    ros::Publisher wrench_pub_;

	/** 制御プロファイルのパブリッシャ */
	ros::Publisher profile_pub_;

	/** 航法メッセージの前回値 */
	ib2_msgs::Navigation last_nav_stamp_;

	/** 機体パラメータ */
	std::unique_ptr<ib2::CtlBody> body_;

	/** 誘導制御則 */
	std::unique_ptr<ib2::PosAttController> controller_;

	/** 位置姿勢誘導プロファイル */
	std::unique_ptr<ib2::PosAttProfiler> profiler_;

	/** ファン選択 */
	//std::unique_ptr<Fsm> fsm_;    // Modification for platform packages

	/** 検知 */
	Dtc dtc_;

	/**  誘導制御モード */
	int32_t status_;

	/** ステータス出力タイマー */
	ros::Timer timer_;

	/** CtlStatus sequcens id  */
	mutable uint32_t seq_status_;
	
	/** 航法えメッセージ取得フラグ */
	bool valid_navigation_;

	/** Wrench mode (User Program) */
    UP_WRENCH_MODE up_wrench_mode_;

	/** Torque gain (User Program) */
    Eigen::Vector3d up_torque_gain_;

};

// End Of File -----------------------------------------------------------------
