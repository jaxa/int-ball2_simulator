
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ctl/dtc.h"

// Standard messages
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ib2_msgs/msg/navigation.hpp"
#include "ib2_msgs/msg/ctl_status_type.hpp"
#include "ib2_msgs/msg/ctl_status.hpp"
#include "ib2_msgs/msg/ctl_profile.hpp"
#include "ib2_msgs/action/ctl_command.hpp"
#include "ib2_msgs/srv/update_parameter.hpp"
#include "ib2_msgs/srv/marker_correction.hpp"

#include <memory>
#include <thread>

#define TOPIC_CTL_WRENCH  "/ctl/wrench"
#define TOPIC_CTL_PROFILE "/ctl/profile"
#define TOPIC_CTL_STATUS  "/ctl/status"
#define TOPIC_NAV_POSE    "/sensor_fusion/navigation"

namespace ib2
{
	class CtlBody;
	class PosAttController;
	class PosAttProfiler;
}

/**
* @brief 制御ノードクラス
*/
class Ctl : public rclcpp::Node
{
public:
	// Action types
	using CtlCommandAction = ib2_msgs::action::CtlCommand;
	using GoalHandleCtlCommand = rclcpp_action::ServerGoalHandle<CtlCommandAction>;

	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	Ctl();

	/** デストラクタ */
	~Ctl();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	Ctl(const Ctl&) = delete;
	Ctl& operator=(const Ctl&) = delete;
	Ctl(Ctl&&) = delete;
	Ctl& operator=(Ctl&&) = delete;

	//----------------------------------------------------------------------
	// 操作(Setter)
private:
	bool setMember();
	void setKeepPose();

	//----------------------------------------------------------------------
	// 実装
private:
	bool guidance(int32_t goal_type, double tolp, double tola);
	void target(const std::shared_ptr<const CtlCommandAction::Goal>& goal);
	void release();
	void docking(bool correction);
	void dockingStandBy();
	void scan();
	void stopping();
	void abortAction(uint8_t reult_type);
	void cancelTarget(bool docking = false);
	void goalTarget();
	void timeoutNavigation();

	bool reachGoal
	(bool& stay, rclcpp::Time& tin, const rclcpp::Time& tnav,
	 const CtlCommandAction::Feedback& fb, double tolp, double tola);

	bool reachGoalScan
	(bool& stay, rclcpp::Time& tin, const rclcpp::Time& tnav,
	 const CtlCommandAction::Feedback& fb, double tola);

	bool reachGoalDock();

	bool validCommand(const std::shared_ptr<const CtlCommandAction::Goal>& goal) const;
	bool validNavigation(const ib2_msgs::msg::Navigation& nav, bool first) const;

	//--------------------------------------------------------------------------
	// Action server callbacks
private:
	rclcpp_action::GoalResponse handleGoal(
		const rclcpp_action::GoalUUID& uuid,
		std::shared_ptr<const CtlCommandAction::Goal> goal);
	rclcpp_action::CancelResponse handleCancel(
		const std::shared_ptr<GoalHandleCtlCommand> goal_handle);
	void handleAccepted(
		const std::shared_ptr<GoalHandleCtlCommand> goal_handle);

	//--------------------------------------------------------------------------
	// 実装（コールバック関数）
public:
	void commandCallback(const std::shared_ptr<GoalHandleCtlCommand> goal_handle);

	void updateCallback(
		const std::shared_ptr<ib2_msgs::srv::UpdateParameter::Request> request,
		std::shared_ptr<ib2_msgs::srv::UpdateParameter::Response> response);

	void navinfoCallback(const ib2_msgs::msg::Navigation::SharedPtr nav_stamp);

	void timerCallback();

	//--------------------------------------------------------------------------
	// Action server helper methods
private:
	bool isGoalActive() const;
	bool isPreemptRequested() const;
	void publishFeedback(const CtlCommandAction::Feedback& fb);
	void setSucceeded(const CtlCommandAction::Result& r);
	void setPreempted(const CtlCommandAction::Result& r);

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** 制御目標アクションサーバ */
	rclcpp_action::Server<CtlCommandAction>::SharedPtr command_as_;

	/** 現在のゴールハンドル */
	std::shared_ptr<GoalHandleCtlCommand> current_goal_handle_;

	/** パラメータ更新サービスサーバ */
	rclcpp::Service<ib2_msgs::srv::UpdateParameter>::SharedPtr update_ss_;

	/** マーカー補正サービスクライアント */
	rclcpp::Client<ib2_msgs::srv::MarkerCorrection>::SharedPtr marker_sc_;

	/** 誘導制御ステータス出力間隔 */
	rclcpp::Duration interval_status_;

	/** フィードバック間隔 */
	rclcpp::Duration interval_feedback_;

	/** 目標到達継続時間 */
	rclcpp::Duration duration_goal_;

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
	rclcpp::Duration waitCancel_;

	/** リリース開始からAIP移動開始までの待ち時間 */
	rclcpp::Duration waitRelease_;

	/** ホーミング時VisualSLAM較正の待ち時間 */
	rclcpp::Duration waitCalibration_;

	/** ドッキング開始からスタンバイまでの待ち時間 */
	rclcpp::Duration waitDocking_;

	/** 航法値のサブスクライバ */
	rclcpp::Subscription<ib2_msgs::msg::Navigation>::SharedPtr navinfo_sub_;

	/** 誘導制御モードパブリッシャ */
	rclcpp::Publisher<ib2_msgs::msg::CtlStatus>::SharedPtr status_pub_;

	/** 力トルクのパブリッシャ */
	rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

	/** 制御プロファイルのパブリッシャ */
	rclcpp::Publisher<ib2_msgs::msg::CtlProfile>::SharedPtr profile_pub_;

	/** 航法メッセージの前回値 */
	ib2_msgs::msg::Navigation last_nav_stamp_;

	/** 機体パラメータ */
	std::unique_ptr<ib2::CtlBody> body_;

	/** 誘導制御則 */
	std::unique_ptr<ib2::PosAttController> controller_;

	/** 位置姿勢誘導プロファイル */
	std::unique_ptr<ib2::PosAttProfiler> profiler_;

	/** 検知 */
	Dtc dtc_;

	/** 誘導制御モード */
	int32_t status_;

	/** ステータス出力タイマー */
	rclcpp::TimerBase::SharedPtr timer_;

	/** CtlStatus sequence id */
	mutable uint32_t seq_status_;

	/** 航法メッセージ取得フラグ */
	bool valid_navigation_;
};

// End Of File -----------------------------------------------------------------
