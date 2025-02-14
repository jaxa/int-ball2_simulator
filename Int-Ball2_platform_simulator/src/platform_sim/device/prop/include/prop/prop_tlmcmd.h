
#pragma once
#include <ros/ros.h>
#include "prop/prop_common.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include "ib2_msgs/FanStatus.h"
#include "ib2_msgs/SwitchPower.h"
#include "ib2_msgs/PowerStatus.h"

#define TOPIC_CTL_DUTY               "/ctl/duty"
#define TOPIC_PROP_STATUS            "/prop/status"
#define SERVICE_SWITCH_POWER         "/prop/switch_power"
#define DUTY_MIN                     0.F			// 最小ファン駆動デューティ比[-]
#define DUTY_MAX                     1.F			// 最大ファン駆動デューティ比[-]

/**
* @brief 推進機能ノード　テレメトリ・コマンドクラス
*/
class PropTlmCmd
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	PropTlmCmd();

	/** デストラクタ */
	~PropTlmCmd();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	PropTlmCmd(const PropTlmCmd&)            = delete;

	/** コピー代入演算子. */
	PropTlmCmd& operator=(const PropTlmCmd&) = delete;

	/** ムーブコンストラクタ. */
	PropTlmCmd(PropTlmCmd&&)                 = delete;

	/** ムーブ代入演算子. */
	PropTlmCmd& operator=(PropTlmCmd&&)      = delete;
    
	//----------------------------------------------------------------------
	// 操作(Setter)
private:

	//----------------------------------------------------------------------
	// 実装
public:
	/** テレメトリ・コマンド機能初期化
	 * @param [in]      nh           ROSノードハンドラ
	 * @param [in]      fanNum       ファン数
	 */
	void initialize(const ros::NodeHandle& nh, const int32_t& fan_num);

	/** ファン駆動状態をパブリッシュ
	 * @param [in]      ev           ROS Timer Event
	 */
	void pubFanStatus(const ros::TimerEvent& ev);

	/** ファン駆動状態をパブリッシュ(異常時)
	 * @param [in]      ev           ROS Timer Event
	 */
	void pubErrorFanStatus(const ros::TimerEvent& ev);

	/** ファン駆動デューティ比のgetter
	 * @return                       ファン駆動デューティ比
	 */
	std::vector<float> getFanDuty();

	/** テレメトリ・コマンド機能停止 */
	void shutdown();

	//--------------------------------------------------------------------------
	// 実装
private:
	/** ファン駆動デューティ比をサブスクライブ
	 * @param [in]      msg           ファン駆動デューティ比
	 */
	void subFanDuty(const std_msgs::Float64MultiArray& msg);

	/** ファン駆動状態初期化
	 * @param [in]      status        推進機能起動/停止ステータス
	 */
	void initFanStatus(const uint8_t& status);

	/** ファン駆動状態メッセージ生成 */
	void generateFanStatus();

	/** 推進機能起動/停止
	 * @param [in]                 req              推進機能起動/停止サービスリクエスト
	 * @param [in]                 res              現在の推進機能ステータス
	 * @retval                     true             設定成功
	 * @retval                     false            設定失敗
	 */
	bool switchPower(
		ib2_msgs::SwitchPower::Request&  req,
		ib2_msgs::SwitchPower::Response& res
	);

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ROSノードハンドラ */
	ros::NodeHandle                  nh_;

	/** ファンデューティ比(誘導制御ノード)　サブスクライバ */
	ros::Subscriber                  sub_fan_duty_;

	/** 推進機能ノードのファン駆動状態パブリッシャ */
	ros::Publisher                   pub_fan_status_;

	/** 推進機能起動/停止サービスサーバ */
	ros::ServiceServer               switch_power_server_;

	/** ファン数 */
	int32_t                          fan_num_;

	/** ファン駆動デューティ比メッセージ */
	std_msgs::Float64MultiArray      fan_duty_;

	/** ファン駆動状態メッセージ */
	ib2_msgs::FanStatus              fan_status_;
};

// End Of File -----------------------------------------------------------------
