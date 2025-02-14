
#pragma once

#include <numeric>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include "ctl/thrust_allocator.h"

/**
* @brief ファン選択ノードクラス
*/
class Fsm
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
private:
	/** デフォルトコンストラクタ */
	Fsm() = delete;

public:
	/** コンストラクタ */
	explicit Fsm(const ros::NodeHandle& nh);

	/** デストラクタ */
	~Fsm();
	
	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	Fsm(const Fsm&) = delete;

	/** コピー代入演算子. */
	Fsm& operator=(const Fsm&) = delete;

	/** ムーブコンストラクタ. */
	Fsm(Fsm&&) = delete;

	/** ムーブ代入演算子. */
	Fsm& operator=(Fsm&&) = delete;

	//----------------------------------------------------------------------
	// 操作(Setter)
public:
	/** メンバ設定
	 * @param [in] thr 推力配分パラメータ
	 * @retval true 設定成功
	 * @retval false 設定失敗
	 */
	bool setMember(const ib2::ThrustAllocator& thr);

	//----------------------------------------------------------------------
	// 実装
public:
	/** 制御推力トルクのサブスクライバのコールバック関数
	 * @param [in] wrench 力トルク
	 */
	//void subscribeCommand(const geometry_msgs::WrenchStamped& wrench) const;    // Modification for platform packages
	void wrenchCallback(const geometry_msgs::WrenchStamped& wrench) const;        // Modification for platform packages

private:
	/** 各ファンの駆動デューティ比のPublish
	 * @param [in] pwm 駆動デューティ比
	 */
	void publishDuty(const Eigen::VectorXd& pwm) const;

	/** ファン推進力飽和時処理
	 * @param [in, out] pwm 駆動デューティ比
	 */
	void saturation(Eigen::VectorXd& pwm) const;

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ROSノードハンドラ */
	ros::NodeHandle nh_;

	/** デューティのパブリッシャ */
	ros::Publisher  pub_duty_;

	/** 推力配分 */
	ib2::ThrustAllocator thr_;

	/** 各ファンのPWM最大値 */
    Eigen::VectorXd pwm_max_;

	/** 各ファンの回転数と推力の関係係数 */
    Eigen::VectorXd fan_kj_;

	/** 力トルクに影響を与えないファン出力群 */
    Eigen::VectorXd fj0_;
    
	/** ファンの数 */
    int nfan_;

	/** 飽和ファン数 */
    int nsaturation_;

	/** 力トルクのサブスクライバ */
    ros::Subscriber wrench_sub_; // Modification for platform packages

};
// End Of File -----------------------------------------------------------------
