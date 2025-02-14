
#pragma once

#include "ib2_msgs/CtlStatus.h"

#include <ros/ros.h>
#include <Eigen/Dense>

/**
* @brief 制御対象状態量を管理する
*/
class CtlElements
{
	//--------------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** デフォルトコンストラクタ */
	CtlElements();

	/** 値によるコンストラクタ
	 * @param [in] t ROS時刻
	 * @param [in] r 位置ベクトル
	 * @param [in] v 速度ベクトル
	 * @param [in] a 加速度ベクトル
	 * @param [in] q 姿勢クォータニオン
	 * @param [in] w 角速度ベクトル
	 */
	CtlElements(const ros::Time& t, 
				const Eigen::Vector3d& r, const Eigen::Vector3d& v,
				const Eigen::Vector3d& a,
				const Eigen::Quaterniond& q, const Eigen::Vector3d& w);

	/** デストラクタ */
	~CtlElements();

	//--------------------------------------------------------------------------
	// コピー/ムーブ
public:
	/** コピーコンストラクタ. */
	CtlElements(const CtlElements&);

	/** コピー代入演算子. */
	CtlElements& operator=(const CtlElements&);

	/** ムーブコンストラクタ. */
	CtlElements(CtlElements&&);

	/** ムーブ代入演算子. */
	CtlElements& operator=(CtlElements&&);

	//--------------------------------------------------------------------------
	// 属性(Getter)
public:
	/** ROS時刻の参照
	 * @return 位置ベクトルの参照
	 */
	const ros::Time& t() const;

	/** 位置ベクトルの参照
	 * @return 位置ベクトルの参照
	 */
	const Eigen::Vector3d& r() const;

	/** 速度ベクトルの参照
	 * @return 位置ベクトルの参照
	 */
	const Eigen::Vector3d& v() const;

	/** 加速度ベクトルの参照
	 * @return 位置ベクトルの参照
	 */
	const Eigen::Vector3d& a() const;

	/** 姿勢クォータニオンの参照
	 * @return 位置ベクトルの参照
	 */
	const Eigen::Quaterniond& q() const;

	/** 角速度ベクトルの参照
	 * @return 角速度ベクトルの参照
	 */
	const Eigen::Vector3d& w() const;

	/** 誘導制御ステータスメッセージ形式での取得
	 * @param [in] status 誘導制御ステータス
	 * @return 誘導制御ステータスメッセージ
	 */
	ib2_msgs::CtlStatus status(int32_t status) const;

	//--------------------------------------------------------------------------
	// メンバ変数
private:
	/** 時刻 */
	ros::Time t_;

	/** 位置ベクトル */
	Eigen::Vector3d r_;

	/** 速度ベクトル */
	Eigen::Vector3d v_;

	/** 加速度ベクトル */
	Eigen::Vector3d a_;

	/** 姿勢クォータニオン */
	Eigen::Quaterniond q_;

	/** 角速度ベクトル */
	Eigen::Vector3d w_;
};

// End Of File -----------------------------------------------------------------
