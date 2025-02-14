
#pragma once

#include "ctl/ctl_elements.h"

#include <ros/ros.h>
#include <Eigen/Core>

namespace ib2
{
	/**
	 * @brief 姿勢制御則パラメータ.
	 */
	class AttController final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		AttController();
		
		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit AttController(const ros::NodeHandle& nh);
		
		/** デストラクタ. */
		~AttController();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		AttController(const AttController&);
		
		/** コピー代入演算子. */
		AttController& operator=(const AttController&);
		
		/** ムーブコンストラクタ. */
		AttController(AttController&&);
		
		/** ムーブ代入演算子. */
		AttController& operator=(AttController&&);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 比例ゲインの取得
		 * @return 比例ゲイン
		 */
		double kp() const;

		/** 微分ゲインの取得
		 * @return 微分ゲイン
		 */
		double kd() const;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** トルクコマンド計算
		 * @param [in] q 姿勢クォータニオン(-)
		 * @param [in] w 角速度ベクトル(rad/s)
		 * @param [in] p 制御プロファイル基準値
		 * @param [in] Is 機体質量特性行列[kgm2]
		 * @retval    トルクコマンド
		 */
		Eigen::Vector3d torqueCommand
		(const Eigen::Quaterniond& q, const Eigen::Vector3d& w,
		 const CtlElements &p, const Eigen::Matrix3d& Is) const;

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 比例ゲイン */
		double kp_;

		/** 微分ゲイン */
		double kd_;
	};
}

// End Of File -----------------------------------------------------------------
