
#pragma once

#include <ros/ros.h>
#include <Eigen/Core>
#include "ctl/ctl_elements.h"

namespace ib2
{
	/**
	 * @brief 位置制御則パラメータ.
	 */
	class PosController final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		PosController();

		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit PosController(const ros::NodeHandle& nh);

		/** デストラクタ. */
		~PosController();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		PosController(const PosController&);

		/** コピー代入演算子. */
		PosController& operator=(const PosController&);

		/** ムーブコンストラクタ. */
		PosController(PosController&&);

		/** ムーブ代入演算子. */
		PosController& operator=(PosController&&);

		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** 積分量のクリア */
		void flash();

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 比例ゲインの取得
		 * @return 比例ゲイン
		 */
		double kp() const;

		/** 積分ゲインの取得
		 * @return 積分ゲイン
		 */
		double ki() const;

		/** 微分ゲインの取得
		 * @return 微分ゲイン
		 */
		double kd() const;

		/** 積分制御最大推力の取得
		 * @return 積分制御最大推力[N]
		 */
		double Fmax() const;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** 力コマンド計算
		 * @param [in] t 航法タイムタグ(-)
		 * @param [in] r 位置ベクトル(m)
		 * @param [in] v 速度ベクトル(m/s)
		 * @param [in] q 姿勢クォータニオン(-)
		 * @param [in] p 制御プロファイル基準値
		 * @param [in] m 機体質量[kg]
		 * @retval     力コマンド
		 */
		Eigen::Vector3d forceCommand
		(const ros::Time& t, const Eigen::Vector3d &r, const Eigen::Vector3d &v,
		 const Eigen::Quaterniond& q, const CtlElements &p, double m);

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 比例ゲイン */
		double kp_;

		/** 積分ゲイン */
		double ki_;

		/** 微分ゲイン */
		double kd_;

		/** 積分制御最大推力 */
		double Fmax_;

		/** 積分項の位置積分値 */
		Eigen::Vector3d s_;

		/** 積分値s_のタイムタグ */
		ros::Time ts_;//
	};
}

// End Of File -----------------------------------------------------------------
