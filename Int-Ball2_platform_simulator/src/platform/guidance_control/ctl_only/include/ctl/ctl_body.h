
#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

namespace ib2
{
	/**
	 * @brief 位置制御プロファイルパラメータ.
	 */
	class CtlBody final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		CtlBody();

		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit CtlBody(const ros::NodeHandle& nh);

		/** デストラクタ. */
		~CtlBody();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		CtlBody(const CtlBody&);

		/** コピー代入演算子. */
		CtlBody& operator=(const CtlBody&);

		/** ムーブコンストラクタ. */
		CtlBody(CtlBody&&);

		/** ムーブ代入演算子. */
		CtlBody& operator=(CtlBody&&);

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 機体質量の取得
		 * @return 機体質量[kg]
		 */
		double m() const;
		
		/** 質量特性行列の参照
		 * @return 質量特性行列[kgm2]の参照
		 */
		const Eigen::Matrix3d& Is() const;
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 機体質量[kg] */
		double m_;
		
		/** 質量特性行列[kgm2] */
		Eigen::Matrix3d Is_;
	};
}

// End Of File -----------------------------------------------------------------
