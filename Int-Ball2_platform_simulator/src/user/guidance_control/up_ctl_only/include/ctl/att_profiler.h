
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

namespace ib2
{
	/**
	 * @brief 姿勢制御プロファイルパラメータ.
	 */
	class AttProfiler final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		AttProfiler();
		
		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit AttProfiler(const ros::NodeHandle& nh);
		
		/** デストラクタ. */
		~AttProfiler();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		AttProfiler(const AttProfiler&);
		
		/** コピー代入演算子. */
		AttProfiler& operator=(const AttProfiler&);
		
		/** ムーブコンストラクタ. */
		AttProfiler(AttProfiler&&);
		
		/** ムーブ代入演算子. */
		AttProfiler& operator=(AttProfiler&&);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 姿勢プロファイル最大トルクの取得
		 * @return 姿勢プロファイル最大トルク[Nm]
		 */
		double Tmax() const;

		/** 姿勢プロファイル最大角速度の取得
		 * @return 姿勢プロファイル最大角速度[rad/s]
		 */
		double wmax() const;

		/** 角速度制御を切る閾値の取得
		 * @return 角速度制御を切る閾値[rad]
		 */
		double qthr() const;

		/** 回転角微小数の取得
		 * @return 回転角微小数[rad]
		 */
		double epsQm() const;

		/** AIAの参照
		 * @return AIAクォータニオン[rad]
		 */
		const Eigen::Quaterniond& aia() const;

		/** RDAの参照
		 * @return RDAクォータニオン[rad]
		 */
		const Eigen::Quaterniond& rda() const;
		
		/** スキャン軸の参照
		 * @return スキャン軸
		 */
		const std::vector<Eigen::Vector3d>& scanAxes() const;
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 姿勢プロファイル最大トルク[Nm] */
		double Tmax_;
		
		/** 姿勢プロファイル最大角速度[rad/s] */
		double wmax_;

		/** 角速度制御を切る閾値[rad] */
		double qthr_;
		
		/** 回転角微小数 */
		double epsQm_;
		
		/** AIA(Approach Insertion Attitude) */
		Eigen::Quaterniond aia_;
		
		/** RDA(Ready to Dock Attitude) */
		Eigen::Quaterniond rda_;
		
		/** スキャンモード回転軸 */
		std::vector<Eigen::Vector3d> scanAxes_;
	};
}

// End Of File -----------------------------------------------------------------
