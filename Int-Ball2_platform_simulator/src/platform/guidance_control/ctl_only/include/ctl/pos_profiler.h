
#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

namespace ib2
{
	/**
	 * @brief 位置制御プロファイルパラメータ.
	 */
	class PosProfiler final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		PosProfiler();

		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit PosProfiler(const ros::NodeHandle& nh);

		/** デストラクタ. */
		~PosProfiler();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		PosProfiler(const PosProfiler&);

		/** コピー代入演算子. */
		PosProfiler& operator=(const PosProfiler&);

		/** ムーブコンストラクタ. */
		PosProfiler(PosProfiler&&);

		/** ムーブ代入演算子. */
		PosProfiler& operator=(PosProfiler&&);

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 位置プロファイル最大推力の取得
		 * @return 位置プロファイル最大推力[N]
		 */
		double Fmax() const;

		/** 位置プロファイル最大速度の取得
		 * @return 位置プロファイル最大速度[m/s]
		 */
		double vmax() const;

		/** 速度制御を切る閾値の取得
		 * @return 速度制御を切る閾値[m]
		 */
		double xthr() const;

		/** プロファイル作成用スラスタ能率の取得
		 * @return プロファイル作成用スラスタ能率[-]
		 */
		double eta() const;

		/** プロファイル作成用スラスタ最大能率の取得
		 * @return プロファイル作成用スラスタ最大能率[-]
		 */
		double etamax() const;

		/** 移動量微小数の取得
		 * @return 移動量微小数[m]
		 */
		double epsRm() const;
		
		/** AIPの参照
		 * @return AIP位置[m]
		 */
		const Eigen::Vector3d& aip() const;

		/** RDPの参照
		 * @return RD位置[m]
		 */
		const Eigen::Vector3d& rdp() const;
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 位置プロファイル最大推力[N] */
		double Fmax_;

		/** 位置プロファイル最大速度[m/s] */
		double vmax_;

		/** 速度制御を切る閾値[m] */
		double xthr_;

		/** プロファイル作成用スラスタ能率 */
		double eta_;

		/** プロファイル作成用スラスタ最大能率 */
		double etamax_;
		
		/** 移動量微小数 */
		double epsRm_;
		
		/** AIP(Approach Insertion Point)位置[m] */
		Eigen::Vector3d aip_;

		/** RDP(Ready to Dock Point)位置[m] */
		Eigen::Vector3d rdp_;
	};
}

// End Of File -----------------------------------------------------------------
