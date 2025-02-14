
#pragma once

#include <ros/ros.h>
#include <Eigen/Core>

namespace ib2
{
	/**
	 * @brief 推力配分.
	 *
	 * 推力配分行列を用いて、並進力・トルクを各スラスタの推力に配分する
	 */
	class ThrustAllocator final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		ThrustAllocator();
		
		/** rosparamによるコンストラクタ
		 * @param [in] nh ノードハンドラ
		 */
		explicit ThrustAllocator(const ros::NodeHandle& nh);

		/** 値によるコンストラクタ.
		 * @param [in] Fmax 各ファンの最大推力
		 * @param [in] Wp 正方向の配列行列
		 * @param [in] Wm 負方向の配列行列
		 */
		ThrustAllocator
		(double Fmax, const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm, const Eigen::VectorXd& fan_null);
		
		/** デストラクタ. */
		~ThrustAllocator();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		ThrustAllocator(const ThrustAllocator&);
		
		/** コピー代入演算子. */
		ThrustAllocator& operator=(const ThrustAllocator&);
		
		/** ムーブコンストラクタ. */
		ThrustAllocator(ThrustAllocator&&);
		
		/** ムーブ代入演算子. */
		ThrustAllocator& operator=(ThrustAllocator&&);
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** メンバ変数の設定.
		 * @param [in] Fmax 各ファンの最大推力
		 * @param [in] Wp 正方向の配列行列
		 * @param [in] Wm 負方向の配列行列
		 * @return 加算結果への参照
		 */
		ThrustAllocator& set
		(double Fmax, const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** ファンの数の取得
		 * @return ファンの数
		 */
		int nfan() const;

		/** ファン推力最大値の取得
		 * @return ファン推力最大値[N]
		 */
		double Fmax() const;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 推力配分
		 * @param [in] F 推力[N]
		 * @param [in] T トルク[Nm]
		 * @return 各スラスタの推力
		 */
		Eigen::VectorXd allocate
		(const Eigen::Vector3d& F, const Eigen::Vector3d& T) const;

		/** 推力配分
		 * @param [in] y 推力・トルク[N]
		 * @return 各スラスタの推力
		 */
		Eigen::VectorXd allocate(const Eigen::VectorXd& y) const;

		
		/** 実効最大推力の計算
		 * @param [in] Fmax 最大推力基準値[N]
		 * @param [in] dir 移動方向（機体座標系）
		 * @param [in] eta スラスタ最大推力レート
		 * @return 実効最大推力[N]
		 */
		double effectiveFmax
		(double Fmax, const Eigen::Vector3d& dir, double eta) const;
		
		/** 実効最大トルクの計算
		 * @param [in] Tmax 最大トルク基準値[Nm]
		 * @param [in] axis 回転軸（機体座標系）
		 * @param [in] eta スラスタ最大推力レート
		 * @return 実効最大推力[Nm]
		 */
		double effectiveTmax
		(double Tmax, const Eigen::Vector3d& axis, double eta) const;

		/** 実効最大推力・トルクの計算
		 * @param [in] Fmax 最大推力基準値[N]
		 * @param [in] dir 移動方向（機体座標系）
		 * @param [in] Tmax 最大トルク基準値[Nm]
		 * @param [in] axis 回転軸（機体座標系）
		 * @param [in] eta スラスタ最大推力レート
		 * @return 実効最大推力[N]
		 * @return 実効最大推力[Nm]
		 */
		std::pair<double,double> effectiveFTmax
		(double Fmax, const Eigen::Vector3d& dir, 
		 double Tmax, const Eigen::Vector3d& axis, double eta) const;

	private:
		/** 実効レートの計算
		 * @param [in] y 最大推力トルク基準値
		 * @param [in] eta スラスタ最大推力レート
		 * @return 実効レート
		 */
		double effective(const Eigen::VectorXd& y, double eta) const;

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** ファンの数 */
		int nfan_;

		/** 各ファンの最大推力 */
		double Fmax_;
		
		/** 正の配分行列 */
		Eigen::MatrixXd Wp_;
		
		/** 負の配分行列 */
		Eigen::MatrixXd Wm_;

		/** 力トルクに影響を与えないファン出力比率 */
		Eigen::VectorXd fan_null_;	
	};
}

// End Of File -----------------------------------------------------------------
