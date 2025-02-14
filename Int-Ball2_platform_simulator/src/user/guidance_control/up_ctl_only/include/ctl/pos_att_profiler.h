
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include "ib2_msgs/CtlCommandAction.h"
#include "ib2_msgs/Navigation.h"
#include "ib2_msgs/CtlProfile.h"
#include "ctl/ctl_elements.h"
#include "ctl/pos_profiler.h"
#include "ctl/att_profiler.h"
#include "ctl/thrust_allocator.h"

#include <tuple>

namespace ib2
{
	// classの前方宣言
	class CtlBody;

	/**
	 * @brief 位置姿勢プロファイルクラス
	 */
	class PosAttProfiler
	{
		//----------------------------------------------------------------------
		// 列挙子
	public:
		/** 位置姿勢制御順の列挙子 */
		enum class SEQUENCE : unsigned int
		{
			POS_ATT,
			ATT_POS,
			PARALLEL
		};
		
		/** ドッキング誘導目標位置の列挙子 */
		enum class DOCKING_POS : unsigned int
		{
			AIP,
			RDP
		};

		/** ドッキング誘導目標姿勢の列挙子 */
		enum class DOCKING_ATT : unsigned int
		{
			AIA,
			RDA
		};
	
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		PosAttProfiler();

		/** rosparamによるコンストラクタ */
		explicit PosAttProfiler(const ros::NodeHandle& nh);

		/** 値によるコンストラクタ */
		PosAttProfiler
		(const PosProfiler& pos, const AttProfiler& att,
		 const ThrustAllocator& thr);

		/** デストラクタ */
		~PosAttProfiler();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		PosAttProfiler(const PosAttProfiler&);

		/** コピー代入演算子. */
		PosAttProfiler& operator=(const PosAttProfiler&);

		/** ムーブコンストラクタ. */
		PosAttProfiler(PosAttProfiler&&);

		/** ムーブ代入演算子. */
		PosAttProfiler& operator=(PosAttProfiler&&);

		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** メンバの設定
		 * @param [in] nh ノードハンドラ
		 * @retval true 設定成功
		 * @retval false 設定失敗
		 */
		bool setMember(const ros::NodeHandle& nh);
	
		/** 位置誘導プロファイルパラメータ設定
		 * @param [in] p 設定パラメータ
		 */
		bool setConfigPos(const PosProfiler& p);

		/** 姿勢誘導プロファイルパラメータ設定
		 * @param [in] p 設定パラメータ
		 */
		bool setConfigAtt(const AttProfiler& p);

		/** 推力配分パラメータ設定
		 * @param [in] thr 設定パラメータ
		 */
		bool setConfigThr(const ThrustAllocator& thr);

		/** 位置姿勢誘導プロファイル作成（現在位置姿勢で静止）
		 * @param [in] nav 航法値
		 */
		ib2_msgs::CtlProfile setProfile(const ib2_msgs::Navigation& nav);

		/** 位置姿勢誘導プロファイル作成
		 * @param [in] nav 航法値
		 * @param [in] goal 制御目標
		 */
		ib2_msgs::CtlProfile setProfile
		(const ib2_msgs::Navigation& nav, 
		 const ib2_msgs::CtlCommandGoalConstPtr& goal, const CtlBody& b);

		/** 位置姿勢停止誘導プロファイル作成
		 * @param [in] nav 航法値
		 * @param [in] b 機体質量特性
		 */
		ib2_msgs::CtlProfile stoppingProfile
		(const ib2_msgs::Navigation& nav, const CtlBody& b);

		/** ドッキング誘導プロファイル作成
		 * @param [in] nav 航法値
		 * @param [in] pos 目標位置番号
		 * @param [in] att 目標姿勢番号
		 */
		ib2_msgs::CtlProfile dockingProfile
		(const ib2_msgs::Navigation& nav,
		 const DOCKING_POS& pos, const DOCKING_ATT& att, const CtlBody& b);
	
		/** スキャンモードプロファイル作成
		 * @param [in] nav 航法値
		 * @param [in] iaxis スキャン回転軸番号
		 */
		ib2_msgs::CtlProfile scanProfile
		(const ib2_msgs::Navigation& nav, size_t iaxis, const CtlBody& b);
	
	private:
		/** 初期位置姿勢の設定
		 * @param [in] nav 航法値
		 */
		void setPose(const ib2_msgs::Navigation& nav);

		/** 並進プロファイルの設定
		 * @param [in] dr 移動量
		 * @param [in] m 機体質量[kg]
		 */
		void setProfilePos(const Eigen::Vector3d& dr, double m);

		/** 回転プロファイルの設定
		 * @param [in] dq 回転量
		 * @param [in] Is 機体質量特性行列[kgm2]
		 */
		void setProfileAtt
		(const Eigen::Quaterniond& dq, const Eigen::Matrix3d& Is);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** スキャンモードの回転軸の数の取得
		 * @return 回転軸の数
		 */
		size_t nscan() const;

		/** プロファイル終了時刻の取得
		 * @return プロファイル終了時刻
		 */
		ros::Time te() const;

		/** プロファイルメッセージの取得
		 * @return プロファイルメッセージ
		 */
		ib2_msgs::CtlProfile message() const;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** 位置姿勢誘導プロファイルに基づき基準値計算
		 * @param [in] t_stamp 現在時刻
		 */
		CtlElements posAttProfile(const ros::Time& t_stamp) const;

		/** 制御目標までの状態量計算
		 * @param [in] nav 航法値
		 * @return 制御終了までの時間[sec]
		 * @return 目標位置姿勢までの誤差
		 */
		ib2_msgs::CtlCommandFeedback statesToGoal
		(const ib2_msgs::Navigation& nav) const;

	private:
		/** 位置誘導リファレンス値計算
		 * @param [in] t プロファイル作成時刻からの経過時間[sec]
		 * @return 位置ベクトル基準値
		 * @return 速度ベクトル基準値
		 * @return 加速度ベクトル基準値
		 */
		std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
		posProfile(double t) const;

		/** 姿勢誘導リファレンス値計算
		 * @param [in] t プロファイル作成時刻からの経過時間[sec]
		 * @return 姿勢クォータニオン基準値
		 * @return 角速度ベクトル
		 */
		std::pair<Eigen::Quaterniond, Eigen::Vector3d>
		 attProfile(double t) const;

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** 目標値 */
		//    rCMD;

		/** プロファイル作成時刻 */
		ros::Time t0_;

		/** 位置誘導プロファイルパラメータ */
		PosProfiler pos_;

		/** 姿勢誘導プロファイルパラメータ */
		AttProfiler att_;

		/** 推力配分 */
		ThrustAllocator thr_;

		// 位置プロファイル用------------
		/** r0(m)初期位置*/
		Eigen::Vector3d r0_;

		/** r1(m)制御目標位置*/
		Eigen::Vector3d r1_;

		/** v0(m/s)初期速度 */
		Eigen::Vector3d v0_;

		/** dhat(-)移動方向*/
		Eigen::Vector3d dh_;

		/** xm(m)移動量*/
		double xm_;

		/** 移動目標への最大加速度[m/s2] */
		double amax_;

		/** 位置クルージング開始 */
		double xcb_;

		/** 位置クルージング終了 */
		double xce_;

		/** xtt(s)移動にかかる時間*/
		double xtt_;

		// 姿勢プロファイル用------------
		/** q0(-)初期姿勢*/
		Eigen::Quaterniond q0_;

		/** q1(-)制御後到達姿勢*/
		Eigen::Quaterniond q1_;

		/** 回転軸方向 */
		Eigen::Vector3d axis_;

		/** qm(-)回転角[rad]*/
		double qm_;

		/** 最大角加速度[rad/s2] */
		double wdmax_;

		/** 姿勢クルージング開始 */
		double qcb_;

		/** 姿勢クルージング終了 */
		double qce_;

		/** qtt(s)姿勢制御の所要時間*/
		double qtt_;

		/** 位置・姿勢順番制御 */
		SEQUENCE seq_;
	
		/** プロファイル */	
		mutable uint32_t msg_seq_;
	};
}

// End Of File -----------------------------------------------------------------

