
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include "ib2_msgs/Navigation.h"
#include <cmath>
#include <cassert>

namespace gazebo
{
	/**
	 * @brief ISSの姿勢変動を模擬するプラグイン.
	 */
	class Issdyn : public ModelPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Issdyn();

		/** デストラクタ. */
		~Issdyn();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Issdyn(const Issdyn&) = delete;

		/** コピー代入演算子. */
		Issdyn& operator=(const Issdyn&) = delete;

		/** ムーブコンストラクタ. */
		Issdyn(Issdyn&&) = delete;

		/** ムーブ代入演算子. */
		Issdyn& operator=(Issdyn&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** プラグインのロード
		 * @param [in, out] _model モデルへのポインタ
		 * @param [in, out] _sdf SDF要素へのポインタ
		 */
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

	private:
		/** ROS Parameter Serverからパラメータ取得
		 */
		void getParameter();

		/** ISSの姿勢変動を設定
		 */
		void setIssAttitude();

		/** ROS Timerのコールバック関数
		 * @param [in, out] event TimerEvent構造体への参照
		 */
		void pubIssNav(const ros::TimerEvent& event);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                nh_;

		/** モデルへのポインタ */
		physics::ModelPtr              model_;

		/** ISSモデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>  link_iss_body_;

		/** 航法値のパブリッシャ */
		ros::Publisher                 pub_nav_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr           update_;

		/** ISS 姿勢変動バイアスの傾き */
		ignition::math::Vector3d       att_bias_slope_;

		/** ISS 姿勢変動正弦波のゲイン */
		ignition::math::Vector3d       att_fluc_gain_;

		/** ISS姿勢変動正弦波の周期 */
		ignition::math::Vector3d       att_fluc_freq_;

		/** ROSタイマ */
		ros::Timer                     pub_timer_;

		/** ISS航法値のパブリッシュ周期 */
		double                         pub_cycle_;
	};
}
// End Of File -----------------------------------------------------------------

