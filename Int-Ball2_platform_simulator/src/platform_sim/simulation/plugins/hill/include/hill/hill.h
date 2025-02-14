
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include <cassert>

namespace gazebo
{
	/**
	 * @brief ISSの軌道レート、重心位置を取得し、HILL方程式による相対加速度を印加するプラグイン.
	 */
	class Hill : public WorldPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Hill();

		/** デストラクタ. */
		~Hill();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Hill(const Hill&) = delete;

		/** コピー代入演算子. */
		Hill& operator=(const Hill&) = delete;

		/** ムーブコンストラクタ. */
		Hill(Hill&&) = delete;

		/** ムーブ代入演算子. */
		Hill& operator=(Hill&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** プラグインのロード
		 * @param [in, out] _world Worldへのポインタ
		 * @param [in, out] _sdf SDF要素へのポインタ
		 */
		virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

	private:
		/** ROS Parameter Serverからパラメータ取得
		 */
		void getParameter();

		/** ISS/IB2モデルを取得
		 */
		void getModels();

		/** 相対加速度(Hill方程式)のGazeboへの設定（Gazeboのコールバック関数）
		 */
		void addHillForce();

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                nh_;

		/** Worldポインタ */
		physics::WorldPtr              world_;

		/** 相対加速度(力)のパブリッシャ */
		ros::Publisher                 pub_hill_force_;

		/** 相対加速度(力)のパブリッシュ周期 */
		double                         pub_cycle_;

		/** ISSモデル名 */
		std::string                    iss_name_;

		/** IB2モデル名 */
		std::string                    ib2_name_;

		/** ISSモデルへのポインタ */
		physics::ModelPtr              iss_model_;

		/** ロボットモデルへのポインタ */
		physics::ModelPtr              ib2_model_;

		/** ISSモデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>  iss_link_;

		/** IB2モデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>  ib2_link_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr           update_;

		/** ISS軌道レート[rad/s] */
		double                         iss_w_;

		/** 相対加速度(力)[N] */
		ignition::math::Vector3d       hill_force_;
	};
}
// End Of File -----------------------------------------------------------------

