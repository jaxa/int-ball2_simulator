
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <string>
#include <cassert>

namespace hill_plugin
{
	/**
	 * @brief ISSの軌道レート、重心位置を取得し、HILL方程式による相対加速度を印加するプラグイン.
	 */
	class Hill : public gz::sim::System,
	             public gz::sim::ISystemConfigure,
	             public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Hill();

		/** デストラクタ. */
		~Hill() override;

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
		/** プラグインの初期設定 (gz-sim ISystemConfigure)
		 * @param [in] _entity ワールドエンティティ
		 * @param [in] _sdf SDF要素へのポインタ
		 * @param [in, out] _ecm Entity-Component Manager
		 * @param [in, out] _eventMgr Event Manager
		 */
		void Configure(const gz::sim::Entity &_entity,
		               const std::shared_ptr<const sdf::Element> &_sdf,
		               gz::sim::EntityComponentManager &_ecm,
		               gz::sim::EventManager &_eventMgr) override;

		/** 物理ステップ前の更新 (gz-sim ISystemPreUpdate)
		 * @param [in] _info シミュレーション更新情報
		 * @param [in, out] _ecm Entity-Component Manager
		 */
		void PreUpdate(const gz::sim::UpdateInfo &_info,
		               gz::sim::EntityComponentManager &_ecm) override;

	private:
		/** ROS Parameter Serverからパラメータ取得
		 */
		void getParameter();

		/** ISS/IB2モデルを取得
		 */
		void getModels(gz::sim::EntityComponentManager &_ecm);

		/** 相対加速度(Hill方程式)のGazeboへの設定
		 */
		void addHillForce(const gz::sim::UpdateInfo &_info,
		                  gz::sim::EntityComponentManager &_ecm);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノード */
		std::shared_ptr<rclcpp::Node>  ros_node_;

		/** 相対加速度(力)のパブリッシャ */
		rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_hill_force_;

		/** 相対加速度(力)のパブリッシュ周期 */
		double                         pub_cycle_{0.0};

		/** ISSモデル名 */
		std::string                    iss_name_;

		/** IB2モデル名 */
		std::string                    ib2_name_;

		/** ISSモデルエンティティ */
		gz::sim::Entity                iss_model_{gz::sim::kNullEntity};

		/** IB2モデルエンティティ */
		gz::sim::Entity                ib2_model_{gz::sim::kNullEntity};

		/** ISSリンクエンティティ */
		gz::sim::Entity                iss_link_{gz::sim::kNullEntity};

		/** IB2リンクエンティティ */
		gz::sim::Entity                ib2_link_{gz::sim::kNullEntity};

		/** ISS軌道レート[rad/s] */
		double                         iss_w_{0.0};

		/** 相対加速度(力)[N] */
		gz::math::Vector3d             hill_force_;

		/** 速度チェック有効化フラグ */
		bool                           velocity_checks_enabled_{false};

		/** Publishカウンタ */
		int                            pub_cnt_{0};
	};
}
// End Of File -----------------------------------------------------------------
