
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>

#include <rclcpp/rclcpp.hpp>
#include <ib2_msgs/msg/navigation.hpp>

#include <cmath>
#include <cassert>
#include <chrono>

namespace issdyn_plugin
{
	/**
	 * @brief ISSの姿勢変動を模擬するプラグイン.
	 */
	class Issdyn : public gz::sim::System,
	               public gz::sim::ISystemConfigure,
	               public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Issdyn();

		/** デストラクタ. */
		~Issdyn() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Issdyn(const Issdyn&) = delete;
		Issdyn& operator=(const Issdyn&) = delete;
		Issdyn(Issdyn&&) = delete;
		Issdyn& operator=(Issdyn&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		void Configure(const gz::sim::Entity &_entity,
		               const std::shared_ptr<const sdf::Element> &_sdf,
		               gz::sim::EntityComponentManager &_ecm,
		               gz::sim::EventManager &_eventMgr) override;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
		               gz::sim::EntityComponentManager &_ecm) override;

	private:
		/** ROS Parameter Serverからパラメータ取得 */
		void getParameter();

		/** ISSの姿勢変動を設定 */
		void setIssAttitude(const gz::sim::UpdateInfo &_info,
		                    gz::sim::EntityComponentManager &_ecm);

		/** ISS航法値をパブリッシュ */
		void pubIssNav(const gz::sim::UpdateInfo &_info,
		               gz::sim::EntityComponentManager &_ecm);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノード */
		std::shared_ptr<rclcpp::Node>  ros_node_;

		/** モデルエンティティ */
		gz::sim::Entity                model_entity_{gz::sim::kNullEntity};

		/** ISSリンクエンティティ */
		gz::sim::Entity                iss_link_{gz::sim::kNullEntity};

		/** 航法値のパブリッシャ */
		rclcpp::Publisher<ib2_msgs::msg::Navigation>::SharedPtr pub_nav_;

		/** ISS 姿勢変動バイアスの傾き */
		gz::math::Vector3d             att_bias_slope_;

		/** ISS 姿勢変動正弦波のゲイン */
		gz::math::Vector3d             att_fluc_gain_;

		/** ISS姿勢変動正弦波の周期 */
		gz::math::Vector3d             att_fluc_freq_;

		/** ISS航法値のパブリッシュ周期 */
		double                         pub_cycle_{0.0};

		/** 本ノード開始フラグ */
		bool                           start_flag_{false};

		/** 本ノード開始時シミュレーション時刻 */
		double                         start_time_{0.0};

		/** 最後のパブリッシュ時刻 */
		double                         last_pub_time_{-1.0};

		/** 速度チェック有効化フラグ */
		bool                           velocity_checks_enabled_{false};
	};
}
// End Of File -----------------------------------------------------------------
