
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>

#include <rclcpp/rclcpp.hpp>
#include <ib2_msgs/msg/imu.hpp>

namespace ib2_imu_sensor_plugin
{
	/**
	 * @brief Int-Ball2用IMUセンサープラグイン.
	 * リンクの角速度・加速度を読み取り、ノイズを加えてPublishする.
	 */
	class Ib2ImuSensor : public gz::sim::System,
	                     public gz::sim::ISystemConfigure,
	                     public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		Ib2ImuSensor();
		~Ib2ImuSensor() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Ib2ImuSensor(const Ib2ImuSensor&) = delete;
		Ib2ImuSensor& operator=(const Ib2ImuSensor&) = delete;
		Ib2ImuSensor(Ib2ImuSensor&&) = delete;
		Ib2ImuSensor& operator=(Ib2ImuSensor&&) = delete;

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
		void getParameter();

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		std::shared_ptr<rclcpp::Node>  ros_node_;
		rclcpp::Publisher<ib2_msgs::msg::IMU>::SharedPtr pub_;

		gz::sim::Entity                model_entity_{gz::sim::kNullEntity};
		gz::sim::Entity                link_entity_{gz::sim::kNullEntity};

		bool                           add_noise_{false};
		bool                           checks_enabled_{false};

		double noise_velocity_mean_x_{0.0};
		double noise_velocity_mean_y_{0.0};
		double noise_velocity_mean_z_{0.0};
		double noise_velocity_stddev_x_{0.0};
		double noise_velocity_stddev_y_{0.0};
		double noise_velocity_stddev_z_{0.0};

		double noise_acceleration_mean_x_{0.0};
		double noise_acceleration_mean_y_{0.0};
		double noise_acceleration_mean_z_{0.0};
		double noise_acceleration_stddev_x_{0.0};
		double noise_acceleration_stddev_y_{0.0};
		double noise_acceleration_stddev_z_{0.0};
	};
}
// End Of File -----------------------------------------------------------------
