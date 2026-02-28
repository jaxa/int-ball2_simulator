
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ib2_msgs/msg/power_status.hpp>
#include <ib2_msgs/srv/switch_power.hpp>
#include <sim_msgs/srv/update_parameter.hpp>

#include "coordinate_transform/coordinate_transform.h"

#include <string>
#include <vector>
#include <cassert>

namespace mag_plugin
{
	/**
	 * @brief ドッキングステーションの磁力による吸引力やトルクを模擬するプラグイン.
	 */
	class Mag : public gz::sim::System,
	            public gz::sim::ISystemConfigure,
	            public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		Mag();
		~Mag() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Mag(const Mag&) = delete;
		Mag& operator=(const Mag&) = delete;
		Mag(Mag&&) = delete;
		Mag& operator=(Mag&&) = delete;

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
		void updateParameter(
			const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> req,
			std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res);
		void logParameter();
		void getModels(gz::sim::EntityComponentManager &_ecm);
		void magCallBack(const gz::sim::UpdateInfo &_info,
		                 gz::sim::EntityComponentManager &_ecm);
		void getDsPose(gz::sim::EntityComponentManager &_ecm,
		               gz::math::Vector3d& r_ds, gz::math::Quaterniond& q_ds);
		void getForce(
			const gz::math::Vector3d& r_ds,    const gz::math::Quaterniond& q_ds,
			      gz::math::Vector3d& fmag_ds,       gz::math::Vector3d&    fmag_bd);
		void getTorque(const gz::math::Vector3d& fmag_bd, gz::math::Vector3d& tmag_bd);
		void addForceAndTorque(gz::sim::EntityComponentManager &_ecm,
		                       const gz::math::Vector3d& fmag_bd, const gz::math::Vector3d& tmag_bd);
		void pubForceAndTorque(const gz::sim::UpdateInfo &_info,
		                       const gz::math::Vector3d& fmag_ds, const gz::math::Vector3d& tmag_bd);
		void switchPower(
			const std::shared_ptr<ib2_msgs::srv::SwitchPower::Request> req,
			std::shared_ptr<ib2_msgs::srv::SwitchPower::Response> res);
		bool U(double x);
		double H(double dist);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		std::shared_ptr<rclcpp::Node>  ros_node_;

		std::string                    iss_name_;
		std::string                    ib2_name_;
		gz::sim::Entity                iss_model_{gz::sim::kNullEntity};
		gz::sim::Entity                ib2_model_{gz::sim::kNullEntity};
		gz::sim::Entity                iss_link_{gz::sim::kNullEntity};
		gz::sim::Entity                ib2_link_{gz::sim::kNullEntity};

		gz::math::Vector3d             jpm_pos_;
		gz::math::Vector3d             jpm_att_;
		gz::math::Vector3d             ds_pos_;
		gz::math::Vector3d             ds_att_;

		rclcpp::Service<sim_msgs::srv::UpdateParameter>::SharedPtr mag_param_server_;
		rclcpp::Service<ib2_msgs::srv::SwitchPower>::SharedPtr switch_power_server_;
		rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_mag_;
		rclcpp::Publisher<ib2_msgs::msg::PowerStatus>::SharedPtr pub_power_status_;

		gz::math::Vector3d             rrif_;
		gz::math::Vector3d             rdif_;
		gz::math::Vector3d             cg_;

		double                         d_threshold_{0.0};
		double                         afar_{0.0}, bfar_{0.0}, cfar_{0.0};
		double                         aprox_{0.0}, bprox_{0.0}, cprox_{0.0};
		double                         stddev_{0.0};
		double                         cycle_{0.0};

		gazebo::CoordinateTransform    coord_transformer_;
		ib2_msgs::msg::PowerStatus     power_status_;

		int                            pub_cnt_{0};
	};
}
// End Of File -----------------------------------------------------------------
