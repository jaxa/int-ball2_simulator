
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <ib2_msgs/msg/navigation.hpp>
#include <ib2_msgs/msg/navigation_status.hpp>
#include <ib2_msgs/msg/power_status.hpp>
#include <ib2_msgs/srv/switch_power.hpp>
#include <ib2_msgs/srv/marker_correction.hpp>
#include <ib2_msgs/action/navigation_start_up.hpp>
#include <sim_msgs/msg/attitude.hpp>
#include <sim_msgs/srv/update_parameter.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>

#include "coordinate_transform/coordinate_transform.h"

#include <string>
#include <queue>
#include <fstream>
#include <sstream>
#include <cassert>
#include <optional>

namespace nav_plugin
{
	using NavigationStartUp = ib2_msgs::action::NavigationStartUp;
	using GoalHandleNSU = rclcpp_action::ServerGoalHandle<NavigationStartUp>;

	/**
	 * @brief Robotの状態量を取得し、航法値メッセージをpublishするプラグイン.
	 */
	class Nav : public gz::sim::System,
	            public gz::sim::ISystemConfigure,
	            public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		Nav();
		~Nav() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Nav(const Nav&) = delete;
		Nav& operator=(const Nav&) = delete;
		Nav(Nav&&) = delete;
		Nav& operator=(Nav&&) = delete;

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
		int  getParameter();
		void updateParameter(
			const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> req,
			std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res);
		void markerCorrection(
			const std::shared_ptr<ib2_msgs::srv::MarkerCorrection::Request> req,
			std::shared_ptr<ib2_msgs::srv::MarkerCorrection::Response> res);
		void switchPower(
			const std::shared_ptr<ib2_msgs::srv::SwitchPower::Request> req,
			std::shared_ptr<ib2_msgs::srv::SwitchPower::Response> res);
		void switchPowerInternal(uint8_t power_status);
		void openCSVFile();
		void getModels(gz::sim::EntityComponentManager &_ecm);
		void navCallBack(const gz::sim::UpdateInfo &_info,
		                 gz::sim::EntityComponentManager &_ecm);
		void sensorFusionStatusCallBack(const gz::sim::UpdateInfo &_info);
		ib2_msgs::msg::Navigation getTrueNavigation(
			const gz::sim::UpdateInfo &_info,
			gz::sim::EntityComponentManager &_ecm);
		ib2_msgs::msg::Navigation transformWtoH(
			const ib2_msgs::msg::Navigation& nav_msgs,
			gz::sim::EntityComponentManager &_ecm);
		ib2_msgs::msg::Navigation addError(const ib2_msgs::msg::Navigation& nav_msgs);
		double controlFreqFluctuation(double sim_time);
		sim_msgs::msg::Attitude makeAttMsgFromNavMsg(const ib2_msgs::msg::Navigation& nav_msgs);
		void sumAcclAndAttRate(gz::sim::EntityComponentManager &_ecm);

		struct NavError
		{
			gz::math::Vector3d pos;
			gz::math::Vector3d vel;
			gz::math::Vector3d acc;
			gz::math::Vector3d rot;
			gz::math::Vector3d w;
		};
		NavError getNavError();

		// Action server callbacks
		rclcpp_action::GoalResponse handleGoal(
			const rclcpp_action::GoalUUID &uuid,
			std::shared_ptr<const NavigationStartUp::Goal> goal);
		rclcpp_action::CancelResponse handleCancel(
			const std::shared_ptr<GoalHandleNSU> goal_handle);
		void handleAccepted(const std::shared_ptr<GoalHandleNSU> goal_handle);

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

		rclcpp::Publisher<ib2_msgs::msg::Navigation>::SharedPtr       pub_nav_;
		rclcpp::Publisher<ib2_msgs::msg::NavigationStatus>::SharedPtr pub_sensor_fusion_status_;
		rclcpp::Publisher<sim_msgs::msg::Attitude>::SharedPtr         pub_att_;
		rclcpp::Publisher<ib2_msgs::msg::Navigation>::SharedPtr       pub_true_nav_;
		rclcpp::Publisher<sim_msgs::msg::Attitude>::SharedPtr         pub_true_att_;

		rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr         sub_status_;
		rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr       sub_time_offset_;

		rclcpp::Service<sim_msgs::srv::UpdateParameter>::SharedPtr    nav_param_server_;
		rclcpp::Service<ib2_msgs::srv::MarkerCorrection>::SharedPtr   marker_correction_server_;
		rclcpp::Service<ib2_msgs::srv::SwitchPower>::SharedPtr        switch_power_server_;

		rclcpp_action::Server<NavigationStartUp>::SharedPtr           navigation_start_up_;

		std::queue<ib2_msgs::msg::Navigation> nav_buffer_;
		double                         delay_{0.0};
		bool                           error_source_csv_{false};
		std::string                    csv_file_name_;
		std::string                    plugin_path_;
		std::ifstream                  ifs_;

		int                            accum_counter_{0};
		gz::math::Vector3d             delta_v_;
		gz::math::Vector3d             delta_angle_;

		gz::math::Vector3d             bias_p_, rand_p_;
		gz::math::Vector3d             bias_v_, rand_v_;
		gz::math::Vector3d             bias_a_, rand_a_;
		gz::math::Vector3d             bias_r_, rand_r_;
		gz::math::Vector3d             bias_w_, rand_w_;

		double                         bias_cnt_{0.0};
		double                         rand_cnt_{0.0};
		double                         gain_cnt_{0.0};
		double                         freq_cnt_{0.0};

		gazebo::CoordinateTransform    coord_transformer_;

		uint8_t                        status_{0};
		double                         tnav_offset_{0.0};
		bool                           invalid_nav_{false};
		bool                           initial_nav_on_{false};
		bool                           nav_running_{false};

		double                         next_nav_time_{0.0};
		double                         next_status_time_{0.0};
		double                         current_sim_time_{0.0};

		bool                           velocity_checks_enabled_{false};

		struct PendingAction
		{
			std::shared_ptr<GoalHandleNSU> goal_handle;
			double                         complete_time;
			uint8_t                        result_type;
		};
		std::optional<PendingAction>   pending_action_;
	};
}
// End Of File -----------------------------------------------------------------
