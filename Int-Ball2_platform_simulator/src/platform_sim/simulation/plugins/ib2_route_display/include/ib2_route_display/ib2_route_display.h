
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/math/Pose3.hh>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <ib2_msgs/msg/ctl_status.hpp>
#include <ib2_msgs/msg/ctl_profile.hpp>

#include <string>

namespace ib2_route_display_plugin
{
	/**
	 * @brief Int-Ball2の移動履歴と経路を可視化するプラグイン.
	 *
	 * ECMからInt-Ball2のボディリンク姿勢を直接取得し、
	 * visualization_msgsマーカーとしてPublishする.
	 */
	class Ib2RouteDisplay : public gz::sim::System,
	                        public gz::sim::ISystemConfigure,
	                        public gz::sim::ISystemPreUpdate
	{
	public:
		Ib2RouteDisplay();
		~Ib2RouteDisplay() override;

	private:
		Ib2RouteDisplay(const Ib2RouteDisplay&) = delete;
		Ib2RouteDisplay& operator=(const Ib2RouteDisplay&) = delete;

	public:
		void Configure(const gz::sim::Entity &_entity,
		               const std::shared_ptr<const sdf::Element> &_sdf,
		               gz::sim::EntityComponentManager &_ecm,
		               gz::sim::EventManager &_eventMgr) override;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
		               gz::sim::EntityComponentManager &_ecm) override;

	private:
		void timerCallback(gz::sim::EntityComponentManager &_ecm, double sim_time);
		void controlHistoryMarkers(const geometry_msgs::msg::Pose &pose, double sim_time);
		void controlRouteMarkers(double sim_time);
		void ctlStatusCallback(const ib2_msgs::msg::CtlStatus::SharedPtr msg);
		void ctlProfileCallback(const ib2_msgs::msg::CtlProfile::SharedPtr msg);
		void getModels(gz::sim::EntityComponentManager &_ecm);

		static constexpr unsigned int HISTORY_INDEX_MIN = 1;
		static constexpr int DEFAULT_HISTORY_MAX = 50;
		static constexpr double DEFAULT_HISTORY_MIN_DISTANCE = 0.1;

		std::shared_ptr<rclcpp::Node> ros_node_;

		std::string         ib2_name_;
		gz::sim::Entity     ib2_model_{gz::sim::kNullEntity};
		gz::sim::Entity     ib2_link_{gz::sim::kNullEntity};

		bool   force_publish_history_{false};
		bool   is_ib2_moving_{false};
		bool   is_route_marker_deleted_{false};
		double history_min_distance_{DEFAULT_HISTORY_MIN_DISTANCE};

		ib2_msgs::msg::CtlProfile ctl_profile_;
		ib2_msgs::msg::CtlStatus  ctl_status_;

		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr      pub_history_;
		rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_route_;

		rclcpp::Subscription<ib2_msgs::msg::CtlStatus>::SharedPtr  sub_ctl_status_;
		rclcpp::Subscription<ib2_msgs::msg::CtlProfile>::SharedPtr sub_ctl_profile_;

		double last_ib2_stop_time_{0.0};
		double last_ctl_profile_time_{0.0};

		std::string  history_frame_id_;
		int          history_max_{DEFAULT_HISTORY_MAX};
		unsigned int last_published_history_id_{HISTORY_INDEX_MIN - 1};

		visualization_msgs::msg::Marker      msg_history_;
		visualization_msgs::msg::MarkerArray msg_route_;

		double timer_period_{0.0};
		double next_timer_time_{0.0};
	};
}  // namespace ib2_route_display_plugin
// End Of File -----------------------------------------------------------------
