
#include "ib2_route_display/ib2_route_display.h"

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/plugin/Register.hh>

#include <ib2_msgs/msg/ctl_status_type.hpp>

#include <cmath>
#include <chrono>

namespace ib2_route_display_plugin
{

namespace
{
	constexpr unsigned int ROUTE_LINE     = 0;
	constexpr unsigned int ROUTE_START    = 1;
	constexpr unsigned int ROUTE_END      = 2;
	constexpr unsigned int ROUTE_LINE_ID  = 1;
	constexpr unsigned int ROUTE_START_ID = 2;
	constexpr unsigned int ROUTE_END_ID   = 3;
}

Ib2RouteDisplay::Ib2RouteDisplay() = default;
Ib2RouteDisplay::~Ib2RouteDisplay() = default;

void Ib2RouteDisplay::Configure(
	const gz::sim::Entity &/*_entity*/,
	const std::shared_ptr<const sdf::Element> &_sdf,
	gz::sim::EntityComponentManager &/*_ecm*/,
	gz::sim::EventManager &/*_eventMgr*/)
{
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	ros_node_ = std::make_shared<rclcpp::Node>("ib2_route_display");

	// Get Int-Ball2 model name from SDF
	auto sdfClone = _sdf->Clone();
	ib2_name_ = sdfClone->Get<std::string>("ib2_name", "ib2").first;

	// Helper lambdas
	auto get_double_param = [this](const std::string &name, double default_val) -> double {
		if (!ros_node_->has_parameter(name))
		{
			ros_node_->declare_parameter<double>(name, default_val);
		}
		double val = default_val;
		ros_node_->get_parameter(name, val);
		return val;
	};

	auto get_int_param = [this](const std::string &name, int default_val) -> int {
		if (!ros_node_->has_parameter(name))
		{
			ros_node_->declare_parameter<int>(name, default_val);
		}
		int val = default_val;
		ros_node_->get_parameter(name, val);
		return val;
	};

	auto get_string_param = [this](const std::string &name, const std::string &default_val) -> std::string {
		if (!ros_node_->has_parameter(name))
		{
			ros_node_->declare_parameter<std::string>(name, default_val);
		}
		std::string val = default_val;
		ros_node_->get_parameter(name, val);
		return val;
	};

	// Timer rate
	double timer_rate = 10.0;
	timer_rate = get_double_param("ib2_route_display.rate", timer_rate);
	timer_period_ = 1.0 / timer_rate;

	// Subscribers
	sub_ctl_status_ = ros_node_->create_subscription<ib2_msgs::msg::CtlStatus>(
		"/ctl/status", 1,
		[this](const ib2_msgs::msg::CtlStatus::SharedPtr msg) {
			ctlStatusCallback(msg);
		});

	sub_ctl_profile_ = ros_node_->create_subscription<ib2_msgs::msg::CtlProfile>(
		"/ctl/profile", 1,
		[this](const ib2_msgs::msg::CtlProfile::SharedPtr msg) {
			ctlProfileCallback(msg);
		});

	// Publishers
	pub_history_ = ros_node_->create_publisher<visualization_msgs::msg::Marker>(
		"/ib2_route_display/history", 1);
	pub_route_ = ros_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
		"/ib2_route_display/target", 1);

	/*
	 * History marker setup
	 */
	history_max_ = get_int_param("ib2_route_display.history.count_max", DEFAULT_HISTORY_MAX);
	history_min_distance_ = get_double_param("ib2_route_display.history.min_distance", DEFAULT_HISTORY_MIN_DISTANCE);
	history_frame_id_ = get_string_param("ib2_route_display.history.frame_id", "");
	if (history_frame_id_.empty())
	{
		RCLCPP_ERROR(ros_node_->get_logger(),
			"Cannot get ib2_route_display.history.frame_id");
		return;
	}

	msg_history_.header.frame_id = history_frame_id_;
	msg_history_.ns = "/ib2_route_display/history";
	msg_history_.type = visualization_msgs::msg::Marker::SPHERE;
	msg_history_.pose.orientation.x = 0;
	msg_history_.pose.orientation.y = 0;
	msg_history_.pose.orientation.z = 0;
	msg_history_.pose.orientation.w = 1.0;
	msg_history_.frame_locked = true;

	double history_scale = get_double_param("ib2_route_display.history.scale", 0.2);
	msg_history_.color.a = static_cast<float>(get_double_param("ib2_route_display.history.color.a", 0.5));
	msg_history_.color.r = static_cast<float>(get_double_param("ib2_route_display.history.color.r", 0.0));
	msg_history_.color.g = static_cast<float>(get_double_param("ib2_route_display.history.color.g", 1.0));
	msg_history_.color.b = static_cast<float>(get_double_param("ib2_route_display.history.color.b", 0.0));
	msg_history_.scale.x = history_scale;
	msg_history_.scale.y = history_scale;
	msg_history_.scale.z = history_scale;

	/*
	 * Route marker setup
	 */
	msg_route_.markers.resize(3);
	std::string route_frame_id = get_string_param("ib2_route_display.route.frame_id", "");
	if (route_frame_id.empty())
	{
		RCLCPP_ERROR(ros_node_->get_logger(),
			"Cannot get ib2_route_display.route.frame_id");
		return;
	}

	// Route LINE
	msg_route_.markers[ROUTE_LINE].header.frame_id = route_frame_id;
	msg_route_.markers[ROUTE_LINE].ns = "/ib2_route_display/route";
	msg_route_.markers[ROUTE_LINE].id = ROUTE_LINE_ID;
	msg_route_.markers[ROUTE_LINE].type = visualization_msgs::msg::Marker::LINE_STRIP;
	msg_route_.markers[ROUTE_LINE].pose.orientation.w = 1.0;
	msg_route_.markers[ROUTE_LINE].color.a = static_cast<float>(get_double_param("ib2_route_display.route.line.color.a", 1.0));
	msg_route_.markers[ROUTE_LINE].color.r = static_cast<float>(get_double_param("ib2_route_display.route.line.color.r", 1.0));
	msg_route_.markers[ROUTE_LINE].color.g = static_cast<float>(get_double_param("ib2_route_display.route.line.color.g", 0.0));
	msg_route_.markers[ROUTE_LINE].color.b = static_cast<float>(get_double_param("ib2_route_display.route.line.color.b", 1.0));
	msg_route_.markers[ROUTE_LINE].scale.x = get_double_param("ib2_route_display.route.line.scale", 0.02);

	// Route START
	msg_route_.markers[ROUTE_START].header.frame_id = route_frame_id;
	msg_route_.markers[ROUTE_START].ns = "/ib2_route_display/route";
	msg_route_.markers[ROUTE_START].id = ROUTE_START_ID;
	msg_route_.markers[ROUTE_START].type = visualization_msgs::msg::Marker::SPHERE;
	msg_route_.markers[ROUTE_START].color.a = static_cast<float>(get_double_param("ib2_route_display.route.start.color.a", 1.0));
	msg_route_.markers[ROUTE_START].color.r = static_cast<float>(get_double_param("ib2_route_display.route.start.color.r", 0.0));
	msg_route_.markers[ROUTE_START].color.g = static_cast<float>(get_double_param("ib2_route_display.route.start.color.g", 0.0));
	msg_route_.markers[ROUTE_START].color.b = static_cast<float>(get_double_param("ib2_route_display.route.start.color.b", 1.0));
	double route_start_scale = get_double_param("ib2_route_display.route.start.scale", 0.1);
	msg_route_.markers[ROUTE_START].scale.x = route_start_scale;
	msg_route_.markers[ROUTE_START].scale.y = route_start_scale;
	msg_route_.markers[ROUTE_START].scale.z = route_start_scale;

	// Route END
	msg_route_.markers[ROUTE_END].header.frame_id = route_frame_id;
	msg_route_.markers[ROUTE_END].ns = "/ib2_route_display/route";
	msg_route_.markers[ROUTE_END].id = ROUTE_END_ID;
	msg_route_.markers[ROUTE_END].type = visualization_msgs::msg::Marker::SPHERE;
	msg_route_.markers[ROUTE_END].color.a = static_cast<float>(get_double_param("ib2_route_display.route.end.color.a", 1.0));
	msg_route_.markers[ROUTE_END].color.r = static_cast<float>(get_double_param("ib2_route_display.route.end.color.r", 1.0));
	msg_route_.markers[ROUTE_END].color.g = static_cast<float>(get_double_param("ib2_route_display.route.end.color.g", 0.0));
	msg_route_.markers[ROUTE_END].color.b = static_cast<float>(get_double_param("ib2_route_display.route.end.color.b", 0.0));
	double route_end_scale = get_double_param("ib2_route_display.route.end.scale", 0.1);
	msg_route_.markers[ROUTE_END].scale.x = route_end_scale;
	msg_route_.markers[ROUTE_END].scale.y = route_end_scale;
	msg_route_.markers[ROUTE_END].scale.z = route_end_scale;

	RCLCPP_INFO(ros_node_->get_logger(), "Ib2RouteDisplay plugin configured");
}

void Ib2RouteDisplay::getModels(gz::sim::EntityComponentManager &_ecm)
{
	if (ib2_model_ != gz::sim::kNullEntity)
		return;

	ib2_model_ = _ecm.EntityByComponents(
		gz::sim::components::Name(ib2_name_),
		gz::sim::components::Model());

	if (ib2_model_ == gz::sim::kNullEntity)
		return;

	gz::sim::Model model(ib2_model_);
	ib2_link_ = model.LinkByName(_ecm, "body");
	if (ib2_link_ == gz::sim::kNullEntity)
	{
		ib2_link_ = model.CanonicalLink(_ecm);
	}

	if (ib2_link_ != gz::sim::kNullEntity)
	{
		RCLCPP_INFO(ros_node_->get_logger(), "Found Int-Ball2 model: %s", ib2_name_.c_str());
	}
}

void Ib2RouteDisplay::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	rclcpp::spin_some(ros_node_);

	getModels(_ecm);
	if (ib2_link_ == gz::sim::kNullEntity)
		return;

	double sim_time = std::chrono::duration<double>(_info.simTime).count();

	if (sim_time >= next_timer_time_)
	{
		timerCallback(_ecm, sim_time);
		next_timer_time_ = sim_time + timer_period_;
	}
}

void Ib2RouteDisplay::timerCallback(
	gz::sim::EntityComponentManager &_ecm,
	double sim_time)
{
	// Get Int-Ball2 body world pose from ECM
	gz::sim::Link link(ib2_link_);
	auto worldPose = link.WorldPose(_ecm);
	if (!worldPose)
		return;

	geometry_msgs::msg::Pose pose;
	pose.position.x = worldPose->Pos().X();
	pose.position.y = worldPose->Pos().Y();
	pose.position.z = worldPose->Pos().Z();
	pose.orientation.x = worldPose->Rot().X();
	pose.orientation.y = worldPose->Rot().Y();
	pose.orientation.z = worldPose->Rot().Z();
	pose.orientation.w = worldPose->Rot().W();

	controlHistoryMarkers(pose, sim_time);
	controlRouteMarkers(sim_time);
}

void Ib2RouteDisplay::controlHistoryMarkers(
	const geometry_msgs::msg::Pose &pose,
	double sim_time)
{
	double dx = pose.position.x - msg_history_.pose.position.x;
	double dy = pose.position.y - msg_history_.pose.position.y;
	double dz = pose.position.z - msg_history_.pose.position.z;
	double history_distance = std::sqrt(dx * dx + dy * dy + dz * dz);

	if (force_publish_history_ || (history_distance >= history_min_distance_))
	{
		auto next_history_id = last_published_history_id_ + 1;
		if (last_published_history_id_ > static_cast<unsigned int>(history_max_))
		{
			next_history_id = HISTORY_INDEX_MIN;
		}

		int64_t sec = static_cast<int64_t>(sim_time);
		uint32_t nsec = static_cast<uint32_t>((sim_time - sec) * 1e9);
		msg_history_.header.stamp.sec = static_cast<int32_t>(sec);
		msg_history_.header.stamp.nanosec = nsec;
		msg_history_.id = next_history_id;
		msg_history_.action = visualization_msgs::msg::Marker::ADD;
		msg_history_.pose = pose;

		pub_history_->publish(msg_history_);
		last_published_history_id_ = next_history_id;

		force_publish_history_ = false;
	}
}

void Ib2RouteDisplay::controlRouteMarkers(double sim_time)
{
	if (is_ib2_moving_ && last_ctl_profile_time_ >= last_ib2_stop_time_)
	{
		int64_t sec = static_cast<int64_t>(sim_time);
		uint32_t nsec = static_cast<uint32_t>((sim_time - sec) * 1e9);
		builtin_interfaces::msg::Time timestamp;
		timestamp.sec = static_cast<int32_t>(sec);
		timestamp.nanosec = nsec;

		msg_route_.markers[ROUTE_LINE].header.stamp = timestamp;
		msg_route_.markers[ROUTE_LINE].action = visualization_msgs::msg::Marker::ADD;

		msg_route_.markers[ROUTE_LINE].points.clear();
		for (const auto &ps : ctl_profile_.poses)
		{
			msg_route_.markers[ROUTE_LINE].points.push_back(ps.pose.position);
		}

		if (!ctl_profile_.poses.empty())
		{
			msg_route_.markers[ROUTE_START].header.stamp = timestamp;
			msg_route_.markers[ROUTE_START].action = visualization_msgs::msg::Marker::ADD;
			msg_route_.markers[ROUTE_START].pose = ctl_profile_.poses.front().pose;

			msg_route_.markers[ROUTE_END].header.stamp = timestamp;
			msg_route_.markers[ROUTE_END].action = visualization_msgs::msg::Marker::ADD;
			msg_route_.markers[ROUTE_END].pose = ctl_profile_.poses.back().pose;
		}

		pub_route_->publish(msg_route_);
		is_route_marker_deleted_ = false;
	}
	else
	{
		if (!is_route_marker_deleted_)
		{
			msg_route_.markers[ROUTE_LINE].action = visualization_msgs::msg::Marker::DELETE;
			msg_route_.markers[ROUTE_START].action = visualization_msgs::msg::Marker::DELETE;
			msg_route_.markers[ROUTE_END].action = visualization_msgs::msg::Marker::DELETE;

			pub_route_->publish(msg_route_);
			is_route_marker_deleted_ = true;
		}
	}
}

void Ib2RouteDisplay::ctlStatusCallback(
	const ib2_msgs::msg::CtlStatus::SharedPtr msg)
{
	if (msg->type.type == ib2_msgs::msg::CtlStatusType::STAND_BY ||
	    msg->type.type == ib2_msgs::msg::CtlStatusType::KEEP_POSE ||
	    msg->type.type == ib2_msgs::msg::CtlStatusType::KEEPING_POSE_BY_COLLISION)
	{
		if (is_ib2_moving_)
		{
			is_ib2_moving_ = false;
			last_ib2_stop_time_ = msg->pose.header.stamp.sec
				+ msg->pose.header.stamp.nanosec * 1e-9;
		}
	}
	else
	{
		if (!is_ib2_moving_)
		{
			is_ib2_moving_ = true;
			force_publish_history_ = true;
		}
		ctl_status_ = *msg;
	}
}

void Ib2RouteDisplay::ctlProfileCallback(
	const ib2_msgs::msg::CtlProfile::SharedPtr msg)
{
	ctl_profile_ = *msg;
	last_ctl_profile_time_ = msg->header.stamp.sec
		+ msg->header.stamp.nanosec * 1e-9;
}

}  // namespace ib2_route_display_plugin

GZ_ADD_PLUGIN(
	ib2_route_display_plugin::Ib2RouteDisplay,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ib2_route_display_plugin::Ib2RouteDisplay,
	"ib2_route_display::Ib2RouteDisplay")

// End Of File -----------------------------------------------------------------
