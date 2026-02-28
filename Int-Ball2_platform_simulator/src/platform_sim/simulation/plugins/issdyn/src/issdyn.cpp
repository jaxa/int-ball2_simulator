
#include "issdyn/issdyn.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/plugin/Register.hh>

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 2PI */
	const double PI2(2.0 * M_PI);

	const std::string FRAME_ISS("iss_body");
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
issdyn_plugin::Issdyn::Issdyn() = default;

//------------------------------------------------------------------------------
// デストラクタ.
issdyn_plugin::Issdyn::~Issdyn() = default;

//------------------------------------------------------------------------------
// プラグインの初期設定
void issdyn_plugin::Issdyn::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{
	// Initialize rclcpp if it has not already been initialized
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	// Store the Model Entity
	model_entity_ = _entity;

	// Get the first link
	gz::sim::Model model(model_entity_);
	auto links = model.Links(_ecm);
	if (!links.empty())
	{
		iss_link_ = links[0];
	}

	// Load simulation parameter files
	rclcpp::NodeOptions node_options;
	try {
		std::string ib2_gazebo_share = ament_index_cpp::get_package_share_directory("ib2_gazebo");
		node_options.arguments({
			"--ros-args",
			"--params-file", ib2_gazebo_share + "/sim/sim.yaml",
			"--params-file", ib2_gazebo_share + "/sim/custom.yaml"
		});
	} catch (...) {}

	// Create ROS node
	ros_node_ = std::make_shared<rclcpp::Node>("issdyn", node_options);

	// Create a Navigation topic, and publish it
	pub_nav_ = ros_node_->create_publisher<ib2_msgs::msg::Navigation>(
		"/sim/iss_navigation", 1);

	// Get ISS Attitude Fluctuation Parameter
	getParameter();
}

//------------------------------------------------------------------------------
//  ROS Parameter Serverからパラメータ取得
void issdyn_plugin::Issdyn::getParameter()
{
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	get_param("issdyn_parameter.cycle", pub_cycle_);

	double x = 0.0, y = 0.0, z = 0.0;

	get_param("issdyn_parameter.slope.x", x);
	get_param("issdyn_parameter.slope.y", y);
	get_param("issdyn_parameter.slope.z", z);
	att_bias_slope_.Set(x, y, z);
	att_bias_slope_ = att_bias_slope_ * DEG2RAD;

	x = 0.0; y = 0.0; z = 0.0;
	get_param("issdyn_parameter.gain.x", x);
	get_param("issdyn_parameter.gain.y", y);
	get_param("issdyn_parameter.gain.z", z);
	att_fluc_gain_.Set(x, y, z);
	att_fluc_gain_ = att_fluc_gain_ * DEG2RAD;

	x = 0.0; y = 0.0; z = 0.0;
	get_param("issdyn_parameter.freq.x", x);
	get_param("issdyn_parameter.freq.y", y);
	get_param("issdyn_parameter.freq.z", z);
	att_fluc_freq_.Set(x, y, z);
}

//------------------------------------------------------------------------------
// 物理ステップ前の更新
void issdyn_plugin::Issdyn::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	if (iss_link_ == gz::sim::kNullEntity)
		return;

	gz::sim::Link issLink(iss_link_);

	// Enable velocity checks if not already done
	if (!velocity_checks_enabled_)
	{
		issLink.EnableVelocityChecks(_ecm, true);
		velocity_checks_enabled_ = true;
	}

	setIssAttitude(_info, _ecm);
	pubIssNav(_info, _ecm);
}

//------------------------------------------------------------------------------
// ISSの姿勢変動を設定
void issdyn_plugin::Issdyn::setIssAttitude(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	double sim_time = std::chrono::duration<double>(_info.simTime).count();

	if (start_flag_)
	{
		double t = sim_time - start_time_;
		gz::math::Vector3d w;
		w.X() = PI2 * att_fluc_freq_.X() * att_fluc_gain_.X() * cos(PI2 * att_fluc_freq_.X() * t) + att_bias_slope_.X();
		w.Y() = PI2 * att_fluc_freq_.Y() * att_fluc_gain_.Y() * cos(PI2 * att_fluc_freq_.Y() * t) + att_bias_slope_.Y();
		w.Z() = PI2 * att_fluc_freq_.Z() * att_fluc_gain_.Z() * cos(PI2 * att_fluc_freq_.Z() * t) + att_bias_slope_.Z();

		// Set angular velocity via WorldAngularVelocityCmd component
		auto *angVelCmd = _ecm.Component<gz::sim::components::WorldAngularVelocityCmd>(iss_link_);
		if (angVelCmd)
		{
			*angVelCmd = gz::sim::components::WorldAngularVelocityCmd(w);
		}
		else
		{
			_ecm.CreateComponent(iss_link_,
				gz::sim::components::WorldAngularVelocityCmd(w));
		}
	}
	else
	{
		start_time_ = sim_time;
		start_flag_ = true;
	}
}

//------------------------------------------------------------------------------
// ISS航法値のパブリッシュ
void issdyn_plugin::Issdyn::pubIssNav(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	double sim_time = std::chrono::duration<double>(_info.simTime).count();

	// Check publish rate
	if (last_pub_time_ >= 0.0 && (sim_time - last_pub_time_) < pub_cycle_)
		return;
	last_pub_time_ = sim_time;

	gz::sim::Link issLink(iss_link_);

	auto pose_opt    = issLink.WorldPose(_ecm);
	auto lin_vel_opt = issLink.WorldLinearVelocity(_ecm);
	auto ang_vel_opt = issLink.WorldAngularVelocity(_ecm);

	if (!pose_opt || !lin_vel_opt || !ang_vel_opt)
		return;

	auto p_iss  = pose_opt.value();
	auto v_iss  = lin_vel_opt.value();
	auto w_world = ang_vel_opt.value();

	// Convert world angular velocity to body-frame
	auto wb_iss = p_iss.Rot().RotateVectorReverse(w_world);

	// Build sim time stamp
	auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		_info.simTime).count();
	builtin_interfaces::msg::Time stamp;
	stamp.sec = static_cast<int32_t>(sim_time_ns / 1000000000LL);
	stamp.nanosec = static_cast<uint32_t>(sim_time_ns % 1000000000LL);

	// Publish ISS Navigation
	ib2_msgs::msg::Navigation iss_nav;

	iss_nav.pose.header.stamp       = stamp;
	iss_nav.pose.header.frame_id    = FRAME_ISS;
	iss_nav.pose.pose.position.x    = p_iss.Pos().X();
	iss_nav.pose.pose.position.y    = p_iss.Pos().Y();
	iss_nav.pose.pose.position.z    = p_iss.Pos().Z();
	iss_nav.pose.pose.orientation.x = p_iss.Rot().X();
	iss_nav.pose.pose.orientation.y = p_iss.Rot().Y();
	iss_nav.pose.pose.orientation.z = p_iss.Rot().Z();
	iss_nav.pose.pose.orientation.w = p_iss.Rot().W();
	iss_nav.twist.linear.x          = v_iss.X();
	iss_nav.twist.linear.y          = v_iss.Y();
	iss_nav.twist.linear.z          = v_iss.Z();
	iss_nav.twist.angular.x         = wb_iss.X();
	iss_nav.twist.angular.y         = wb_iss.Y();
	iss_nav.twist.angular.z         = wb_iss.Z();
	iss_nav.a.x                     = 0.0;
	iss_nav.a.y                     = 0.0;
	iss_nav.a.z                     = 0.0;
	iss_nav.status.status           = ib2_msgs::msg::NavigationStatus::NAV_FUSION;

	pub_nav_->publish(iss_nav);
}

//------------------------------------------------------------------------------
// gz-simのシステムプラグインとして登録
GZ_ADD_PLUGIN(
	issdyn_plugin::Issdyn,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(issdyn_plugin::Issdyn, "issdyn::Issdyn")

// End Of File -----------------------------------------------------------------
