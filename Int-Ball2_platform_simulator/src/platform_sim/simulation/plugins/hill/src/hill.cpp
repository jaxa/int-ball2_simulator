
#include "hill/hill.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 微小値 */
	const double EPS(1.0E-10);

	/** 許容誤差 */
	const double TOL(0.1);
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
hill_plugin::Hill::Hill() = default;

//------------------------------------------------------------------------------
// デストラクタ.
hill_plugin::Hill::~Hill() = default;

//------------------------------------------------------------------------------
// プラグインの初期設定
void hill_plugin::Hill::Configure(
	const gz::sim::Entity &/*_entity*/,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &/*_ecm*/,
	gz::sim::EventManager &/*_eventMgr*/)
{
	// Initialize rclcpp if it has not already been initialized
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	// Create ROS node
	ros_node_ = std::make_shared<rclcpp::Node>("hill");

	// Publish Hill Force
	pub_hill_force_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
		"/hill/force", 1);

	// Get parameters
	getParameter();
}

//------------------------------------------------------------------------------
//  ROS Parameter Serverからパラメータ取得
void hill_plugin::Hill::getParameter()
{
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	get_param("model_name.iss_name", iss_name_);
	if (iss_name_.empty()) {
		RCLCPP_ERROR(ros_node_->get_logger(), "Cannot Get model_name.iss_name in hill plugin");
	}

	get_param("model_name.ib2_name", ib2_name_);
	if (ib2_name_.empty()) {
		RCLCPP_ERROR(ros_node_->get_logger(), "Cannot Get model_name.ib2_name in hill plugin");
	}

	get_param("hill_parameter.cycle", pub_cycle_);
	get_param("hill_parameter.w", iss_w_);

	iss_w_ = iss_w_ * DEG2RAD;
}

//------------------------------------------------------------------------------
// Model取得
void hill_plugin::Hill::getModels(gz::sim::EntityComponentManager &_ecm)
{
	if (iss_model_ == gz::sim::kNullEntity)
	{
		iss_model_ = _ecm.EntityByComponents(
			gz::sim::components::Name(iss_name_),
			gz::sim::components::Model());
		if (iss_model_ != gz::sim::kNullEntity)
		{
			gz::sim::Model issModel(iss_model_);
			auto links = issModel.Links(_ecm);
			if (!links.empty()) {
				iss_link_ = links[0];
			}
		}
	}
	if (ib2_model_ == gz::sim::kNullEntity)
	{
		ib2_model_ = _ecm.EntityByComponents(
			gz::sim::components::Name(ib2_name_),
			gz::sim::components::Model());
		if (ib2_model_ != gz::sim::kNullEntity)
		{
			gz::sim::Model ib2Model(ib2_model_);
			auto links = ib2Model.Links(_ecm);
			if (!links.empty()) {
				ib2_link_ = links[0];
			}
		}
	}
}

//------------------------------------------------------------------------------
// 物理ステップ前の更新
void hill_plugin::Hill::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	addHillForce(_info, _ecm);
}

//------------------------------------------------------------------------------
// 相対加速度(Hill方程式)の印加
void hill_plugin::Hill::addHillForce(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	getModels(_ecm);

	if (iss_link_ == gz::sim::kNullEntity || ib2_link_ == gz::sim::kNullEntity)
		return;

	gz::sim::Link issLink(iss_link_);
	gz::sim::Link ib2Link(ib2_link_);

	// Enable velocity checks if not already done
	if (!velocity_checks_enabled_)
	{
		ib2Link.EnableVelocityChecks(_ecm, true);
		issLink.EnableVelocityChecks(_ecm, true);
		velocity_checks_enabled_ = true;
		return;  // Velocities won't be available until next step
	}

	auto iss_pose_opt = issLink.WorldPose(_ecm);
	auto ib2_pose_opt = ib2Link.WorldPose(_ecm);
	auto vel_opt      = ib2Link.WorldLinearVelocity(_ecm);

	if (!iss_pose_opt || !ib2_pose_opt || !vel_opt)
		return;

	auto iss_pose = iss_pose_opt.value();
	auto ib2_pose = ib2_pose_opt.value();
	auto v        = vel_opt.value();

	// Get mass from inertial component
	auto *inertialComp = _ecm.Component<gz::sim::components::Inertial>(ib2_link_);
	if (!inertialComp)
		return;
	double m = inertialComp->Data().MassMatrix().Mass();

	auto pr = ib2_pose.Pos() - iss_pose.Pos();

	double w  = iss_w_;
	double w2 = std::pow(w, 2.0);

	hill_force_.X() = m * ( 2.0 * w * v.Z());
	hill_force_.Y() = m * (                       - w2 * pr.Y());
	hill_force_.Z() = m * (-2.0 * w * v.X() + 3.0 * w2 * pr.Z());

	// Add Hill Force
	ib2Link.AddWorldForce(_ecm, hill_force_);

	// Publish Hill Force
	if (pub_cnt_ == 0)
	{
		auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
			_info.simTime).count();

		geometry_msgs::msg::WrenchStamped force_stamped;
		force_stamped.header.stamp.sec =
			static_cast<int32_t>(sim_time_ns / 1000000000LL);
		force_stamped.header.stamp.nanosec =
			static_cast<uint32_t>(sim_time_ns % 1000000000LL);
		force_stamped.header.frame_id = "";
		force_stamped.wrench.force.x  = hill_force_.X();
		force_stamped.wrench.force.y  = hill_force_.Y();
		force_stamped.wrench.force.z  = hill_force_.Z();
		force_stamped.wrench.torque.x = 0.0;
		force_stamped.wrench.torque.y = 0.0;
		force_stamped.wrench.torque.z = 0.0;

		pub_hill_force_->publish(force_stamped);
	}
	else if (pub_cnt_ >= static_cast<int>(pub_cycle_ * 1000 + TOL) - 1)
	{
		pub_cnt_ = -1;
	}
	pub_cnt_++;
}

//------------------------------------------------------------------------------
// gz-simのシステムプラグインとして登録
GZ_ADD_PLUGIN(
	hill_plugin::Hill,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(hill_plugin::Hill, "hill::Hill")

// End Of File -----------------------------------------------------------------
