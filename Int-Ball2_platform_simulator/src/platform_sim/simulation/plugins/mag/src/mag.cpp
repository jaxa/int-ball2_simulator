
#include "mag/mag.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/plugin/Register.hh>

namespace
{
	const double DEG2RAD(M_PI / 180.0);
	const double TOL(0.1);
	std::string SERVICE_SWITCH_POWER("/mag/switch_power");
}

//------------------------------------------------------------------------------
mag_plugin::Mag::Mag() = default;
mag_plugin::Mag::~Mag() = default;

//------------------------------------------------------------------------------
void mag_plugin::Mag::Configure(
	const gz::sim::Entity &/*_entity*/,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &/*_ecm*/,
	gz::sim::EventManager &/*_eventMgr*/)
{
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	ros_node_ = std::make_shared<rclcpp::Node>("mag");

	pub_mag_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
		"/mag/wrench_stamped", 1);
	pub_power_status_ = ros_node_->create_publisher<ib2_msgs::msg::PowerStatus>(
		"/mag/power_status", 1);

	switch_power_server_ = ros_node_->create_service<ib2_msgs::srv::SwitchPower>(
		SERVICE_SWITCH_POWER,
		std::bind(&Mag::switchPower, this,
			std::placeholders::_1, std::placeholders::_2));

	mag_param_server_ = ros_node_->create_service<sim_msgs::srv::UpdateParameter>(
		"/sim/mag/update_params",
		std::bind(&Mag::updateParameter, this,
			std::placeholders::_1, std::placeholders::_2));

	getParameter();
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::magCallBack(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	auto fmag_ds = gz::math::Vector3d::Zero;
	auto fmag_bd = gz::math::Vector3d::Zero;
	auto tmag_bd = gz::math::Vector3d::Zero;
	auto r_ds    = gz::math::Vector3d::Zero;
	auto q_ds    = gz::math::Quaterniond(1.0, 0.0, 0.0, 0.0);

	getModels(_ecm);

	if (iss_link_ == gz::sim::kNullEntity || ib2_link_ == gz::sim::kNullEntity)
		return;

	getDsPose(_ecm, r_ds, q_ds);
	getForce(r_ds, q_ds, fmag_ds, fmag_bd);
	getTorque(fmag_bd, tmag_bd);

	if (!power_status_.status)
	{
		fmag_ds = gz::math::Vector3d::Zero;
		fmag_bd = gz::math::Vector3d::Zero;
		tmag_bd = gz::math::Vector3d::Zero;
	}

	addForceAndTorque(_ecm, fmag_bd, tmag_bd);
	pubForceAndTorque(_info, fmag_ds, tmag_bd);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::getParameter()
{
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	double x = 0.0, y = 0.0, z = 0.0;

	get_param("model_name.iss_name", iss_name_);
	get_param("model_name.ib2_name", ib2_name_);

	// JPM Pose
	x = 0.0; y = 0.0; z = 0.0;
	get_param("jpm_pose.pos.x", x);
	get_param("jpm_pose.pos.y", y);
	get_param("jpm_pose.pos.z", z);
	jpm_pos_.Set(x, y, z);

	x = 0.0; y = 0.0; z = 0.0;
	get_param("jpm_pose.att.r", x);
	get_param("jpm_pose.att.p", y);
	get_param("jpm_pose.att.y", z);
	jpm_att_.Set(x, y, z);
	jpm_att_ = jpm_att_ * DEG2RAD;

	// DS Pose
	x = 0.0; y = 0.0; z = 0.0;
	get_param("ds_pose.pos.x", x);
	get_param("ds_pose.pos.y", y);
	get_param("ds_pose.pos.z", z);
	ds_pos_.Set(x, y, z);

	x = 0.0; y = 0.0; z = 0.0;
	get_param("ds_pose.att.r", x);
	get_param("ds_pose.att.p", y);
	get_param("ds_pose.att.y", z);
	ds_att_.Set(x, y, z);
	ds_att_ = ds_att_ * DEG2RAD;

	// Power Status
	bool ps = false;
	get_param("mag_parameter.power_status", ps);
	power_status_.status = ps;

	// Mag IF
	x = 0.0; y = 0.0; z = 0.0;
	get_param("mag_parameter.robo_if.x", x);
	get_param("mag_parameter.robo_if.y", y);
	get_param("mag_parameter.robo_if.z", z);
	rrif_.Set(x, y, z);

	x = 0.0; y = 0.0; z = 0.0;
	get_param("mag_parameter.ds_if.x", x);
	get_param("mag_parameter.ds_if.y", y);
	get_param("mag_parameter.ds_if.z", z);
	rdif_.Set(x, y, z);

	// CG
	x = 0.0; y = 0.0; z = 0.0;
	get_param("robot_mass_property.cg.x", x);
	get_param("robot_mass_property.cg.y", y);
	get_param("robot_mass_property.cg.z", z);
	cg_.Set(x, y, z);

	// Threshold
	get_param("mag_parameter.threshold", d_threshold_);

	// Coefficients
	get_param("mag_parameter.coeff_far.a", afar_);
	get_param("mag_parameter.coeff_far.b", bfar_);
	get_param("mag_parameter.coeff_far.c", cfar_);
	get_param("mag_parameter.coeff_prox.a", aprox_);
	get_param("mag_parameter.coeff_prox.b", bprox_);
	get_param("mag_parameter.coeff_prox.c", cprox_);

	// Standard Deviation
	get_param("mag_parameter.stddev", stddev_);

	// Cycle
	get_param("mag_parameter.cycle", cycle_);

	// Random seed
	int seed = -1;
	get_param("sim_common.random_seed", seed);
	if (seed >= 0)
	{
		RCLCPP_INFO(ros_node_->get_logger(), "Set the random seed value %d", seed);
		gz::math::Rand::Seed(static_cast<unsigned int>(seed));
	}

	logParameter();
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::updateParameter(
	const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> /*req*/,
	std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res)
{
	getParameter();
	res->result = true;
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::getModels(gz::sim::EntityComponentManager &_ecm)
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
			if (!links.empty())
				iss_link_ = links[0];
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
			if (!links.empty())
				ib2_link_ = links[0];
		}
	}
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::getDsPose(
	gz::sim::EntityComponentManager &_ecm,
	gz::math::Vector3d& r_ds, gz::math::Quaterniond& q_ds)
{
	gz::sim::Link ib2Link(ib2_link_);
	gz::sim::Link issLink(iss_link_);

	auto ib2_pose_opt = ib2Link.WorldPose(_ecm);
	auto iss_pose_opt = issLink.WorldPose(_ecm);
	if (!ib2_pose_opt || !iss_pose_opt)
		return;

	auto ib2_pose = ib2_pose_opt.value();
	auto iss_pose = iss_pose_opt.value();

	auto iss_cg = gz::math::Vector3d(iss_pose.Pos().X(), iss_pose.Pos().Y(), iss_pose.Pos().Z());
	coord_transformer_.set(iss_cg, jpm_pos_, jpm_att_, ds_pos_, ds_att_);

	auto iss_qtn   = gz::math::Quaterniond(iss_pose.Rot().W(), iss_pose.Rot().X(), iss_pose.Rot().Y(), iss_pose.Rot().Z());
	auto world_pos = ib2_pose.Pos();
	r_ds           = coord_transformer_.getDsPosFromWorld(world_pos, iss_qtn);

	auto world_qtn = ib2_pose.Rot();
	q_ds           = coord_transformer_.getDsQtnFromWorld(world_qtn, iss_qtn);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::getForce(
	const gz::math::Vector3d& r_ds,   const gz::math::Quaterniond& q_ds,
	      gz::math::Vector3d& fmag_ds,      gz::math::Vector3d&    fmag_bd)
{
	auto   dif   = rdif_ - r_ds;
	auto   rif   = q_ds.RotateVector(rrif_);
	auto   rrifd = r_ds  + rif;
	auto   rho   = rdif_ - rrifd;
	double rhon  = rho.Length();
	double inn   = dif.Dot(rif);

	double nx    = gz::math::Rand::DblNormal(0.0, 1.0);
	double ny    = gz::math::Rand::DblNormal(0.0, 1.0);
	double nz    = gz::math::Rand::DblNormal(0.0, 1.0);
	fmag_ds      = U(inn) * H(rhon) * rho.Normalize();
	fmag_ds.X()  = fmag_ds.X() * (1.0 + stddev_ * nx);
	fmag_ds.Y()  = fmag_ds.Y() * (1.0 + stddev_ * ny);
	fmag_ds.Z()  = fmag_ds.Z() * (1.0 + stddev_ * nz);

	fmag_bd      = q_ds.RotateVectorReverse(fmag_ds);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::getTorque(
	const gz::math::Vector3d& fmag_bd, gz::math::Vector3d& tmag_bd)
{
	auto arm = rrif_ - cg_;
	tmag_bd  = arm.Cross(fmag_bd);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::addForceAndTorque(
	gz::sim::EntityComponentManager &_ecm,
	const gz::math::Vector3d& fmag_bd, const gz::math::Vector3d& tmag_bd)
{
	gz::sim::Link ib2Link(ib2_link_);

	// Convert body-frame force/torque to world-frame
	auto worldPose = ib2Link.WorldPose(_ecm);
	if (!worldPose)
		return;

	auto worldForce  = worldPose->Rot().RotateVector(fmag_bd);
	auto worldTorque = worldPose->Rot().RotateVector(tmag_bd);

	ib2Link.AddWorldWrench(_ecm, worldForce, worldTorque);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::pubForceAndTorque(
	const gz::sim::UpdateInfo &_info,
	const gz::math::Vector3d& fmag_ds, const gz::math::Vector3d& tmag_bd)
{
	if (pub_cnt_ == 0)
	{
		auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
			_info.simTime).count();

		geometry_msgs::msg::WrenchStamped wrench_stamped;
		wrench_stamped.header.stamp.sec =
			static_cast<int32_t>(sim_time_ns / 1000000000LL);
		wrench_stamped.header.stamp.nanosec =
			static_cast<uint32_t>(sim_time_ns % 1000000000LL);
		wrench_stamped.header.frame_id = "";
		wrench_stamped.wrench.force.x  = fmag_ds.X();
		wrench_stamped.wrench.force.y  = fmag_ds.Y();
		wrench_stamped.wrench.force.z  = fmag_ds.Z();
		wrench_stamped.wrench.torque.x = tmag_bd.X();
		wrench_stamped.wrench.torque.y = tmag_bd.Y();
		wrench_stamped.wrench.torque.z = tmag_bd.Z();

		pub_mag_->publish(wrench_stamped);
		pub_power_status_->publish(power_status_);
	}
	else if (pub_cnt_ >= static_cast<int>(cycle_ * 1000 + TOL) - 1)
	{
		pub_cnt_ = -1;
	}
	pub_cnt_++;
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::switchPower(
	const std::shared_ptr<ib2_msgs::srv::SwitchPower::Request> req,
	std::shared_ptr<ib2_msgs::srv::SwitchPower::Response> res)
{
	power_status_.status     = req->power.status;
	res->current_power.status = power_status_.status;
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	// Process pending ROS callbacks (for service servers)
	rclcpp::spin_some(ros_node_);

	magCallBack(_info, _ecm);
}

//------------------------------------------------------------------------------
void mag_plugin::Mag::logParameter()
{
	int ps = power_status_.status;

	RCLCPP_INFO(ros_node_->get_logger(), "***************** Mag Parameter");
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.power_status          : %d", ps);
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.robo_if               : %f %f %f", rrif_.X(), rrif_.Y(), rrif_.Z());
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.ds_if                 : %f %f %f", rdif_.X(), rdif_.Y(), rdif_.Z());
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.threshold             : %f", d_threshold_);
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.coeff_far             : %f %f %f", afar_, bfar_, cfar_);
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.coeff_prox            : %f %f %f", aprox_, bprox_, cprox_);
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.stddev                : %f", stddev_);
	RCLCPP_INFO(ros_node_->get_logger(), "mag_parameter.cycle                 : %f", cycle_);
}

//------------------------------------------------------------------------------
bool mag_plugin::Mag::U(double x)
{
	return (x > 0.0);
}

//------------------------------------------------------------------------------
double mag_plugin::Mag::H(double dist)
{
	dist = std::fabs(dist);

	double a = afar_;
	double b = bfar_;
	double c = cfar_;

	if (dist <= d_threshold_)
	{
		a = aprox_;
		b = bprox_;
		c = cprox_;
	}

	double f_mn = b / pow((1000.0 * dist + a), 2) + c;

	return f_mn / 1000.0;
}

//------------------------------------------------------------------------------
GZ_ADD_PLUGIN(
	mag_plugin::Mag,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(mag_plugin::Mag, "mag::Mag")

// End Of File -----------------------------------------------------------------
