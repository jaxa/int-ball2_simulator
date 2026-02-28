
#include "thr/thr.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/plugin/Register.hh>

namespace
{
	/** 微小値 */
	const double EPS(1.0E-10);
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
thr_plugin::Thr::Thr() = default;

//------------------------------------------------------------------------------
// デストラクタ.
thr_plugin::Thr::~Thr() = default;

//------------------------------------------------------------------------------
// プラグインの初期設定
void thr_plugin::Thr::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	// Store the model entity
	model_entity_ = _entity;

	// Get the first link
	gz::sim::Model model(model_entity_);
	auto links = model.Links(_ecm);
	if (!links.empty())
	{
		link_entity_ = links[0];
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
	ros_node_ = std::make_shared<rclcpp::Node>("thr", node_options);

	// Thr Parameter Update Server
	thr_param_server_ = ros_node_->create_service<sim_msgs::srv::UpdateParameter>(
		"/sim/thr/update_params",
		std::bind(&Thr::updateParameter, this,
			std::placeholders::_1, std::placeholders::_2));

	// Get Parameters
	getParameter();

	// Create subscribers and publishers based on debug mode
	if (debug_)
	{
		sub_wrench_ = ros_node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
			"/ctl/wrench", 1,
			std::bind(&Thr::setCtlCmd, this, std::placeholders::_1));
	}
	else
	{
		sub_duty_ = ros_node_->create_subscription<ib2_msgs::msg::FanStatus>(
			"/prop/status", 1,
			std::bind(&Thr::subFanDuty, this, std::placeholders::_1));

		pub_fan_force_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
			"/thr/fan_force", 1);
	}

	f_.resize(fan_num_, 0.0);
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータ取得
void thr_plugin::Thr::getParameter()
{
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	double x = 0.0, y = 0.0, z = 0.0;

	get_param("thr_parameter.debug", debug_);
	get_param("robot_mass_property.cg.x", x);
	get_param("robot_mass_property.cg.y", y);
	get_param("robot_mass_property.cg.z", z);
	cg_.Set(x, y, z);

	get_param("thr_parameter.fan_num", fan_num_);

	// Mounting Position and Thrust Vector of Fans
	for (int i = 0; i < fan_num_; i++)
	{
		gz::math::Vector3d fp;
		gz::math::Vector3d tv;
		double tf = 0.0, sd = 0.0, k = 0.0, kprop = 0.0;

		std::string fan = "thr_parameter.fan" + std::to_string(i + 1);

		x = 0.0; y = 0.0; z = 0.0;
		get_param(fan + ".pos.x", x);
		get_param(fan + ".pos.y", y);
		get_param(fan + ".pos.z", z);
		fp.Set(x, y, z);
		fan_pos_.push_back(fp);

		x = 0.0; y = 0.0; z = 0.0;
		get_param(fan + ".vec.x", x);
		get_param(fan + ".vec.y", y);
		get_param(fan + ".vec.z", z);
		tv.Set(x, y, z);
		fan_frc_vec_.push_back(tv);

		get_param(fan + ".force", tf);
		fan_frc_.push_back(tf);

		get_param(fan + ".stddev", sd);
		stddev_.push_back(sd);

		get_param(fan + ".kappa", k);
		fan_k_.push_back(k);

		get_param(fan + ".Kprop", kprop);
		k_prop_.push_back(kprop);

		// Set fan's torque vector(unit)
		gz::math::Vector3d tarm = fan_pos_[i] - cg_;
		gz::math::Vector3d trqv = tarm.Cross(fan_frc_vec_[i]);
		gz::math::Vector3d dtrq = fan_k_[i] * fan_frc_vec_[i];
		fan_trq_vec_.push_back(trqv + dtrq);
	}

	// 2nd Order Filter Coefficient
	x = 0.0; y = 0.0; z = 0.0;
	get_param("thr_parameter.filter.coeff_a.a0", x);
	get_param("thr_parameter.filter.coeff_a.a1", y);
	get_param("thr_parameter.filter.coeff_a.a2", z);
	coeff_a_.Set(x, y, z);

	x = 0.0; y = 0.0; z = 0.0;
	get_param("thr_parameter.filter.coeff_b.b0", x);
	get_param("thr_parameter.filter.coeff_b.b1", y);
	get_param("thr_parameter.filter.coeff_b.b2", z);
	coeff_b_.Set(x, y, z);

	// 2nd Order Filter Buffer Initialization
	in_.reserve(fan_num_);
	out_.reserve(fan_num_);
	for (int i = 0; i < fan_num_; i++)
	{
		in_.push_back({0.0, 0.0});
		out_.push_back({0.0, 0.0});
	}

	// 乱数のシード値が設定されている場合は読み込む
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
// パラメータ更新サービス
void thr_plugin::Thr::updateParameter(
	const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> /*req*/,
	std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res)
{
	fan_pos_.clear();
	fan_frc_vec_.clear();
	fan_frc_.clear();
	stddev_.clear();
	fan_k_.clear();
	k_prop_.clear();

	getParameter();

	res->result = true;
}

//------------------------------------------------------------------------------
// ファン駆動デューティ比をサブスクライブ
void thr_plugin::Thr::subFanDuty(const ib2_msgs::msg::FanStatus::SharedPtr msg)
{
	int size = static_cast<int>(msg->duty.data.size());

	assert(size == fan_num_);

	for (int i = 0; i < size; i++)
	{
		assert(std::abs(k_prop_[i]) > EPS);

		double t = msg->duty.data[i] / k_prop_[i];
		f_.at(i) = std::min(t * t, fan_frc_[i]);
	}
}

//------------------------------------------------------------------------------
// 制御コマンドを直接設定
void thr_plugin::Thr::setCtlCmd(const geometry_msgs::msg::WrenchStamped::SharedPtr ctl)
{
	force_.X()  = ctl->wrench.force.x;
	force_.Y()  = ctl->wrench.force.y;
	force_.Z()  = ctl->wrench.force.z;
	torque_.X() = ctl->wrench.torque.x;
	torque_.Y() = ctl->wrench.torque.y;
	torque_.Z() = ctl->wrench.torque.z;
}

//------------------------------------------------------------------------------
// 物理ステップ前の更新
void thr_plugin::Thr::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	if (link_entity_ == gz::sim::kNullEntity)
		return;

	// Process pending ROS callbacks
	rclcpp::spin_some(ros_node_);

	addForceAndTorque(_ecm);
}

//------------------------------------------------------------------------------
// 力・トルクのGazeboへの設定
void thr_plugin::Thr::addForceAndTorque(gz::sim::EntityComponentManager &_ecm)
{
	gz::sim::Link link(link_entity_);

	if (!debug_)
	{
		// Initialize force and torque
		force_.Set();
		torque_.Set();

		int size = static_cast<int>(f_.size());

		std_msgs::msg::Float64MultiArray fanforce;
		fanforce.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
		fanforce.layout.dim[0].size   = size;
		fanforce.layout.dim[0].stride = 1;
		fanforce.layout.dim[0].label  = "fan_force";
		fanforce.layout.data_offset   = 0;
		fanforce.data.resize(size, 0.0);

		for (int i = 0; i < size; i++)
		{
			double f_fltrd = biQuadFilter(coeff_a_, coeff_b_, f_[i],
				in_[i].data(), out_[i].data(), false);

			double ratio     = stddev_[i] / fan_frc_[i];
			double fanNoise  = gz::math::Rand::DblNormal(0.0, f_fltrd * ratio);
			fanforce.data[i] = f_fltrd + fanNoise;
			force_           = force_  + fanforce.data[i] * fan_frc_vec_[i];
			torque_          = torque_ + fanforce.data[i] * fan_trq_vec_[i];
		}

		// Publish fan force Message
		pub_fan_force_->publish(fanforce);
	}

	// Convert body-frame force/torque to world-frame
	auto worldPose = link.WorldPose(_ecm);
	if (!worldPose)
		return;

	auto worldForce  = worldPose->Rot().RotateVector(force_);
	auto worldTorque = worldPose->Rot().RotateVector(torque_);

	link.AddWorldWrench(_ecm, worldForce, worldTorque);
}

//------------------------------------------------------------------------------
// 双二次フィルタ
double thr_plugin::Thr::biQuadFilter(
	const gz::math::Vector3d& a, const gz::math::Vector3d& b,
	const double& in, double* ibuf, double* obuf, bool rst)
{
	assert(std::abs(a[0]) > EPS);

	if (rst)
	{
		ibuf[0] = ibuf[1] = 0.0;
		obuf[0] = obuf[1] = 0.0;
	}

	double output = (b[0] * in + b[1] * ibuf[0] + b[2] * ibuf[1]
		- a[1] * obuf[0] - a[2] * obuf[1]) / a[0];

	ibuf[1] = ibuf[0]; ibuf[0] = in;
	obuf[1] = obuf[0]; obuf[0] = output;

	return output;
}

//------------------------------------------------------------------------------
// 推力プラグインパラメータログ作成
void thr_plugin::Thr::logParameter()
{
	RCLCPP_INFO(ros_node_->get_logger(), "***************** Thr Parameter");
	RCLCPP_INFO(ros_node_->get_logger(), "thr_parameter.debug             : %d", debug_);
	RCLCPP_INFO(ros_node_->get_logger(), "robot_mass_property.cg          : %f %f %f", cg_.X(), cg_.Y(), cg_.Z());
	RCLCPP_INFO(ros_node_->get_logger(), "thr_parameter.fan_num           : %d", fan_num_);

	for (int i = 0; i < fan_num_; i++)
	{
		std::string fan = "fan" + std::to_string(i + 1);
		RCLCPP_INFO(ros_node_->get_logger(), "%s pos   : %f %f %f", fan.c_str(), fan_pos_[i].X(), fan_pos_[i].Y(), fan_pos_[i].Z());
		RCLCPP_INFO(ros_node_->get_logger(), "%s vec   : %f %f %f", fan.c_str(), fan_frc_vec_[i].X(), fan_frc_vec_[i].Y(), fan_frc_vec_[i].Z());
		RCLCPP_INFO(ros_node_->get_logger(), "%s force : %f", fan.c_str(), fan_frc_[i]);
		RCLCPP_INFO(ros_node_->get_logger(), "%s stddev: %f", fan.c_str(), stddev_[i]);
		RCLCPP_INFO(ros_node_->get_logger(), "%s kappa : %f", fan.c_str(), fan_k_[i]);
		RCLCPP_INFO(ros_node_->get_logger(), "%s Kprop : %f", fan.c_str(), k_prop_[i]);
	}

	RCLCPP_INFO(ros_node_->get_logger(), "filter coeff_a: %f %f %f", coeff_a_.X(), coeff_a_.Y(), coeff_a_.Z());
	RCLCPP_INFO(ros_node_->get_logger(), "filter coeff_b: %f %f %f", coeff_b_.X(), coeff_b_.Y(), coeff_b_.Z());
}

//------------------------------------------------------------------------------
// gz-simのシステムプラグインとして登録
GZ_ADD_PLUGIN(
	thr_plugin::Thr,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(thr_plugin::Thr, "thr::Thr")

// End Of File -----------------------------------------------------------------
