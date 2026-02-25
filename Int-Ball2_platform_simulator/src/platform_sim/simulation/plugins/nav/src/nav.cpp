
#include "nav/nav.h"

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/plugin/Register.hh>

#include <limits>

namespace
{
	const double DEG2RAD(M_PI / 180.0);
	const double RAD2DEG(180.0 / M_PI);
	const double EPS(1.0E-10);
	const std::string FRAME_ISS("iss_body");
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
nav_plugin::Nav::Nav() :
	accum_counter_(0),
	delta_v_(gz::math::Vector3d::Zero),
	delta_angle_(gz::math::Vector3d::Zero)
{}

//------------------------------------------------------------------------------
// デストラクタ.
nav_plugin::Nav::~Nav()
{
	if (ifs_.is_open())
	{
		ifs_.close();
	}
}

//------------------------------------------------------------------------------
// プラグインの初期設定
void nav_plugin::Nav::Configure(
	const gz::sim::Entity &/*_entity*/,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &/*_ecm*/,
	gz::sim::EventManager &/*_eventMgr*/)
{
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	// Create ROS node
	ros_node_ = std::make_shared<rclcpp::Node>("nav");

	// Get Parameters
	if (getParameter() != 0)
	{
		return;
	}

	// Open Nav Error CSV
	openCSVFile();

	// Get initial control period
	double cnt_duration = controlFreqFluctuation(0.0);
	next_nav_time_    = cnt_duration;
	next_status_time_ = cnt_duration;

	// Publishers
	pub_nav_                  = ros_node_->create_publisher<ib2_msgs::msg::Navigation>("/sensor_fusion/navigation", 1);
	pub_sensor_fusion_status_ = ros_node_->create_publisher<ib2_msgs::msg::NavigationStatus>("/sensor_fusion/navigation_status", 1);
	pub_att_                  = ros_node_->create_publisher<sim_msgs::msg::Attitude>("/nav/attitude", 1);
	pub_true_nav_             = ros_node_->create_publisher<ib2_msgs::msg::Navigation>("/nav/true/navigation", 1);
	pub_true_att_             = ros_node_->create_publisher<sim_msgs::msg::Attitude>("/nav/true/attitude", 1);

	// Subscribers
	sub_status_ = ros_node_->create_subscription<std_msgs::msg::Int32>(
		"/nav/status", 5,
		[this](const std_msgs::msg::Int32::SharedPtr msg) {
			if (msg->data >= 0) {
				status_ = static_cast<uint8_t>(msg->data);
				invalid_nav_ = false;
			} else {
				invalid_nav_ = true;
				status_ = static_cast<uint8_t>(-msg->data);
			}
		});

	sub_time_offset_ = ros_node_->create_subscription<std_msgs::msg::Float64>(
		"/nav/time_offset", 5,
		[this](const std_msgs::msg::Float64::SharedPtr msg) {
			tnav_offset_ = msg->data;
		});

	// Service Servers
	nav_param_server_ = ros_node_->create_service<sim_msgs::srv::UpdateParameter>(
		"/sim/nav/update_params",
		std::bind(&Nav::updateParameter, this,
			std::placeholders::_1, std::placeholders::_2));

	marker_correction_server_ = ros_node_->create_service<ib2_msgs::srv::MarkerCorrection>(
		"/sensor_fusion/marker_correction",
		std::bind(&Nav::markerCorrection, this,
			std::placeholders::_1, std::placeholders::_2));

	switch_power_server_ = ros_node_->create_service<ib2_msgs::srv::SwitchPower>(
		"/nav/switch_power",
		std::bind(&Nav::switchPower, this,
			std::placeholders::_1, std::placeholders::_2));

	// Action Server
	navigation_start_up_ = rclcpp_action::create_server<NavigationStartUp>(
		ros_node_,
		"/sensor_fusion/navigation_start_up",
		std::bind(&Nav::handleGoal, this,
			std::placeholders::_1, std::placeholders::_2),
		std::bind(&Nav::handleCancel, this,
			std::placeholders::_1),
		std::bind(&Nav::handleAccepted, this,
			std::placeholders::_1));

	// Initial state
	nav_running_ = initial_nav_on_;
	status_      = initial_nav_on_
		? ib2_msgs::msg::NavigationStatus::NAV_FUSION
		: ib2_msgs::msg::NavigationStatus::NAV_OFF;
	tnav_offset_ = 0.0;
	invalid_nav_ = false;
}

//------------------------------------------------------------------------------
// 物理ステップ前の更新
void nav_plugin::Nav::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	// Process pending ROS callbacks
	rclcpp::spin_some(ros_node_);

	double sim_time = std::chrono::duration<double>(_info.simTime).count();
	current_sim_time_ = sim_time;

	// Find models
	getModels(_ecm);

	if (ib2_link_ == gz::sim::kNullEntity)
		return;

	// Enable velocity/acceleration checks on first call
	if (!velocity_checks_enabled_)
	{
		gz::sim::Link ib2_lnk(ib2_link_);
		ib2_lnk.EnableVelocityChecks(_ecm);
		ib2_lnk.EnableAccelerationChecks(_ecm);

		if (iss_link_ != gz::sim::kNullEntity)
		{
			gz::sim::Link iss_lnk(iss_link_);
			iss_lnk.EnableVelocityChecks(_ecm);
		}

		velocity_checks_enabled_ = true;
		return;
	}

	// Accumulate acceleration and angular rate every physics step
	sumAcclAndAttRate(_ecm);

	// Nav callback (timer simulation)
	if (nav_running_ && sim_time >= next_nav_time_)
	{
		navCallBack(_info, _ecm);
		double period = controlFreqFluctuation(sim_time);
		next_nav_time_ = sim_time + period;
	}

	// Sensor fusion status callback (timer simulation)
	if (sim_time >= next_status_time_)
	{
		sensorFusionStatusCallBack(_info);
		double period = controlFreqFluctuation(sim_time);
		next_status_time_ = sim_time + period;
	}

	// Check pending action completion
	if (pending_action_ && sim_time >= pending_action_->complete_time)
	{
		auto result = std::make_shared<NavigationStartUp::Result>();
		result->type = pending_action_->result_type;
		auto sim_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
		result->stamp.sec     = static_cast<int32_t>(sim_ns / 1000000000);
		result->stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000);
		pending_action_->goal_handle->succeed(result);
		pending_action_.reset();
	}
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
int nav_plugin::Nav::getParameter()
{
	int ret = 0;
	double x1 = 0.0, y1 = 0.0, z1 = 0.0;
	double x2 = 0.0, y2 = 0.0, z2 = 0.0;

	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	// Model Name
	iss_name_ = "";
	ib2_name_ = "";
	get_param("model_name.iss_name", iss_name_);
	get_param("model_name.ib2_name", ib2_name_);
	if (iss_name_.empty())
	{
		RCLCPP_ERROR(ros_node_->get_logger(), "Cannot Get model_name.iss_name");
		ret |= 0x0001;
	}
	if (ib2_name_.empty())
	{
		RCLCPP_ERROR(ros_node_->get_logger(), "Cannot Get model_name.ib2_name");
		ret |= 0x0002;
	}

	// JPM Pose
	x1 = 0.0; y1 = 0.0; z1 = 0.0;
	get_param("jpm_pose.pos.x", x1);
	get_param("jpm_pose.pos.y", y1);
	get_param("jpm_pose.pos.z", z1);
	jpm_pos_.Set(x1, y1, z1);

	x1 = 0.0; y1 = 0.0; z1 = 0.0;
	get_param("jpm_pose.att.r", x1);
	get_param("jpm_pose.att.p", y1);
	get_param("jpm_pose.att.y", z1);
	jpm_att_.Set(x1, y1, z1);
	jpm_att_ = jpm_att_ * DEG2RAD;

	// DS Pose
	x1 = 0.0; y1 = 0.0; z1 = 0.0;
	get_param("ds_pose.pos.x", x1);
	get_param("ds_pose.pos.y", y1);
	get_param("ds_pose.pos.z", z1);
	ds_pos_.Set(x1, y1, z1);

	x1 = 0.0; y1 = 0.0; z1 = 0.0;
	get_param("ds_pose.att.r", x1);
	get_param("ds_pose.att.p", y1);
	get_param("ds_pose.att.y", z1);
	ds_att_.Set(x1, y1, z1);
	ds_att_ = ds_att_ * DEG2RAD;

	// Navigation Error Parameter
	get_param("nav_parameter.error.error_source_csv", error_source_csv_);
	get_param("nav_parameter.error.csv", csv_file_name_);

	plugin_path_ = "";
	get_param("nav_parameter.plugin_path", plugin_path_);

	x1 = 0.0; y1 = 0.0; z1 = 0.0; x2 = 0.0; y2 = 0.0; z2 = 0.0;
	get_param("nav_parameter.error.pos.mean.x",   x1);
	get_param("nav_parameter.error.pos.mean.y",   y1);
	get_param("nav_parameter.error.pos.mean.z",   z1);
	get_param("nav_parameter.error.pos.stddev.x", x2);
	get_param("nav_parameter.error.pos.stddev.y", y2);
	get_param("nav_parameter.error.pos.stddev.z", z2);
	bias_p_.Set(x1, y1, z1);
	rand_p_.Set(x2, y2, z2);

	x1 = 0.0; y1 = 0.0; z1 = 0.0; x2 = 0.0; y2 = 0.0; z2 = 0.0;
	get_param("nav_parameter.error.vel.mean.x",   x1);
	get_param("nav_parameter.error.vel.mean.y",   y1);
	get_param("nav_parameter.error.vel.mean.z",   z1);
	get_param("nav_parameter.error.vel.stddev.x", x2);
	get_param("nav_parameter.error.vel.stddev.y", y2);
	get_param("nav_parameter.error.vel.stddev.z", z2);
	bias_v_.Set(x1, y1, z1);
	rand_v_.Set(x2, y2, z2);

	x1 = 0.0; y1 = 0.0; z1 = 0.0; x2 = 0.0; y2 = 0.0; z2 = 0.0;
	get_param("nav_parameter.error.acc.mean.x",   x1);
	get_param("nav_parameter.error.acc.mean.y",   y1);
	get_param("nav_parameter.error.acc.mean.z",   z1);
	get_param("nav_parameter.error.acc.stddev.x", x2);
	get_param("nav_parameter.error.acc.stddev.y", y2);
	get_param("nav_parameter.error.acc.stddev.z", z2);
	bias_a_.Set(x1, y1, z1);
	rand_a_.Set(x2, y2, z2);

	x1 = 0.0; y1 = 0.0; z1 = 0.0; x2 = 0.0; y2 = 0.0; z2 = 0.0;
	get_param("nav_parameter.error.att.mean.x",   x1);
	get_param("nav_parameter.error.att.mean.y",   y1);
	get_param("nav_parameter.error.att.mean.z",   z1);
	get_param("nav_parameter.error.att.stddev.x", x2);
	get_param("nav_parameter.error.att.stddev.y", y2);
	get_param("nav_parameter.error.att.stddev.z", z2);
	bias_r_.Set(x1, y1, z1);
	rand_r_.Set(x2, y2, z2);

	x1 = 0.0; y1 = 0.0; z1 = 0.0; x2 = 0.0; y2 = 0.0; z2 = 0.0;
	get_param("nav_parameter.error.att_rate.mean.x",   x1);
	get_param("nav_parameter.error.att_rate.mean.y",   y1);
	get_param("nav_parameter.error.att_rate.mean.z",   z1);
	get_param("nav_parameter.error.att_rate.stddev.x", x2);
	get_param("nav_parameter.error.att_rate.stddev.y", y2);
	get_param("nav_parameter.error.att_rate.stddev.z", z2);
	bias_w_.Set(x1, y1, z1);
	rand_w_.Set(x2, y2, z2);

	// Control Frequency Fluctuation
	get_param("nav_parameter.control.mean",   bias_cnt_);
	get_param("nav_parameter.control.stddev", rand_cnt_);
	get_param("nav_parameter.control.gain",   gain_cnt_);
	get_param("nav_parameter.control.freq",   freq_cnt_);

	// Navigation Delay
	get_param("nav_parameter.delay", delay_);

	// Initial state of Navigation
	get_param("nav_parameter.initial_nav_on", initial_nav_on_);

	// 乱数のシード値
	int seed = -1;
	get_param("sim_common.random_seed", seed);
	if (seed >= 0)
	{
		RCLCPP_INFO(ros_node_->get_logger(), "Set the random seed value %d", seed);
		gz::math::Rand::Seed(static_cast<unsigned int>(seed));
	}

	return ret;
}

//------------------------------------------------------------------------------
// パラメータ更新サービス
void nav_plugin::Nav::updateParameter(
	const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> /*req*/,
	std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res)
{
	res->result = false;

	if (getParameter() != 0)
		return;

	if (error_source_csv_)
	{
		openCSVFile();
	}

	res->result = true;
}

//------------------------------------------------------------------------------
// マーカー補正サービス
void nav_plugin::Nav::markerCorrection(
	const std::shared_ptr<ib2_msgs::srv::MarkerCorrection::Request> /*req*/,
	std::shared_ptr<ib2_msgs::srv::MarkerCorrection::Response> res)
{
	int marker = 0;
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};
	get_param("nav_parameter.marker", marker);

	auto sim_ns = static_cast<int64_t>(current_sim_time_ * 1e9);
	res->stamp.sec     = static_cast<int32_t>(sim_ns / 1000000000);
	res->stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000);
	res->status = marker > 0
		? ib2_msgs::srv::MarkerCorrection::Response::SUCCESS
		: ib2_msgs::srv::MarkerCorrection::Response::FAILURE_UPDATE;
}

//------------------------------------------------------------------------------
// Nav ON/OFF サービス
void nav_plugin::Nav::switchPower(
	const std::shared_ptr<ib2_msgs::srv::SwitchPower::Request> req,
	std::shared_ptr<ib2_msgs::srv::SwitchPower::Response> res)
{
	if ((status_ == ib2_msgs::msg::NavigationStatus::NAV_OFF &&
	     req->power.status == ib2_msgs::msg::PowerStatus::OFF) ||
	    (status_ == ib2_msgs::msg::NavigationStatus::NAV_FUSION &&
	     req->power.status == ib2_msgs::msg::PowerStatus::ON))
	{
		res->current_power.status = req->power.status;
		return;
	}

	if (req->power.status == ib2_msgs::msg::PowerStatus::OFF)
	{
		nav_running_ = false;
		status_ = ib2_msgs::msg::NavigationStatus::NAV_OFF;
		res->current_power.status = ib2_msgs::msg::PowerStatus::OFF;

		while (!nav_buffer_.empty())
		{
			nav_buffer_.pop();
		}
		return;
	}

	nav_running_ = true;
	next_nav_time_ = current_sim_time_;

	if (status_ == ib2_msgs::msg::NavigationStatus::NAV_OFF)
	{
		status_ = ib2_msgs::msg::NavigationStatus::NAV_FUSION;
		res->current_power.status = ib2_msgs::msg::PowerStatus::ON;
	}
}

//------------------------------------------------------------------------------
// Nav ON/OFF 内部処理 (Action server用)
void nav_plugin::Nav::switchPowerInternal(uint8_t power_status)
{
	if (power_status == ib2_msgs::msg::PowerStatus::OFF)
	{
		nav_running_ = false;
		status_ = ib2_msgs::msg::NavigationStatus::NAV_OFF;
		while (!nav_buffer_.empty())
		{
			nav_buffer_.pop();
		}
	}
	else
	{
		nav_running_ = true;
		next_nav_time_ = current_sim_time_;
		if (status_ == ib2_msgs::msg::NavigationStatus::NAV_OFF)
		{
			status_ = ib2_msgs::msg::NavigationStatus::NAV_FUSION;
		}
	}
}

//------------------------------------------------------------------------------
// Action Server: ゴール受付
rclcpp_action::GoalResponse nav_plugin::Nav::handleGoal(
	const rclcpp_action::GoalUUID &/*uuid*/,
	std::shared_ptr<const NavigationStartUp::Goal> /*goal*/)
{
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

//------------------------------------------------------------------------------
// Action Server: キャンセル
rclcpp_action::CancelResponse nav_plugin::Nav::handleCancel(
	const std::shared_ptr<GoalHandleNSU> /*goal_handle*/)
{
	return rclcpp_action::CancelResponse::ACCEPT;
}

//------------------------------------------------------------------------------
// Action Server: 受付後の処理
void nav_plugin::Nav::handleAccepted(const std::shared_ptr<GoalHandleNSU> goal_handle)
{
	auto goal = goal_handle->get_goal();

	if (goal->command == NavigationStartUp::Goal::ON)
	{
		switchPowerInternal(ib2_msgs::msg::PowerStatus::ON);
		double delay = gz::math::Rand::DblUniform(1.0, 2.5);
		pending_action_ = PendingAction{
			goal_handle,
			current_sim_time_ + delay,
			NavigationStartUp::Result::ON_READY
		};
	}
	else if (goal->command == NavigationStartUp::Goal::OFF)
	{
		switchPowerInternal(ib2_msgs::msg::PowerStatus::OFF);
		double delay = gz::math::Rand::DblUniform(0.5, 1.5);
		pending_action_ = PendingAction{
			goal_handle,
			current_sim_time_ + delay,
			NavigationStartUp::Result::OFF
		};
	}
	else
	{
		// ABORTED - return immediately
		auto result = std::make_shared<NavigationStartUp::Result>();
		result->type = NavigationStartUp::Result::ABORTED;
		auto sim_ns = static_cast<int64_t>(current_sim_time_ * 1e9);
		result->stamp.sec     = static_cast<int32_t>(sim_ns / 1000000000);
		result->stamp.nanosec = static_cast<uint32_t>(sim_ns % 1000000000);
		goal_handle->succeed(result);
	}
}

//------------------------------------------------------------------------------
// Model取得
void nav_plugin::Nav::getModels(gz::sim::EntityComponentManager &_ecm)
{
	if (iss_model_ == gz::sim::kNullEntity && !iss_name_.empty())
	{
		iss_model_ = _ecm.EntityByComponents(
			gz::sim::components::Name(iss_name_),
			gz::sim::components::Model());
		if (iss_model_ != gz::sim::kNullEntity)
		{
			gz::sim::Model model(iss_model_);
			auto links = model.Links(_ecm);
			if (!links.empty())
			{
				iss_link_ = links[0];
			}
		}
	}

	if (ib2_model_ == gz::sim::kNullEntity && !ib2_name_.empty())
	{
		ib2_model_ = _ecm.EntityByComponents(
			gz::sim::components::Name(ib2_name_),
			gz::sim::components::Model());
		if (ib2_model_ != gz::sim::kNullEntity)
		{
			gz::sim::Model model(ib2_model_);
			auto links = model.Links(_ecm);
			if (!links.empty())
			{
				ib2_link_ = links[0];
			}
		}
	}
}

//------------------------------------------------------------------------------
// 加速度・姿勢レートの加算 (毎物理ステップ呼ばれる)
void nav_plugin::Nav::sumAcclAndAttRate(gz::sim::EntityComponentManager &_ecm)
{
	if (ib2_link_ == gz::sim::kNullEntity)
		return;

	gz::sim::Link ib2_lnk(ib2_link_);

	// Body-frame angular velocity
	auto worldAngVel = ib2_lnk.WorldAngularVelocity(_ecm);
	auto worldPose   = ib2_lnk.WorldPose(_ecm);
	if (!worldAngVel || !worldPose)
		return;

	gz::math::Vector3d w = worldPose->Rot().RotateVectorReverse(*worldAngVel);

	// Body-frame linear acceleration (= applied force / mass)
	auto worldLinAccel = ib2_lnk.WorldLinearAcceleration(_ecm);
	gz::math::Vector3d ab = gz::math::Vector3d::Zero;
	if (worldLinAccel)
	{
		ab = worldPose->Rot().RotateVectorReverse(*worldLinAccel);
	}

	delta_v_     += ab;
	delta_angle_ += w;
	accum_counter_++;
}

//------------------------------------------------------------------------------
// 航法コールバック (タイマー模擬)
void nav_plugin::Nav::navCallBack(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	// Get True Navigation
	ib2_msgs::msg::Navigation nav_msgs_w = getTrueNavigation(_info, _ecm);

	// Transform World to Home Coordination
	ib2_msgs::msg::Navigation nav_msgs_h = transformWtoH(nav_msgs_w, _ecm);

	// Publish True Navigation
	pub_true_nav_->publish(nav_msgs_h);

	// Publish True Attitude
	pub_true_att_->publish(makeAttMsgFromNavMsg(nav_msgs_h));

	// Add Error
	nav_msgs_h = addError(nav_msgs_h);
	if (invalid_nav_)
	{
		nav_msgs_h.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
	}

	// Navigation Delay Model
	nav_buffer_.push(nav_msgs_h);
	int size = static_cast<int>(bias_cnt_ * delay_ + 0.5 + EPS);
	if (static_cast<int>(nav_buffer_.size()) >= size + 1)
	{
		pub_nav_->publish(nav_buffer_.front());
		pub_att_->publish(makeAttMsgFromNavMsg(nav_buffer_.front()));
		nav_buffer_.pop();
	}
}

//------------------------------------------------------------------------------
// 航法機能ステータスコールバック (タイマー模擬)
void nav_plugin::Nav::sensorFusionStatusCallBack(const gz::sim::UpdateInfo &/*_info*/)
{
	ib2_msgs::msg::NavigationStatus sensor_fusion_status;
	sensor_fusion_status.status = status_;
	sensor_fusion_status.marker = false;
	pub_sensor_fusion_status_->publish(sensor_fusion_status);
}

//------------------------------------------------------------------------------
// ロボットの航法値真値を取得する
ib2_msgs::msg::Navigation nav_plugin::Nav::getTrueNavigation(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	gz::sim::Link ib2_lnk(ib2_link_);

	auto pi = ib2_lnk.WorldPose(_ecm);
	auto vi = ib2_lnk.WorldLinearVelocity(_ecm);

	auto w  = (accum_counter_ > 0) ? (delta_angle_ / accum_counter_) : gz::math::Vector3d::Zero;
	auto ab = (accum_counter_ > 0) ? (delta_v_     / accum_counter_) : gz::math::Vector3d::Zero;

	// Sim time + offset
	double stamp_time = std::chrono::duration<double>(_info.simTime).count() + tnav_offset_;
	auto stamp_ns = static_cast<int64_t>(stamp_time * 1e9);

	ib2_msgs::msg::Navigation nav_msgs;
	nav_msgs.pose.header.stamp.sec     = static_cast<int32_t>(stamp_ns / 1000000000);
	nav_msgs.pose.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
	nav_msgs.pose.header.frame_id      = FRAME_ISS;

	if (pi)
	{
		nav_msgs.pose.pose.position.x    = pi->Pos().X();
		nav_msgs.pose.pose.position.y    = pi->Pos().Y();
		nav_msgs.pose.pose.position.z    = pi->Pos().Z();
		nav_msgs.pose.pose.orientation.x = pi->Rot().X();
		nav_msgs.pose.pose.orientation.y = pi->Rot().Y();
		nav_msgs.pose.pose.orientation.z = pi->Rot().Z();
		nav_msgs.pose.pose.orientation.w = pi->Rot().W();
	}

	if (vi)
	{
		nav_msgs.twist.linear.x = vi->X();
		nav_msgs.twist.linear.y = vi->Y();
		nav_msgs.twist.linear.z = vi->Z();
	}

	nav_msgs.twist.angular.x = w.X();
	nav_msgs.twist.angular.y = w.Y();
	nav_msgs.twist.angular.z = w.Z();
	nav_msgs.a.x             = ab.X();
	nav_msgs.a.y             = ab.Y();
	nav_msgs.a.z             = ab.Z();
	nav_msgs.status.status   = status_;

	// Reset accumulators
	delta_angle_   = gz::math::Vector3d::Zero;
	delta_v_       = gz::math::Vector3d::Zero;
	accum_counter_ = 0;

	return nav_msgs;
}

//------------------------------------------------------------------------------
// World座標系からドッキングステーション(ホーム)座標系への変換
ib2_msgs::msg::Navigation nav_plugin::Nav::transformWtoH(
	const ib2_msgs::msg::Navigation& nav_msgs,
	gz::sim::EntityComponentManager &_ecm)
{
	if (iss_link_ == gz::sim::kNullEntity)
		return nav_msgs;

	gz::sim::Link iss_lnk(iss_link_);
	auto iss_pose = iss_lnk.WorldPose(_ecm);
	if (!iss_pose)
		return nav_msgs;

	auto iss_cg = gz::math::Vector3d(iss_pose->Pos().X(), iss_pose->Pos().Y(), iss_pose->Pos().Z());
	coord_transformer_.set(iss_cg, jpm_pos_, jpm_att_, ds_pos_, ds_att_);

	auto rn = nav_msgs.pose.pose.position;
	auto qn = nav_msgs.pose.pose.orientation;
	auto vn = nav_msgs.twist.linear;

	// Position
	auto iss_qtn   = gz::math::Quaterniond(iss_pose->Rot().W(), iss_pose->Rot().X(),
	                                        iss_pose->Rot().Y(), iss_pose->Rot().Z());
	auto world_pos = gz::math::Vector3d(rn.x, rn.y, rn.z);
	auto ds_pos    = coord_transformer_.getDsPosFromWorld(world_pos, iss_qtn);

	// Velocity
	auto worldAngVel = iss_lnk.WorldAngularVelocity(_ecm);
	gz::math::Vector3d iss_w = gz::math::Vector3d::Zero;
	if (worldAngVel)
	{
		iss_w = iss_pose->Rot().RotateVectorReverse(*worldAngVel);
	}
	auto world_vel = gz::math::Vector3d(vn.x, vn.y, vn.z);
	auto ds_vel    = coord_transformer_.getDsVelFromWorld(world_pos, world_vel, iss_qtn, iss_w);

	// Quaternion
	auto world_qtn = gz::math::Quaterniond(qn.w, qn.x, qn.y, qn.z);
	auto ds_qtn    = coord_transformer_.getDsQtnFromWorld(world_qtn, iss_qtn);

	// Navigation Message
	ib2_msgs::msg::Navigation nav_msgs_h;
	nav_msgs_h.pose.header             = nav_msgs.pose.header;
	nav_msgs_h.pose.pose.position.x    = ds_pos.X();
	nav_msgs_h.pose.pose.position.y    = ds_pos.Y();
	nav_msgs_h.pose.pose.position.z    = ds_pos.Z();
	nav_msgs_h.pose.pose.orientation.x = ds_qtn.X();
	nav_msgs_h.pose.pose.orientation.y = ds_qtn.Y();
	nav_msgs_h.pose.pose.orientation.z = ds_qtn.Z();
	nav_msgs_h.pose.pose.orientation.w = ds_qtn.W();
	nav_msgs_h.twist.linear.x          = ds_vel.X();
	nav_msgs_h.twist.linear.y          = ds_vel.Y();
	nav_msgs_h.twist.linear.z          = ds_vel.Z();
	nav_msgs_h.twist.angular.x         = nav_msgs.twist.angular.x;
	nav_msgs_h.twist.angular.y         = nav_msgs.twist.angular.y;
	nav_msgs_h.twist.angular.z         = nav_msgs.twist.angular.z;
	nav_msgs_h.a.x                     = nav_msgs.a.x;
	nav_msgs_h.a.y                     = nav_msgs.a.y;
	nav_msgs_h.a.z                     = nav_msgs.a.z;
	nav_msgs_h.status.status           = nav_msgs.status.status;

	return nav_msgs_h;
}

//------------------------------------------------------------------------------
// 航法値に誤差を付加する
ib2_msgs::msg::Navigation nav_plugin::Nav::addError(const ib2_msgs::msg::Navigation& nav_msgs)
{
	ib2_msgs::msg::Navigation nav_msgs_e = nav_msgs;
	NavError nav_error = getNavError();

	// Add noise
	nav_msgs_e.pose.pose.position.x += nav_error.pos.X();
	nav_msgs_e.pose.pose.position.y += nav_error.pos.Y();
	nav_msgs_e.pose.pose.position.z += nav_error.pos.Z();
	nav_msgs_e.twist.linear.x      += nav_error.vel.X();
	nav_msgs_e.twist.linear.y      += nav_error.vel.Y();
	nav_msgs_e.twist.linear.z      += nav_error.vel.Z();
	nav_msgs_e.a.x                 += nav_error.acc.X();
	nav_msgs_e.a.y                 += nav_error.acc.Y();
	nav_msgs_e.a.z                 += nav_error.acc.Z();

	auto qne = nav_msgs_e.pose.pose.orientation;
	gz::math::Quaterniond q(qne.w, qne.x, qne.y, qne.z);
	gz::math::Vector3d euler = q.Euler() + nav_error.rot * DEG2RAD;

	q.SetFromEuler(euler);
	nav_msgs_e.pose.pose.orientation.x = q.X();
	nav_msgs_e.pose.pose.orientation.y = q.Y();
	nav_msgs_e.pose.pose.orientation.z = q.Z();
	nav_msgs_e.pose.pose.orientation.w = q.W();

	nav_msgs_e.twist.angular.x += nav_error.w.X() * DEG2RAD;
	nav_msgs_e.twist.angular.y += nav_error.w.Y() * DEG2RAD;
	nav_msgs_e.twist.angular.z += nav_error.w.Z() * DEG2RAD;

	return nav_msgs_e;
}

//------------------------------------------------------------------------------
// 制御周期の変動を模擬する
double nav_plugin::Nav::controlFreqFluctuation(double sim_time)
{
	double wg   = gz::math::Rand::DblNormal(bias_cnt_, rand_cnt_);
	double freq = gain_cnt_ * sin(2.0 * M_PI * freq_cnt_ * sim_time) + wg;

	assert(std::abs(freq) > EPS);
	double duration = 1.0 / freq;

	return duration;
}

//------------------------------------------------------------------------------
// NavigationメッセージからAttitudeメッセージを作成
sim_msgs::msg::Attitude nav_plugin::Nav::makeAttMsgFromNavMsg(
	const ib2_msgs::msg::Navigation& nav_msgs)
{
	auto qn = nav_msgs.pose.pose.orientation;
	auto wn = nav_msgs.twist.angular;

	sim_msgs::msg::Attitude att_msgs;
	att_msgs.stamp = nav_msgs.pose.header.stamp;
	att_msgs.q     = qn;
	att_msgs.w.x   = wn.x * RAD2DEG;
	att_msgs.w.y   = wn.y * RAD2DEG;
	att_msgs.w.z   = wn.z * RAD2DEG;

	gz::math::Quaterniond quat(qn.w, qn.x, qn.y, qn.z);
	gz::math::Vector3d eulr(quat.Euler());
	att_msgs.euler.x = eulr.X() * RAD2DEG;
	att_msgs.euler.y = eulr.Y() * RAD2DEG;
	att_msgs.euler.z = eulr.Z() * RAD2DEG;

	return att_msgs;
}

//------------------------------------------------------------------------------
// 航法誤差を取得する
nav_plugin::Nav::NavError nav_plugin::Nav::getNavError()
{
	NavError nav_error;

	// From CSV
	if (error_source_csv_)
	{
		std::string line;
		getline(ifs_, line);

		if (ifs_.eof())
		{
			RCLCPP_WARN(ros_node_->get_logger(), "End Of File of %s", csv_file_name_.c_str());
			RCLCPP_WARN(ros_node_->get_logger(), "Reset to the Top Of the File");
			ifs_.clear();
			ifs_.seekg(0, std::ios_base::beg);
			getline(ifs_, line);
		}

		// skip comment line
		if (line.find_first_of('#') == 0)
		{
			getline(ifs_, line);  // Read unit line
			getline(ifs_, line);  // Read error value
		}

		double val[15];
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		for (int i = 0; i < 15; i++)
		{
			iss >> val[i];
		}
		nav_error.pos.Set(val[0],  val[1],  val[2]);
		nav_error.vel.Set(val[3],  val[4],  val[5]);
		nav_error.acc.Set(val[6],  val[7],  val[8]);
		nav_error.rot.Set(val[9],  val[10], val[11]);
		nav_error.w.Set  (val[12], val[13], val[14]);
	}
	// From Random Number
	else
	{
		nav_error.pos.Set(
			gz::math::Rand::DblNormal(bias_p_.X(), rand_p_.X()),
			gz::math::Rand::DblNormal(bias_p_.Y(), rand_p_.Y()),
			gz::math::Rand::DblNormal(bias_p_.Z(), rand_p_.Z()));
		nav_error.vel.Set(
			gz::math::Rand::DblNormal(bias_v_.X(), rand_v_.X()),
			gz::math::Rand::DblNormal(bias_v_.Y(), rand_v_.Y()),
			gz::math::Rand::DblNormal(bias_v_.Z(), rand_v_.Z()));
		nav_error.acc.Set(
			gz::math::Rand::DblNormal(bias_a_.X(), rand_a_.X()),
			gz::math::Rand::DblNormal(bias_a_.Y(), rand_a_.Y()),
			gz::math::Rand::DblNormal(bias_a_.Z(), rand_a_.Z()));
		nav_error.rot.Set(
			gz::math::Rand::DblNormal(bias_r_.X(), rand_r_.X()),
			gz::math::Rand::DblNormal(bias_r_.Y(), rand_r_.Y()),
			gz::math::Rand::DblNormal(bias_r_.Z(), rand_r_.Z()));
		nav_error.w.Set(
			gz::math::Rand::DblNormal(bias_w_.X(), rand_w_.X()),
			gz::math::Rand::DblNormal(bias_w_.Y(), rand_w_.Y()),
			gz::math::Rand::DblNormal(bias_w_.Z(), rand_w_.Z()));
	}

	return nav_error;
}

//------------------------------------------------------------------------------
// 航法誤差CSVファイルオープン
void nav_plugin::Nav::openCSVFile()
{
	if (!error_source_csv_ || ifs_.is_open())
	{
		return;
	}

	ifs_.open(plugin_path_ + csv_file_name_, std::ios::in);

	if (ifs_.fail() || csv_file_name_.empty())
	{
		error_source_csv_ = false;
		RCLCPP_INFO(ros_node_->get_logger(),
			"Navigation Error will be added by generating random number");
	}
}

//------------------------------------------------------------------------------
// gz-simのシステムプラグインとして登録
GZ_ADD_PLUGIN(
	nav_plugin::Nav,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(nav_plugin::Nav, "nav::Nav")

// End Of File -----------------------------------------------------------------
