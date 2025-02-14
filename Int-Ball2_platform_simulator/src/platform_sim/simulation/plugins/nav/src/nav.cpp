
#include "nav/nav.h"

#include <limits>

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 単位変換係数RAD->DEG */
	const double RAD2DEG(180.0 / M_PI);

	/** 微小値 */
	const double EPS(1.0E-10);

	/** navプラグインのパス */
	std::string plugin_path;

	/* 航法誤差CSVファイルストリーム */
	std::ifstream ifs;

	const std::string FRAME_ISS("iss_body");

}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Nav::Nav() :
accum_counter_(0),
delta_v_(ignition::math::Vector3d::Zero),
delta_angle_(ignition::math::Vector3d::Zero)
{}

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Nav::~Nav()
{
	if(ifs.is_open())
	{
		ifs.close();
	}
}

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Nav::Load(physics::WorldPtr world, sdf::ElementPtr)
{

	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "nav",
		ros::init_options::NoSigintHandler);
	}

	// Store the World Pointer
	world_      = world;

	// Create ROS node.
	nh_         = ros::NodeHandle("nav");

	// Get Plugin Path
	plugin_path = ros::package::getPath("nav") + "/";

	// Get Parameters
	if(getParameter() != 0)
	{
		return;
	}

	// Open Nav Error
	openCSVFile();

	// Get Control Duration[s]
	double cnt_duration = controlFreqFluctuation();

	// Timer for callback
	timer_            		    = nh_.createTimer(ros::Duration(cnt_duration), &Nav::navCallBack, this, true, initial_nav_on_);
	timer_sensor_fusion_status_ = nh_.createTimer(ros::Duration(cnt_duration), &Nav::sensorFusionStatusCallBack, this, true, true);

	// Create a Navigation topic, and publish it.
	pub_nav_                   = nh_.advertise<ib2_msgs::Navigation>("/sensor_fusion/navigation", 1);
	pub_sensor_fusion_status_  = nh_.advertise<ib2_msgs::NavigationStatus>("/sensor_fusion/navigation_status", 1);

	// Create a Attitude topic, and publish it.
	pub_att_          = nh_.advertise<sim_msgs::Attitude>("/nav/attitude", 1);

	// Create a True Navigation topic, and publish it.
	pub_true_nav_     = nh_.advertise<ib2_msgs::Navigation>("/nav/true/navigation", 1);

	// Create a True Attitude topic, and publish it.
	pub_true_att_     = nh_.advertise<sim_msgs::Attitude>("/nav/true/attitude", 1);
	
	// Subscribers
	sub_status_       = nh_.subscribe("/nav/status", 5, &Nav::statusCallback, this);
	sub_time_offset_  = nh_.subscribe("/nav/time_offset", 5, &Nav::offsetCallback, this);

	// Nav Parameter Update Server
	nav_param_server_ = nh_.advertiseService("/sim/nav/update_params", &Nav::updateParameter, this);

	// Nav Marker Correction Server
	marker_correction_server_ = nh_.advertiseService("/sensor_fusion/marker_correction", &Nav::markerCorrection, this);

	// Nav ON/OFF Service Server
	switch_power_server_      = nh_.advertiseService("/nav/switch_power", &Nav::switchPower, this);

	// Nav ON/OFF Action Server
	navigation_start_up_.reset(new actionlib::SimpleActionServer<ib2_msgs::NavigationStartUpAction>(
		nh_, "/sensor_fusion/navigation_start_up", boost::bind(&gazebo::Nav::navigationStartUpCallback, this, _1), false));
	navigation_start_up_->start();

	// Set callback to get current accl and att. rate
	update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Nav::sumAcclAndAttRate, this));
	
	status_      = initial_nav_on_ ? ib2_msgs::NavigationStatus::NAV_FUSION : ib2_msgs::NavigationStatus::NAV_OFF;
	tnav_offset_ = ros::Duration(0.);
	invalid_nav_ = false;
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void gazebo::Nav::navCallBack(const ros::TimerEvent&)
{
	// Get True Navigation
	ib2_msgs::Navigation nav_msgs_w = getTrueNavigation();

	// Transform World Coordination to Home Coordination 
	ib2_msgs::Navigation nav_msgs_h = transformWtoH(nav_msgs_w);

	// Publish True Navigation Message
	pub_true_nav_.publish(nav_msgs_h);

	// Publish True Attitude Message
	pub_true_att_.publish(makeAttMsgFromNavMsg(nav_msgs_h));

	// Add Error
	nav_msgs_h = addError(nav_msgs_h);
	if (invalid_nav_)
	{
		nav_msgs_h.pose.pose.position.z = std::numeric_limits<double>::quiet_NaN();
	}

	// Navigation Delay Model
	nav_buffer_.push(nav_msgs_h);
	int size = static_cast<int>(bias_cnt_ * delay_ + 0.5 + EPS);
	if(nav_buffer_.size() >= size + 1)
	{
#ifdef NAV_DEBUG
	gzmsg << "Navigation Delay is " << ros::Time::now() - nav_buffer_.front().pose.header.stamp << "[s]\n";
#endif
		// Publish Navigation Message
		pub_nav_.publish(nav_buffer_.front());
		// Publish Attitude Message
		pub_att_.publish(makeAttMsgFromNavMsg(nav_buffer_.front()));

		nav_buffer_.pop();
	}

	// Get Control Duration[s]
	double cnt_duration = controlFreqFluctuation();

	// Timer for callback
	timer_ = nh_.createTimer(ros::Duration(cnt_duration), &Nav::navCallBack, this, true, true);
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void gazebo::Nav::sensorFusionStatusCallBack(const ros::TimerEvent&)
{
	// Publish Navigation Status Message
	ib2_msgs::NavigationStatus sensor_fusion_status;
	sensor_fusion_status.status = status_;
	sensor_fusion_status.marker = false;
	pub_sensor_fusion_status_.publish(sensor_fusion_status);

	// Get Control Duration[s]
	double cnt_duration = controlFreqFluctuation();

	// Timer for callback
	timer_sensor_fusion_status_ = nh_.createTimer(ros::Duration(cnt_duration), &Nav::sensorFusionStatusCallBack, this, true, true);
}
//------------------------------------------------------------------------------
// 航法ステータスのサブスクライバのコールバック関数
void gazebo::Nav::statusCallback(const std_msgs::Int32& status)
{
	if (status.data >= 0)
	{
		status_ = static_cast<uint8_t>(status.data);
		invalid_nav_ = false;
	}
	else
	{
		invalid_nav_ = true;
		status_ = static_cast<uint8_t>(-status.data);
	}
}

//------------------------------------------------------------------------------
// 航法時刻オフセットのサブスクライバのコールバック関数
void gazebo::Nav::offsetCallback(const std_msgs::Float64& offset)
{
	tnav_offset_ = ros::Duration(offset.data);
}

//------------------------------------------------------------------------------
// ロボットの航法値真値を取得する
ib2_msgs::Navigation gazebo::Nav::getTrueNavigation()
{
	auto pi(ib2_link_[0]->WorldCoGPose());			// Position and Rotaion(Quaternion)
	auto vi(ib2_link_[0]->WorldCoGLinearVel());		// Velocity(in World Frame)
	auto w (accum_counter_ > 0 ? delta_angle_ / accum_counter_ : ignition::math::Vector3d::Zero);
	auto ab(accum_counter_ > 0 ? delta_v_     / accum_counter_ : ignition::math::Vector3d::Zero);

	static uint32_t seq(0);
	ib2_msgs::Navigation nav_msgs;
	nav_msgs.pose.header.seq         = ++seq;
	nav_msgs.pose.header.stamp       = ros::Time::now() + tnav_offset_;
	nav_msgs.pose.header.frame_id    = FRAME_ISS;
	nav_msgs.pose.pose.position.x    = pi.Pos().X();
	nav_msgs.pose.pose.position.y    = pi.Pos().Y();
	nav_msgs.pose.pose.position.z    = pi.Pos().Z();
	nav_msgs.pose.pose.orientation.x = pi.Rot().X();
	nav_msgs.pose.pose.orientation.y = pi.Rot().Y();
	nav_msgs.pose.pose.orientation.z = pi.Rot().Z();
	nav_msgs.pose.pose.orientation.w = pi.Rot().W();
	nav_msgs.twist.linear.x          = vi.X();
	nav_msgs.twist.linear.y          = vi.Y();
	nav_msgs.twist.linear.z          = vi.Z();
	nav_msgs.twist.angular.x         = w.X();
	nav_msgs.twist.angular.y         = w.Y();
	nav_msgs.twist.angular.z         = w.Z();
	nav_msgs.a.x                     = ab.X();
	nav_msgs.a.y                     = ab.Y();
	nav_msgs.a.z                     = ab.Z();
	nav_msgs.status.status           = status_;

	delta_angle_   = ignition::math::Vector3d::Zero;
	delta_v_       = ignition::math::Vector3d::Zero;
	accum_counter_ = 0;

	return nav_msgs;
}

//------------------------------------------------------------------------------
// World(慣性)座標系からドッキングステーション(ホーム)座標系への座標変換
ib2_msgs::Navigation gazebo::Nav::transformWtoH(const ib2_msgs::Navigation& nav_msgs)
{
	auto iss_pose  = iss_link_[0]->WorldCoGPose();
	auto iss_cg    = ignition::math::Vector3d(iss_pose.Pos().X(), iss_pose.Pos().Y(), iss_pose.Pos().Z());
	coord_transformer_.set(iss_cg, jpm_pos_, jpm_att_, ds_pos_, ds_att_);

	auto rn(nav_msgs.pose.pose.position);
	auto qn(nav_msgs.pose.pose.orientation);
	auto vn(nav_msgs.twist.linear);

	// Position
	auto iss_qtn   = ignition::math::Quaterniond(iss_pose.Rot().W(), iss_pose.Rot().X(), iss_pose.Rot().Y(), iss_pose.Rot().Z());
	auto world_pos = ignition::math::Vector3d(rn.x, rn.y, rn.z);
	auto ds_pos    = coord_transformer_.getDsPosFromWorld(world_pos, iss_qtn);

	// Velocity
	auto iss_w     = iss_link_[0]->RelativeAngularVel();
	auto world_vel = ignition::math::Vector3d(vn.x, vn.y, vn.z);
	auto ds_vel    = coord_transformer_.getDsVelFromWorld(world_pos, world_vel, iss_qtn, iss_w);

	// Quaternion
	auto world_qtn = ignition::math::Quaterniond(qn.w, qn.x, qn.y, qn.z);
	auto ds_qtn    = coord_transformer_.getDsQtnFromWorld(world_qtn, iss_qtn);
	

	// Navigation Message
	ib2_msgs::Navigation nav_msgs_h;
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
// ROS Parameter Serverからパラメータを取得
int gazebo::Nav::getParameter()
{
	int ret = 0;
	double x1, y1, z1;
	double x2, y2, z2;

	// Model Name
	if(!nh_.getParam("/model_name/iss_name", iss_name_))
	{
		gzerr << "Cannot Get /model_name/iss_name in nav plugin \n";
		ret   = ret | 0x0001;
	}
	if(!nh_.getParam("/model_name/ib2_name", ib2_name_))
	{
		gzerr << "Cannot Get /model_name/ib2_name in nav plugin \n";
		ret   = ret | 0x0002;
	}

	// JPM Pose
	if(
		!nh_.getParam("/jpm_pose/pos/x", x1) ||
		!nh_.getParam("/jpm_pose/pos/y", y1) ||
		!nh_.getParam("/jpm_pose/pos/z", z1)
	)
	{
		gzerr << "Cannot Get /jpm_pose/pos/ in nav plugin \n";
		ret   = ret | 0x0004;
	}
	jpm_pos_.Set(x1, y1, z1);
	if(
		!nh_.getParam("/jpm_pose/att/r", x1) ||
		!nh_.getParam("/jpm_pose/att/p", y1) ||
		!nh_.getParam("/jpm_pose/att/y", z1)
	)
	{
		gzerr << "Cannot Get /jpm_pose/att/ in nav plugin \n";
		ret   = ret | 0x0008;
	}
	jpm_att_.Set(x1, y1, z1);
	jpm_att_ = jpm_att_ * DEG2RAD;

	// DS Pose
	if(
		!nh_.getParam("/ds_pose/pos/x", x1) ||
		!nh_.getParam("/ds_pose/pos/y", y1) ||
		!nh_.getParam("/ds_pose/pos/z", z1)
	)
	{
		gzerr << "Cannot Get /ds_pose/pos/ in nav plugin \n";
		ret   = ret | 0x000F;
	}
	ds_pos_.Set(x1, y1, z1);
	if(
		!nh_.getParam("/ds_pose/att/r", x1) ||
		!nh_.getParam("/ds_pose/att/p", y1) ||
		!nh_.getParam("/ds_pose/att/y", z1)
	)
	{
		gzerr << "Cannot Get /ds_pose/att/ in nav plugin \n";
		ret   = ret | 0x0010;
	}
	ds_att_.Set(x1, y1, z1);
	ds_att_ = ds_att_ * DEG2RAD;

	// Navigation Error Parameter
	if(!nh_.getParam("/nav_parameter/error/error_source_csv/", error_source_csv_))
	{
		gzerr << "Cannot Get /nav_parameter/error/error_source_csv/ in nav plugin \n";
		ret   = ret | 0x0011;
	}

	if(!nh_.getParam("/nav_parameter/error/csv/", csv_file_name_))
	{
		gzerr << "Cannot Get /nav_parameter/error/csv/ in nav plugin \n";
		ret   = ret | 0x0012;
	}

	if(
		!nh_.getParam("/nav_parameter/error/pos/mean/x",   x1) ||
		!nh_.getParam("/nav_parameter/error/pos/mean/y",   y1) ||
		!nh_.getParam("/nav_parameter/error/pos/mean/z",   z1) ||
		!nh_.getParam("/nav_parameter/error/pos/stddev/x", x2) ||
		!nh_.getParam("/nav_parameter/error/pos/stddev/y", y2) ||
		!nh_.getParam("/nav_parameter/error/pos/stddev/z", z2)
	)
	{
		gzerr << "Cannot Get /nav_parameter/error/pos/ in nav plugin \n";
		ret   = ret | 0x0014;
	}
	bias_p_.Set(x1, y1, z1);
	rand_p_.Set(x2, y2, z2);
	if(
		!nh_.getParam("/nav_parameter/error/vel/mean/x",   x1) ||
		!nh_.getParam("/nav_parameter/error/vel/mean/y",   y1) ||
		!nh_.getParam("/nav_parameter/error/vel/mean/z",   z1) ||
		!nh_.getParam("/nav_parameter/error/vel/stddev/x", x2) ||
		!nh_.getParam("/nav_parameter/error/vel/stddev/y", y2) ||
		!nh_.getParam("/nav_parameter/error/vel/stddev/z", z2)
	)
	{
		gzerr << "Cannot Get /nav_parameter/error/vel/ in nav plugin \n";
		ret   = ret | 0x0018;
	}
	bias_v_.Set(x1, y1, z1);
	rand_v_.Set(x2, y2, z2);
	if(
		!nh_.getParam("/nav_parameter/error/acc/mean/x",   x1) ||
		!nh_.getParam("/nav_parameter/error/acc/mean/y",   y1) ||
		!nh_.getParam("/nav_parameter/error/acc/mean/z",   z1) ||
		!nh_.getParam("/nav_parameter/error/acc/stddev/x", x2) ||
		!nh_.getParam("/nav_parameter/error/acc/stddev/y", y2) ||
		!nh_.getParam("/nav_parameter/error/acc/stddev/z", z2)
	)
	{
		gzerr << "Cannot Get /nav_parameter/error/acc/ in nav plugin \n";
		ret   = ret | 0x001F;
	}
	bias_a_.Set(x1, y1, z1);
	rand_a_.Set(x2, y2, z2);
	if(
		!nh_.getParam("/nav_parameter/error/att/mean/x",   x1) ||
		!nh_.getParam("/nav_parameter/error/att/mean/y",   y1) ||
		!nh_.getParam("/nav_parameter/error/att/mean/z",   z1) ||
		!nh_.getParam("/nav_parameter/error/att/stddev/x", x2) ||
		!nh_.getParam("/nav_parameter/error/att/stddev/y", y2) ||
		!nh_.getParam("/nav_parameter/error/att/stddev/z", z2)
	)
	{
		gzerr << "Cannot Get /nav_parameter/error/att/ in nav plugin \n";
		ret   = ret | 0x0020;
	}
	bias_r_.Set(x1, y1, z1);
	rand_r_.Set(x2, y2, z2);
	if(
		!nh_.getParam("/nav_parameter/error/att_rate/mean/x",   x1) ||
		!nh_.getParam("/nav_parameter/error/att_rate/mean/y",   y1) ||
		!nh_.getParam("/nav_parameter/error/att_rate/mean/z",   z1) ||
		!nh_.getParam("/nav_parameter/error/att_rate/stddev/x", x2) ||
		!nh_.getParam("/nav_parameter/error/att_rate/stddev/y", y2) ||
		!nh_.getParam("/nav_parameter/error/att_rate/stddev/z", z2)
	)
	{
		gzerr << "Cannot Get /nav_parameter/error/att_rate/ in nav plugin \n";
		ret   = ret | 0x0021;
	}
	bias_w_.Set(x1, y1, z1);
	rand_w_.Set(x2, y2, z2);

	// Control Frequency Fluctuation Parameter
	if(
		!nh_.getParam("/nav_parameter/control/mean",   bias_cnt_) ||
		!nh_.getParam("/nav_parameter/control/stddev", rand_cnt_) ||
		!nh_.getParam("/nav_parameter/control/gain",   gain_cnt_) ||
		!nh_.getParam("/nav_parameter/control/freq",   freq_cnt_)
	)
	{
		gzerr << "Cannot Get /nav_parameter/control/ in nav plugin \n";
		ret   = ret | 0x0022;
	}

	// Navigation Delay Parameter
	if(!nh_.getParam("/nav_parameter/delay", delay_))
	{
		gzerr << "Cannot Get /nav_parameter/delay in nav plugin \n";
		ret   = ret | 0x0024;
	}

	// Initial state of Navigation (true=ON)
	if(!nh_.getParam("/nav_parameter/initial_nav_on", initial_nav_on_))
	{
		gzerr << "Cannot Get /nav_parameter/initial_nav_on in nav plugin \n";
		ret   = ret | 0x0028;
	}

	// 乱数のシード値が設定されている場合は読み込む
	int seed = -1;
	if (nh_.getParam("/sim_common/random_seed", seed))
	{
		if(seed >= 0)
		{
			gazebo::common::Console::msg(__FILE__, __LINE__) << "Set the random seed value " << seed << "\n";
			ignition::math::Rand::Seed(static_cast<unsigned int>(seed));
		}
	}
	else
	{
		gzerr << "Could not read the parameters of \"/sim_common/random_seed\".\n";
		ret = ret | 0x002F;
	}

	return ret;
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool gazebo::Nav::updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res)
{
	res.result = false;

	if(getParameter() != 0)
		return true;

	if(error_source_csv_)
	{
		openCSVFile();
	}

	res.result = true;
	return true;
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool gazebo::Nav::markerCorrection(ib2_msgs::MarkerCorrection::Request& req, ib2_msgs::MarkerCorrection::Response& res)
{
	int marker(0);
	nh_.getParam("/nav_parameter/marker", marker);

	res.stamp = ros::Time::now();
	res.status = marker > 0 ? ib2_msgs::MarkerCorrection::Response::SUCCESS : ib2_msgs::MarkerCorrection::Response::FAILURE_UPDATE;
	return true;
}

//------------------------------------------------------------------------------
// Nav ON/OFF ServerからON/OFFコマンドを取得
bool gazebo::Nav::switchPower
(ib2_msgs::SwitchPower::Request&   req, 
 ib2_msgs::SwitchPower::Response&  res)
{
	if((status_ == ib2_msgs::NavigationStatus::NAV_OFF && req.power.status == ib2_msgs::PowerStatus::OFF) || 
		(status_ == ib2_msgs::NavigationStatus::NAV_FUSION && req.power.status == ib2_msgs::PowerStatus::ON))
	{
		// ステータス変更無し
		res.current_power.status = req.power.status;
		return true;
	}

	if(req.power.status == ib2_msgs::PowerStatus::OFF)
	{
		timer_.stop();
		status_                  = ib2_msgs::NavigationStatus::NAV_OFF;
		res.current_power.status = ib2_msgs::PowerStatus::OFF;

		while(!nav_buffer_.empty())
		{
			nav_buffer_.pop();
		}

		return true;
	}
	
	timer_.start();

	if(status_ == ib2_msgs::NavigationStatus::NAV_OFF)
	{
		status_                  = ib2_msgs::NavigationStatus::NAV_FUSION;
		res.current_power.status = ib2_msgs::PowerStatus::ON;
	}

	return true;
}

// 航法機能アクション（実機固有インタフェース）模擬
void gazebo::Nav::navigationStartUpCallback(const ib2_msgs::NavigationStartUpGoalConstPtr& request)
{
	if(request->command == ib2_msgs::NavigationStartUpGoal::ON)
	{
		// SwitchPower: ON
		ib2_msgs::SwitchPower::Request switch_power_request;
		switch_power_request.power.status = ib2_msgs::PowerStatus::ON;
		ib2_msgs::SwitchPower::Response switch_power_response;
		switchPower(switch_power_request, switch_power_response);

		auto sleep_second = ignition::math::Rand::DblUniform(1.0, 2.5);
		ros::Duration(sleep_second).sleep();

		// Response result
		ib2_msgs::NavigationStartUpResult result;
		result.stamp = ros::Time::now();
		result.type = ib2_msgs::NavigationStartUpResult::ON_READY;
		navigation_start_up_->setSucceeded(result);
	}
	else if(request->command == ib2_msgs::NavigationStartUpGoal::OFF)
	{
		// SwitchPower: OFF
		ib2_msgs::SwitchPower::Request switch_power_request;
		switch_power_request.power.status = ib2_msgs::PowerStatus::OFF;
		ib2_msgs::SwitchPower::Response switch_power_response;
		switchPower(switch_power_request, switch_power_response);

		auto sleep_second = ignition::math::Rand::DblUniform(0.5, 1.5);
		ros::Duration(sleep_second).sleep();

		// Response result
		ib2_msgs::NavigationStartUpResult result;
		result.stamp = ros::Time::now();
		result.type = ib2_msgs::NavigationStartUpResult::OFF;
		navigation_start_up_->setSucceeded(result);
	}
	else
	{
		// Response result (ABORTED)
		ib2_msgs::NavigationStartUpResult result;
		result.stamp = ros::Time::now();
		result.type = ib2_msgs::NavigationStartUpResult::ABORTED;
		navigation_start_up_->setSucceeded(result);
	}
}

//------------------------------------------------------------------------------
// Model取得
void gazebo::Nav::getModels()
{
	if(!iss_model_)
	{
		iss_model_ = world_->ModelByName(iss_name_);
		if(iss_model_)
		{
			iss_link_ = iss_model_->GetLinks();
		}
	}
	if(!ib2_model_)
	{
		ib2_model_ = world_->ModelByName(ib2_name_);
		if(ib2_model_)
		{
			ib2_link_ = ib2_model_->GetLinks();
		}
	}
}

//------------------------------------------------------------------------------
// 航法値に誤差を付加する
ib2_msgs::Navigation gazebo::Nav::addError(const ib2_msgs::Navigation& nav_msgs)
{
	ib2_msgs::Navigation nav_msgs_e = nav_msgs;
	NavError             nav_error  = getNavError();

	// Add noise
	nav_msgs_e.pose.pose.position.x += nav_error.pos.X();
	nav_msgs_e.pose.pose.position.y += nav_error.pos.Y();
	nav_msgs_e.pose.pose.position.z += nav_error.pos.Z();
	nav_msgs_e.twist.linear.x       += nav_error.vel.X();
	nav_msgs_e.twist.linear.y       += nav_error.vel.Y();
	nav_msgs_e.twist.linear.z       += nav_error.vel.Z();
	nav_msgs_e.a.x                  += nav_error.acc.X();
	nav_msgs_e.a.y                  += nav_error.acc.Y();
	nav_msgs_e.a.z                  += nav_error.acc.Z();

	auto             qne(nav_msgs_e.pose.pose.orientation);
	ignition::math::Quaterniond q(qne.w, qne.x, qne.y, qne.z);
	ignition::math::Vector3d    euler = q.Euler() + nav_error.rot * DEG2RAD;

	q.EulerToQuaternion(euler);
	nav_msgs_e.pose.pose.orientation.x = q.X();
	nav_msgs_e.pose.pose.orientation.y = q.Y();
	nav_msgs_e.pose.pose.orientation.z = q.Z();
	nav_msgs_e.pose.pose.orientation.w = q.W();

	nav_msgs_e.twist.angular.x      += nav_error.w.X() * DEG2RAD;
	nav_msgs_e.twist.angular.y      += nav_error.w.Y() * DEG2RAD;
	nav_msgs_e.twist.angular.z      += nav_error.w.Z() * DEG2RAD;

	return nav_msgs_e;
}

//------------------------------------------------------------------------------
// 制御周期の変動を模擬する
double gazebo::Nav::controlFreqFluctuation()
{
	// Fluctuated Control Frequency[Hz]
	gazebo::common::Time t    = world_->SimTime();
	double               wg   = ignition::math::Rand::DblNormal(bias_cnt_, rand_cnt_);
	double               freq = gain_cnt_ * sin(2.0 * M_PI * freq_cnt_ * t.Double()) + wg;
	
	// Convert Freq[Hz] to Duration[s]	
	assert(std::abs(freq > EPS));
	double duration = 1.0 / freq;

	return duration;
}

//------------------------------------------------------------------------------
// NavigationメッセージからAttitudeメッセージを作成する
sim_msgs::Attitude gazebo::Nav::makeAttMsgFromNavMsg(const ib2_msgs::Navigation& nav_msgs)
{
	auto qn(nav_msgs.pose.pose.orientation);
	auto wn(nav_msgs.twist.angular);

	// Make Attitude Message
	sim_msgs::Attitude att_msgs;
	att_msgs.stamp = nav_msgs.pose.header.stamp;
	att_msgs.q     = qn;
	att_msgs.w.x   = wn.x * RAD2DEG;
	att_msgs.w.y   = wn.y * RAD2DEG;
	att_msgs.w.z   = wn.z * RAD2DEG;

	ignition::math::Quaterniond quat(qn.w, qn.x, qn.y, qn.z);
	ignition::math::Vector3d    eulr(quat.Euler());
	att_msgs.euler.x = eulr.X() * RAD2DEG;
	att_msgs.euler.y = eulr.Y() * RAD2DEG;
	att_msgs.euler.z = eulr.Z() * RAD2DEG;

	return att_msgs;
}

//------------------------------------------------------------------------------
// 航法誤差を取得する
gazebo::Nav::NavError gazebo::Nav::getNavError()
{
	NavError nav_error;

	// From CSV
	if(error_source_csv_)
	{
		std::string line;
		getline(ifs, line);

		if(ifs.eof())
		{
			gzwarn << "End Of File of " << csv_file_name_ << "\n";
			gzwarn << "Reset to the Top Of the File\n";
			ifs.clear();
			ifs.seekg(0, std::ios_base::beg);
			getline(ifs, line);
		}

		// skip comment line
		if(line.find_first_of('#') == 0)
		{
			getline(ifs, line);  // Read unit line
			getline(ifs, line);  // Read error value
		}

		double val[15];
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);

		for(int i = 0; i < 15; i++)
		{
			iss >> val[i];
		}
		nav_error.pos.Set(val[0],  val[1],  val[2]);
		nav_error.vel.Set(val[3],  val[4],  val[5]);
		nav_error.acc.Set(val[6],  val[7],  val[8]);
		nav_error.rot.Set(val[9],  val[10], val[11]);
		nav_error.w.Set(val[12],  val[13],  val[14]);
	}

	// From Random Number
	else
	{
		nav_error.pos.Set(
			ignition::math::Rand::DblNormal(bias_p_.X(), rand_p_.X()),
			ignition::math::Rand::DblNormal(bias_p_.Y(), rand_p_.Y()),
			ignition::math::Rand::DblNormal(bias_p_.Z(), rand_p_.Z())
		);
		nav_error.vel.Set(
			ignition::math::Rand::DblNormal(bias_v_.X(), rand_v_.X()),
			ignition::math::Rand::DblNormal(bias_v_.Y(), rand_v_.Y()),
			ignition::math::Rand::DblNormal(bias_v_.Z(), rand_v_.Z())
		);
		nav_error.acc.Set(
			ignition::math::Rand::DblNormal(bias_a_.X(), rand_a_.X()),
			ignition::math::Rand::DblNormal(bias_a_.Y(), rand_a_.Y()),
			ignition::math::Rand::DblNormal(bias_a_.Z(), rand_a_.Z())
		);
		nav_error.rot.Set(
			ignition::math::Rand::DblNormal(bias_r_.X(), rand_r_.X()),
			ignition::math::Rand::DblNormal(bias_r_.Y(), rand_r_.Y()),
			ignition::math::Rand::DblNormal(bias_r_.Z(), rand_r_.Z())
		);
		nav_error.w.Set(
			ignition::math::Rand::DblNormal(bias_w_.X(), rand_w_.X()),
			ignition::math::Rand::DblNormal(bias_w_.Y(), rand_w_.Y()),
			ignition::math::Rand::DblNormal(bias_w_.Z(), rand_w_.Z())
		);
	}

	return nav_error;
}

//------------------------------------------------------------------------------
// 航法誤差CSVファイルオープン
void gazebo::Nav::openCSVFile()
{
	if(!error_source_csv_ || ifs.is_open())
	{
		return;
	}
	
	ifs.open(plugin_path + csv_file_name_, std::ios::in);

	if(ifs.fail() || csv_file_name_ == "")
	{
		error_source_csv_ = false;
		gzmsg << "Navigation Error will be added by generating random number\n";
	}
}

//------------------------------------------------------------------------------
// 加速度・姿勢レートの加算
void gazebo::Nav::sumAcclAndAttRate()
{
	// Get ISS and IB2 model
	getModels();

	auto w (ib2_link_[0]->RelativeAngularVel());	    // Angular Rate
	auto fb(ib2_link_[0]->RelativeForce());			// Applied Force(in Body Frame)
	auto m (ib2_link_[0]->GetInertial()->Mass());	// Mass of this link

	auto ab = (std::abs(m) > EPS)? (fb / m) : (ignition::math::Vector3d::Zero);

	delta_v_     += ab;
	delta_angle_ += w;
	accum_counter_++;
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_WORLD_PLUGIN(gazebo::Nav)

// End Of File -----------------------------------------------------------------


