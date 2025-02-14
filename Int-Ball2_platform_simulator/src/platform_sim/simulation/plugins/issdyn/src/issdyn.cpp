
#include "issdyn/issdyn.h"

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 2PI*/
	const double PI2(2.0 * M_PI);

	/** 本ノード開始フラグ */
	bool start_flag = false;

	/** 本ノード開始時シミュレーション時刻 */
	double start_time = 0;

	const std::string FRAME_ISS("iss_body");
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Issdyn::Issdyn() = default;

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Issdyn::~Issdyn() = default;

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Issdyn::Load(physics::ModelPtr model, sdf::ElementPtr)
{

	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "issdyn",
		ros::init_options::NoSigintHandler);
	}

	// Store the Model Pointer
	model_ = model;

	// Get the first link.
	link_iss_body_  = model_->GetLinks();

	// Create a Navigation topic, and publish it.
	pub_nav_ = nh_.advertise<ib2_msgs::Navigation>("/sim/iss_navigation", 1);

	// Get ISS Attitude Fluctuation Parameter
	getParameter();

	// Set callback to move the link in gazebo cycle
	update_  = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Issdyn::setIssAttitude, this));

	// Timer for callback
	pub_timer_ = nh_.createTimer(ros::Duration(pub_cycle_), &Issdyn::pubIssNav, this, false, true);
}

//------------------------------------------------------------------------------
// URDFからISS姿勢変動パラメータを取得
void gazebo::Issdyn::getParameter()
{
	double x, y, z;

	if(!nh_.getParam("/issdyn_parameter/cycle", pub_cycle_))
	{
		gzerr << "Cannot Get /issdyn_parameter/cycle in issdyn plugin \n";
	}
	if(
		!nh_.getParam("/issdyn_parameter/slope/x", x) ||
		!nh_.getParam("/issdyn_parameter/slope/y", y) ||
		!nh_.getParam("/issdyn_parameter/slope/z", z)
		)
	{
		gzerr << "Cannot Get /issdyn_parameter/slope in issdyn plugin \n";
	}
	att_bias_slope_.Set(x, y, z);
	att_bias_slope_ = att_bias_slope_ * DEG2RAD;
	if(
		!nh_.getParam("/issdyn_parameter/gain/x", x) ||
		!nh_.getParam("/issdyn_parameter/gain/y", y) ||
		!nh_.getParam("/issdyn_parameter/gain/z", z)
		)
	{
		gzerr << "Cannot Get /issdyn_parameter/gain in issdyn plugin \n";
	}
	att_fluc_gain_.Set(x, y, z);
	att_fluc_gain_ = att_fluc_gain_ * DEG2RAD;
	if(
		!nh_.getParam("/issdyn_parameter/freq/x", x) ||
		!nh_.getParam("/issdyn_parameter/freq/y", y) ||
		!nh_.getParam("/issdyn_parameter/freq/z", z)
		)
	{
		gzerr << "Cannot Get /issdyn_parameter/freq in issdyn plugin \n";
	}
	att_fluc_freq_.Set(x, y, z);
}

//------------------------------------------------------------------------------
// ISSの姿勢変動を設定
void gazebo::Issdyn::setIssAttitude()
{
	ignition::math::Vector3d w;
	gazebo::common::Time  sim_time = model_->GetWorld()->SimTime();

	if(start_flag)
	{
		double t = sim_time.Double() - start_time;
		w.X()    = PI2 * att_fluc_freq_.X() * att_fluc_gain_.X() * cos(PI2 * att_fluc_freq_.X() * t) + att_bias_slope_.X();
		w.Y()    = PI2 * att_fluc_freq_.Y() * att_fluc_gain_.Y() * cos(PI2 * att_fluc_freq_.Y() * t) + att_bias_slope_.Y();
		w.Z()    = PI2 * att_fluc_freq_.Z() * att_fluc_gain_.Z() * cos(PI2 * att_fluc_freq_.Z() * t) + att_bias_slope_.Z();

		link_iss_body_[0]->SetAngularVel(w);
	}
	else
	{
		start_time  = sim_time.Double();
		start_flag  = true;
	}
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void gazebo::Issdyn::pubIssNav(const ros::TimerEvent&)
{
	ros::Time now                   = ros::Time::now();

	auto p_iss (link_iss_body_[0]->WorldCoGPose());
	auto v_iss (link_iss_body_[0]->WorldCoGLinearVel());
	auto wb_iss(link_iss_body_[0]->RelativeAngularVel());

	// Publis ISS Navigation
	static uint32_t seq(0);
	ib2_msgs::Navigation iss_nav;

	iss_nav.pose.header.seq         = ++seq;
	iss_nav.pose.header.stamp       = now;
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
	iss_nav.status.status           = ib2_msgs::NavigationStatus::NAV_FUSION;

	pub_nav_.publish(iss_nav);
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_MODEL_PLUGIN(gazebo::Issdyn)

// End Of File -----------------------------------------------------------------


