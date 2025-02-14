
#include "thr/thr.h"

namespace
{
	/** 微小値 */
	const double EPS(1.0E-10);
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Thr::Thr():
	debug_   (false),
	coeff_a_ {0.0, 0.0, 0.0},
	coeff_b_ {0.0, 0.0, 0.0}
	{}

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Thr::~Thr() = default;

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Thr::Load(physics::ModelPtr model, sdf::ElementPtr)
{

	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "thr",
				  ros::init_options::NoSigintHandler);
	}

	// Create ROS node.
	nh_ = ros::NodeHandle("thr");


	// Store the model pointer
	model_ = model;

	// Get the first link.
	link_  = model_->GetLinks();

	// Thr Parameter Update Server
	thr_param_server_ = nh_.advertiseService("/sim/thr/update_params", &Thr::updateParameter, this);

	// Get Parameters by rosparam
	getParameter();

	// Set callback to add force and torque in gazebo cycle
	update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Thr::addForceAndTorque, this));

	// Create a name topic, and subscribe to it.
	if(debug_)
	{
		sub_wrench_ = nh_.subscribe("/ctl/wrench", 1, &Thr::setCtlCmd,  this);
	}
	else
	{
		sub_duty_   = nh_.subscribe("/prop/status", 1, &Thr::subFanDuty, this);

		// Create a force of each fan topic, and publish it.	
		pub_fan_force_ = nh_.advertise<std_msgs::Float64MultiArray>("/thr/fan_force", 1);
	}

	f_.resize(fan_num_, 0.);
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータ取得
void gazebo::Thr::getParameter()
{
	double x, y, z;

	if(!nh_.getParam("/thr_parameter/debug", debug_))
	{
		gzerr << "Cannot Get /thr_parameter/debug in thr plugin \n";
	}

	if(
		!nh_.getParam("/robot_mass_property/cg/x", x) || 
		!nh_.getParam("/robot_mass_property/cg/y", y) || 
		!nh_.getParam("/robot_mass_property/cg/z", z)
		)
	{
		gzerr << "Cannot Get /robot_mass_property/cg in thr plugin \n";
	}
	cg_.Set(x, y, z);

	if(!nh_.getParam("/thr_parameter/fan_num", fan_num_))
	{
		gzerr << "Cannot Get /thr_parameter/fan_num in thr plugin \n";
	}
	
	// Mounting Position and Thrust Vector of Fans
	for(int i = 0; i < fan_num_; i++)
	{
		ignition::math::Vector3d fp;
		ignition::math::Vector3d tv;
		double                   tf;
		double                   sd;
		double                   k;
		double                   kprop;

		std::string fan = "/thr_parameter/fan";
		fan            += std::to_string(i + 1);

		if(
			!nh_.getParam(fan + "/pos/x", x) || 
			!nh_.getParam(fan + "/pos/y", y) || 
			!nh_.getParam(fan + "/pos/z", z)
		)
		{
			gzerr << "Cannot Get " + fan + "/pos in thr plugin \n";
		}
		fp.Set(x, y, z);
		fan_pos_.push_back(fp);

		if(
			!nh_.getParam(fan + "/vec/x", x) || 
			!nh_.getParam(fan + "/vec/y", y) || 
			!nh_.getParam(fan + "/vec/z", z)
		)
		{
			gzerr << "Cannot Get " + fan + "/vec in thr plugin \n";
		}
		tv.Set(x, y, z);
		fan_frc_vec_.push_back(tv);

		if(!nh_.getParam(fan + "/force", tf))
		{
			gzerr << "Cannot Get " + fan + "/force in thr plugin \n";
		}
		fan_frc_.push_back(tf);

		if(!nh_.getParam(fan + "/stddev", sd))
		{
			gzerr << "Cannot Get " + fan + "/stddev in thr plugin \n";
		}
		stddev_.push_back(sd);

		if(!nh_.getParam(fan + "/kappa", k))
		{
			gzerr << "Cannot Get " + fan + "/kappa in thr plugin \n";
		}
		fan_k_.push_back(k);
		
		if(!nh_.getParam(fan + "/Kprop", kprop))
		{
			gzerr << "Cannot Get " + fan + "/Kprop in thr plugin \n";
		}
		k_prop_.push_back(kprop);

		// Set fan's torque vector(unit)
		ignition::math::Vector3d tarm;
		ignition::math::Vector3d trqv;
		ignition::math::Vector3d dtrq;

		tarm = fan_pos_[i] - cg_;
		trqv = tarm.Cross(fan_frc_vec_[i]);
		dtrq = fan_k_[i] * fan_frc_vec_[i];
		fan_trq_vec_.push_back(trqv + dtrq);
	}

	// 2nd Order Filter Coefficient
	if(
		!nh_.getParam("/thr_parameter/filter/coeff_a/a0", x) || 
		!nh_.getParam("/thr_parameter/filter/coeff_a/a1", y) || 
		!nh_.getParam("/thr_parameter/filter/coeff_a/a2", z)
		)
	{
		gzerr << "Cannot Get /thr_parameter/filter/coeff_a in thr plugin \n";
	}
	coeff_a_.Set(x, y, z);

	if(
		!nh_.getParam("/thr_parameter/filter/coeff_b/b0", x) || 
		!nh_.getParam("/thr_parameter/filter/coeff_b/b1", y) || 
		!nh_.getParam("/thr_parameter/filter/coeff_b/b2", z)
		)
	{
		gzerr << "Cannot Get /thr_parameter/filter/coeff_b in thr plugin \n";
	}
	coeff_b_.Set(x, y, z);

	// 2nd Order Filter Buffer Initialization
	in_.reserve (fan_num_);
	out_.reserve(fan_num_);
	for(int i = 0; i < fan_num_; i++)
	{
		in_.push_back ({0.0, 0.0});
		out_.push_back({0.0, 0.0});
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
	}

	logParameter();
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool gazebo::Thr::updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res)
{
	fan_pos_.clear();
	fan_frc_vec_.clear();
	fan_frc_.clear();
	stddev_.clear();
	fan_k_.clear();
	k_prop_.clear();

	getParameter();

	res.result = true;
	return true;
}


//------------------------------------------------------------------------------
// ファン駆動デューティ比をサブスクライブ
void gazebo::Thr::subFanDuty(const ib2_msgs::FanStatus& msg)
{
	int size = msg.duty.data.size();

	// Check the number of fans
	assert(size == fan_num_);
	
	// Calculate required force
	for(int i = 0; i < size; i++)
	{
		assert(std::abs(k_prop_[i]) > EPS);

		double t(msg.duty.data[i] / k_prop_[i]);
		f_.at(i) = std::min(t * t, fan_frc_[i]);
	}
}

//------------------------------------------------------------------------------
// 制御コマンドを直接Gazeboへ設定
void gazebo::Thr::setCtlCmd(const geometry_msgs::WrenchStamped& ctl)
{
	force_.X()  = ctl.wrench.force.x;
	force_.Y()  = ctl.wrench.force.y;
	force_.Z()  = ctl.wrench.force.z;
	torque_.X() = ctl.wrench.torque.x;
	torque_.Y() = ctl.wrench.torque.y;
	torque_.Z() = ctl.wrench.torque.z;
}

//------------------------------------------------------------------------------
// 力・トルクのGazeboへの設定
void gazebo::Thr::addForceAndTorque()
{
	if(!debug_)
	{
		// Initialize force and torque
		force_.Set();
		torque_.Set();

		// Sum up all fan's force and torque
		int size = static_cast<int>(f_.size());

		std_msgs::Float64MultiArray fanforce;
		fanforce.layout.dim.push_back(std_msgs::MultiArrayDimension());
		fanforce.layout.dim[0].size   = size;
		fanforce.layout.dim[0].stride = 1;
		fanforce.layout.dim[0].label  = "fan_force";
		fanforce.layout.data_offset   = 0;
		fanforce.data.resize(size, 0.);
	
		for(int i = 0; i < size; i++)
		{
			double f_fltrd   = biQuadFilter(coeff_a_, coeff_b_, f_[i], in_[i].data(), out_[i].data(), false);

			double ratio     = stddev_[i] / fan_frc_[i];
			double fanNoise  = ignition::math::Rand::DblNormal(0.0, f_fltrd * ratio);   // 推力立上り時に、ノイズ重畳により推力が負になるのを防ぐため、割合でsigmaを設定する
			fanforce.data[i] = f_fltrd + fanNoise;
			force_           = force_  + fanforce.data[i] * fan_frc_vec_[i];
			torque_          = torque_ + fanforce.data[i] * fan_trq_vec_[i];
		}

		// Publish fan force Message
		pub_fan_force_.publish(fanforce);
	}

	link_[0]->AddRelativeForce(force_);
	link_[0]->AddRelativeTorque(torque_);
}

//------------------------------------------------------------------------------
// 双二次フィルタ
double gazebo::Thr::biQuadFilter(const ignition::math::Vector3d& a, const ignition::math::Vector3d& b, const double& in, double* ibuf, double* obuf, bool rst)
{
	assert(std::abs(a[0]) > EPS);

	// Filter Reset
	if(rst)
	{
		ibuf[0] = ibuf[1] = 0.0;
		obuf[0] = obuf[1] = 0.0;
	}

	double output = (b[0] * in + b[1] * ibuf[0] + b[2] * ibuf[1] - a[1] * obuf[0] - a[2] * obuf[1]) / a[0];

	ibuf[1] = ibuf[0]; ibuf[0] = in;
	obuf[1] = obuf[0]; obuf[0] = output; 

	return output;
}

//------------------------------------------------------------------------------
// 推力プラグインパラメータログ作成
void gazebo::Thr::logParameter()
{
	gzlog << "***************** Thr Parameter \n";
	gzlog << "/thr_parameter/debug             : " << debug_   << "\n";
	gzlog << "/robot_mass_property/cg/x        : " << cg_.X()  << "\n";
	gzlog << "/robot_mass_property/cg/y        : " << cg_.Y()  << "\n";
	gzlog << "/robot_mass_property/cg/z        : " << cg_.Z()  << "\n";
	gzlog << "/thr_parameter/fan_num           : " << fan_num_ << "\n";
	
	for(int i = 0; i < fan_num_; i++)
	{
		std::string fan = "/thr_parameter/fan";
		fan            += std::to_string(i + 1);

		gzlog << fan << "/pos/x        : " << fan_pos_[i].X()   << "\n";
		gzlog << fan << "/pos/y        : " << fan_pos_[i].Y()   << "\n";
		gzlog << fan << "/pos/z        : " << fan_pos_[i].Z()   << "\n";
		gzlog << fan << "/vec/x        : " << fan_frc_vec_[i].X() << "\n";
		gzlog << fan << "/vec/y        : " << fan_frc_vec_[i].Y() << "\n";
		gzlog << fan << "/vec/z        : " << fan_frc_vec_[i].Z() << "\n";
		gzlog << fan << "/force        : " << fan_frc_[i]       << "\n";
		gzlog << fan << "/stddev       : " << stddev_[i]        << "\n";
		gzlog << fan << "/kappa        : " << fan_k_[i]         << "\n";
		gzlog << fan << "/Kprop        : " << k_prop_[i]        << "\n";

		gzlog << "fan" << i + 1 << " Fx                          : " << fan_frc_[i] * fan_frc_vec_[i].X() << "\n";
		gzlog << "fan" << i + 1 << " Fy                          : " << fan_frc_[i] * fan_frc_vec_[i].Y() << "\n";
		gzlog << "fan" << i + 1 << " Fz                          : " << fan_frc_[i] * fan_frc_vec_[i].Z() << "\n";
		gzlog << "fan" << i + 1 << " Tx                          : " << fan_frc_[i] * fan_trq_vec_[i].X() << "\n";
		gzlog << "fan" << i + 1 << " Ty                          : " << fan_frc_[i] * fan_trq_vec_[i].Y() << "\n";
		gzlog << "fan" << i + 1 << " Tz                          : " << fan_frc_[i] * fan_trq_vec_[i].Z() << "\n";
	}

	gzlog << "/thr_parameter/filter/coeff_a/a0 : " << coeff_a_.X() << "\n";
	gzlog << "/thr_parameter/filter/coeff_a/a1 : " << coeff_a_.Y() << "\n";
	gzlog << "/thr_parameter/filter/coeff_a/a2 : " << coeff_a_.Z() << "\n";
	gzlog << "/thr_parameter/filter/coeff_b/b0 : " << coeff_b_.X() << "\n";
	gzlog << "/thr_parameter/filter/coeff_b/b1 : " << coeff_b_.Y() << "\n";
	gzlog << "/thr_parameter/filter/coeff_b/b2 : " << coeff_b_.Z() << "\n";
	
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_MODEL_PLUGIN(gazebo::Thr)

// End Of File -----------------------------------------------------------------
