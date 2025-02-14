
#include "mag/mag.h"

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 許容誤差 */
	const double TOL(0.1);

	/** Publishカウンタ */
	int pub_cnt(0);

	/** 磁力ON/OFFサービス名 */
	std::string SERVICE_SWITCH_POWER("/mag/switch_power");
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Mag::Mag() = default;

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Mag::~Mag() = default;

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Mag::Load(physics::WorldPtr world, sdf::ElementPtr)
{

	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "mag",
		ros::init_options::NoSigintHandler);
	}

	// Store the World Pointer
	world_  = world;

	// Create ROS node.
	nh_     = ros::NodeHandle("mag");

	// Publisher
	pub_mag_          = nh_.advertise<geometry_msgs::WrenchStamped>("/mag/wrench_stamped", 1);
	pub_power_status_ = nh_.advertise<ib2_msgs::PowerStatus>("/mag/power_status", 1);

	// Magnet Force ON/OFF Service Server
	switch_power_server_ = nh_.advertiseService(SERVICE_SWITCH_POWER, &Mag::switchPower, this);

	// Mag Parameter Update Server
	mag_param_server_ = nh_.advertiseService("/sim/mag/update_params", &Mag::updateParameter, this);

	// Get Parameters
	getParameter();

	// Set callback to add force in gazebo cycle
	update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Mag::magCallBack, this));
}

//------------------------------------------------------------------------------
// ROS Timerのコールバック関数
void gazebo::Mag::magCallBack()
{
	auto fmag_ds = ignition::math::Vector3d::Zero;
	auto fmag_bd = ignition::math::Vector3d::Zero;
	auto tmag_bd = ignition::math::Vector3d::Zero;
	auto r_ds    = ignition::math::Vector3d::Zero;
	auto q_ds    = ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0);
	
	getModels();
	getDsPose(r_ds, q_ds);
	getForce(r_ds, q_ds, fmag_ds, fmag_bd);
	getTorque(fmag_bd, tmag_bd);

	if(!power_status_.status){
		fmag_ds = ignition::math::Vector3d::Zero; 
		fmag_bd = ignition::math::Vector3d::Zero;
		tmag_bd = ignition::math::Vector3d::Zero;
	}

	addForceAndTorque(fmag_bd, tmag_bd);
	pubForceAndTorque(fmag_ds, tmag_bd);
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
void gazebo::Mag::getParameter()
{
	double x, y, z;

	// Model Name
	if(!nh_.getParam("/model_name/iss_name", iss_name_))
	{
		gzerr << "Cannot Get /model_name/iss_name in mag plugin \n";
	}
	if(!nh_.getParam("/model_name/ib2_name", ib2_name_))
	{
		gzerr << "Cannot Get /model_name/ib2_name in mag plugin \n";
	}

	// JPM Pose
	if(
		!nh_.getParam("/jpm_pose/pos/x", x) ||
		!nh_.getParam("/jpm_pose/pos/y", y) ||
		!nh_.getParam("/jpm_pose/pos/z", z)
	)
	{
		gzerr << "Cannot Get /jpm_pose/pos/ in mag plugin \n";
	}
	jpm_pos_.Set(x, y, z);
	if(
		!nh_.getParam("/jpm_pose/att/r", x) ||
		!nh_.getParam("/jpm_pose/att/p", y) ||
		!nh_.getParam("/jpm_pose/att/y", z)
	)
	{
		gzerr << "Cannot Get /jpm_pose/att/ in mag plugin \n";
	}
	jpm_att_.Set(x, y, z);
	jpm_att_ = jpm_att_ * DEG2RAD;

	// DS Pose
	if(
		!nh_.getParam("/ds_pose/pos/x", x) ||
		!nh_.getParam("/ds_pose/pos/y", y) ||
		!nh_.getParam("/ds_pose/pos/z", z)
	)
	{
		gzerr << "Cannot Get /ds_pose/pos/ in mag plugin \n";
	}
	ds_pos_.Set(x, y, z);
	if(
		!nh_.getParam("/ds_pose/att/r", x) ||
		!nh_.getParam("/ds_pose/att/p", y) ||
		!nh_.getParam("/ds_pose/att/y", z)
	)
	{
		gzerr << "Cannot Get /ds_pose/att/ in mag plugin \n";
	}
	ds_att_.Set(x, y, z);
	ds_att_ = ds_att_ * DEG2RAD;

	// Power Status
	bool ps;
	if(!nh_.getParam("/mag_parameter/power_status", ps)){
		power_status_.status = ib2_msgs::PowerStatus::OFF;
		gzerr << "Cannot Get /mag_parameter/power_status in mag plugin \n";
	}
	power_status_.status = ps;

	// Mag IF
	if(
		!nh_.getParam("/mag_parameter/robo_if/x", x) ||
		!nh_.getParam("/mag_parameter/robo_if/y", y) ||
		!nh_.getParam("/mag_parameter/robo_if/z", z)
	)
	{
		gzerr << "Cannot Get /mag_parameter/robo_if/ in mag plugin \n";
	}
	rrif_.Set(x, y, z);
	if(
		!nh_.getParam("/mag_parameter/ds_if/x", x) ||
		!nh_.getParam("/mag_parameter/ds_if/y", y) ||
		!nh_.getParam("/mag_parameter/ds_if/z", z)
	)
	{
		gzerr << "Cannot Get /mag_parameter/ds_if/ in mag plugin \n";
	}
	rdif_.Set(x, y, z);

	// CG
	if(
		!nh_.getParam("/robot_mass_property/cg/x", x) || 
		!nh_.getParam("/robot_mass_property/cg/y", y) || 
		!nh_.getParam("/robot_mass_property/cg/z", z)
		)
	{
		gzerr << "Cannot Get /robot_mass_property/cg in mag plugin \n";
	}
	cg_.Set(x, y, z);

	// Threshold
	if(!nh_.getParam("/mag_parameter/threshold", d_threshold_))
	{
		gzerr << "Cannot Get /mag_parameter/threshold in mag plugin \n";
	}

	// Coefficient
	if(
		!nh_.getParam("/mag_parameter/coeff_far/a", afar_) ||
		!nh_.getParam("/mag_parameter/coeff_far/b", bfar_) ||
		!nh_.getParam("/mag_parameter/coeff_far/c", cfar_)
	)
	{
		gzerr << "Cannot Get /mag_parameter/coeff_far/ in mag plugin \n";
	}
	if(
		!nh_.getParam("/mag_parameter/coeff_prox/a", aprox_) ||
		!nh_.getParam("/mag_parameter/coeff_prox/b", bprox_) ||
		!nh_.getParam("/mag_parameter/coeff_prox/c", cprox_)
	)
	{
		gzerr << "Cannot Get /mag_parameter/coeff_prox/ in mag plugin \n";
	}

	// Standard Deviation
	if(!nh_.getParam("/mag_parameter/stddev", stddev_))
	{
		gzerr << "Cannot Get /mag_parameter/stddev in mag plugin \n";
	}

	// Cycle
	if(!nh_.getParam("/mag_parameter/cycle", cycle_))
	{
		gzerr << "Cannot Get /mag_parameter/cycle in mag plugin \n";
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
bool gazebo::Mag::updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res)
{
	getParameter();

	res.result = true;
	return true;
}

//------------------------------------------------------------------------------
// Model取得
void gazebo::Mag::getModels()
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
// ドッキングステーション座標系(ホーム座標系)でのロボット位置・姿勢を取得
void gazebo::Mag::getDsPose(ignition::math::Vector3d& r_ds, ignition::math::Quaterniond& q_ds)
{
	auto ib2_pose  = ib2_link_[0]->WorldCoGPose();
	auto iss_pose  = iss_link_[0]->WorldCoGPose();
	auto iss_cg    = ignition::math::Vector3d(iss_pose.Pos().X(), iss_pose.Pos().Y(), iss_pose.Pos().Z());
	coord_transformer_.set(iss_cg, jpm_pos_, jpm_att_, ds_pos_, ds_att_);

	// CG Pos
	auto iss_qtn   = ignition::math::Quaterniond(iss_pose.Rot().W(), iss_pose.Rot().X(), iss_pose.Rot().Y(), iss_pose.Rot().Z());
	auto world_pos = ib2_pose.Pos();
	r_ds           = coord_transformer_.getDsPosFromWorld(world_pos, iss_qtn);

	// Quaternion
	auto world_qtn = ib2_pose.Rot();
	q_ds           = coord_transformer_.getDsQtnFromWorld(world_qtn, iss_qtn);
}

//------------------------------------------------------------------------------
// 磁石による吸引力を取得
void gazebo::Mag::getForce(
	const ignition::math::Vector3d& r_ds,   const ignition::math::Quaterniond& q_ds,
	      ignition::math::Vector3d& fmag_ds,      ignition::math::Vector3d&    fmag_bd
)
{
	auto   dif   = rdif_ - r_ds;                   // ロボット重心       to DS側磁力IF点(ホーム座標系)
	auto   rif   = q_ds.RotateVector(rrif_);       // ロボット重心       to ロボット側磁力IF点(ホーム座標系)
	auto   rrifd = r_ds  + rif;                    // DS原点             to ロボット側磁力IF点(ホーム座標系)
	auto   rho   = rdif_ - rrifd;                  // ロボット側磁力IF点 to DS側磁力IF点(ホーム座標系)
	double rhon  = rho.Length();
	double inn   = dif.Dot(rif);

	double nx    = ignition::math::Rand::DblNormal(0.0, 1.0);
	double ny    = ignition::math::Rand::DblNormal(0.0, 1.0);
	double nz    = ignition::math::Rand::DblNormal(0.0, 1.0);
	fmag_ds      = U(inn) * H(rhon) * rho.Normalize();
	fmag_ds.X()  = fmag_ds.X() * (1.0 + stddev_ * nx);
	fmag_ds.Y()  = fmag_ds.Y() * (1.0 + stddev_ * ny);
	fmag_ds.Z()  = fmag_ds.Z() * (1.0 + stddev_ * nz);
	
	fmag_bd      = q_ds.RotateVectorReverse(fmag_ds);
}

//------------------------------------------------------------------------------
//  磁石によるトルクを取得
void gazebo::Mag::getTorque(const ignition::math::Vector3d& fmag_bd, ignition::math::Vector3d& tmag_bd)
{
	auto arm = rrif_ - cg_;
	tmag_bd  = arm.Cross(fmag_bd);
}

//------------------------------------------------------------------------------
// 吸引力・トルクを印加
void gazebo::Mag::addForceAndTorque(const ignition::math::Vector3d& fmag_bd, const ignition::math::Vector3d& tmag_bd)
{
	ib2_link_[0]->AddRelativeForce(fmag_bd);
	ib2_link_[0]->AddRelativeTorque(tmag_bd);
}

//------------------------------------------------------------------------------
// 吸引力・トルクをパブリッシュ
void gazebo::Mag::pubForceAndTorque(const ignition::math::Vector3d& fmag_ds, const ignition::math::Vector3d& tmag_bd)
{
	if(pub_cnt == 0)
	{
		double sim_time = world_->SimTime().Double();

		geometry_msgs::WrenchStamped wrench_stamped;
		wrench_stamped.header.seq      = 0;
		wrench_stamped.header.stamp    = ros::Time(sim_time);
		wrench_stamped.header.frame_id = "";
		wrench_stamped.wrench.force.x  = fmag_ds.X();
		wrench_stamped.wrench.force.y  = fmag_ds.Y();
		wrench_stamped.wrench.force.z  = fmag_ds.Z();
		wrench_stamped.wrench.torque.x = tmag_bd.X();
		wrench_stamped.wrench.torque.y = tmag_bd.Y();
		wrench_stamped.wrench.torque.z = tmag_bd.Z();

		pub_mag_.publish(wrench_stamped);
		pub_power_status_.publish(power_status_);
	}
	else if(pub_cnt >= static_cast<int>(cycle_ * 1000 + TOL) - 1)
	{
		pub_cnt = -1;
	}
	pub_cnt++;
}

//------------------------------------------------------------------------------
// 磁力ON/OFF
bool gazebo::Mag::switchPower
(ib2_msgs::SwitchPower::Request&   req, 
 ib2_msgs::SwitchPower::Response&  res)
{
	power_status_.status     = req.power.status;
	res.current_power.status = power_status_.status;

	return true;
}

//------------------------------------------------------------------------------
// 磁力プラグインパラメータログ作成
void gazebo::Mag::logParameter()
{
	int ps = power_status_.status;

	gzlog << "***************** Mag Parameter \n";
	gzlog << "/mag_parameter/power_status          : " << ps                   << "\n";
	gzlog << "/mag_parameter/robo_if/x             : " << rrif_.X()            << "\n";
	gzlog << "/mag_parameter/robo_if/y             : " << rrif_.Y()            << "\n";
	gzlog << "/mag_parameter/robo_if/z             : " << rrif_.Z()            << "\n";
	gzlog << "/mag_parameter/ds_if/x               : " << rdif_.X()            << "\n";
	gzlog << "/mag_parameter/ds_if/y               : " << rdif_.Y()            << "\n";
	gzlog << "/mag_parameter/ds_if/z               : " << rdif_.Z()            << "\n";
	gzlog << "/mag_parameter/threshold             : " << d_threshold_         << "\n";
	gzlog << "/mag_parameter/coeff_far/a           : " << afar_                << "\n";
	gzlog << "/mag_parameter/coeff_far/b           : " << bfar_                << "\n";
	gzlog << "/mag_parameter/coeff_far/c           : " << cfar_                << "\n";
	gzlog << "/mag_parameter/coeff_prox/a          : " << aprox_               << "\n";
	gzlog << "/mag_parameter/coeff_prox/b          : " << bprox_               << "\n";
	gzlog << "/mag_parameter/coeff_prox/c          : " << cprox_               << "\n";
	gzlog << "/mag_parameter/stddev                : " << stddev_              << "\n";
	gzlog << "/mag_parameter/cycle                 : " << cycle_               << "\n";
}

//------------------------------------------------------------------------------
// 単位ステップ関数
bool gazebo::Mag::U(double x)
{
	return ((x > 0.0)? true : false);
}

//------------------------------------------------------------------------------
// 吸引力関数
double gazebo::Mag::H(double dist)
{
	dist = std::fabs(dist);

	double a = afar_;
	double b = bfar_;
	double c = cfar_;

	if(dist <= d_threshold_)
	{
		a = aprox_;
		b = bprox_;
		c = cprox_;
	}

	double f_mn = b / pow((1000.0 * dist + a), 2) + c;

	return f_mn / 1000.0;
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_WORLD_PLUGIN(gazebo::Mag)

// End Of File -----------------------------------------------------------------


