
#include "hill/hill.h"

namespace
{
	/** 単位変換係数DEG->RAD */
	const double DEG2RAD(M_PI / 180.0);

	/** 微小値 */
	const double EPS(1.0E-10);

	/** 許容誤差 */
	const double TOL(0.1);

	/** Publishカウンタ */
	int pub_cnt(0);
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Hill::Hill() = default;

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Hill::~Hill() = default;

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Hill::Load(physics::WorldPtr world, sdf::ElementPtr)
{
	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "hill",
		ros::init_options::NoSigintHandler);
	}

	// Store the World Pointer
	world_ = world;

	// Create ROS node.
	nh_    = ros::NodeHandle("hill");

	// Publish Hill Force
	pub_hill_force_ = nh_.advertise<geometry_msgs::WrenchStamped>("/hill/force", 1);

	// Get parameters by rosparam
	getParameter();

	// Set callback to add force in gazebo cycle
	update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Hill::addHillForce, this));
}

//------------------------------------------------------------------------------
//  ROS Parameter Serverからパラメータ取得
void gazebo::Hill::getParameter()
{
	if(!nh_.getParam("/model_name/iss_name", iss_name_))
	{
		gzerr << "Cannot Get /model_name/iss_name in hill plugin \n";
	}
	if(!nh_.getParam("/model_name/ib2_name", ib2_name_))
	{
		gzerr << "Cannot Get /model_name/ib2_name in hill plugin \n";
	}
	if(!nh_.getParam("/hill_parameter/cycle", pub_cycle_))
	{
		gzerr << "Cannot Get /hill_parameter/cycle in hill plugin \n";
	}
	if(!nh_.getParam("/hill_parameter/w",     iss_w_))
	{
		gzerr << "Cannot Get /hill_parameter/w in hill plugin \n";
	}

	iss_w_ = iss_w_ * DEG2RAD;
}

//------------------------------------------------------------------------------
// Model取得
void gazebo::Hill::getModels()
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
// 相対加速度(Hill方程式)の印加
void gazebo::Hill::addHillForce()
{
	getModels();

	double w  = iss_w_;
	double w2 = std::pow(w, 2.0);

	auto   iss_pose(iss_link_[0]->WorldCoGPose());
	auto   ib2_pose(ib2_link_[0]->WorldCoGPose());
	auto   v       (ib2_link_[0]->WorldCoGLinearVel());
	auto   m       (ib2_link_[0]->GetInertial()->Mass());
	auto   pr      (ib2_pose.Pos() - iss_pose.Pos());

	hill_force_.X() = m * ( 2.0 * w * v.Z());
	hill_force_.Y() = m * (                       - w2 * pr.Y());
	hill_force_.Z() = m * (-2.0 * w * v.X() + 3.0 * w2 * pr.Z());

	// Add Hill Force
	ib2_link_[0]->AddForce(hill_force_);

	// Publish Hill Force
	if(pub_cnt == 0)
	{
		double sim_time = world_->SimTime().Double();

		geometry_msgs::WrenchStamped force_stamped;
		force_stamped.header.seq      = 0;
		force_stamped.header.stamp    = ros::Time(sim_time);
		force_stamped.header.frame_id = "";
		force_stamped.wrench.force.x  = hill_force_.X();
		force_stamped.wrench.force.y  = hill_force_.Y();
		force_stamped.wrench.force.z  = hill_force_.Z();
		force_stamped.wrench.torque.x = 0.0;
		force_stamped.wrench.torque.y = 0.0;
		force_stamped.wrench.torque.z = 0.0;

		pub_hill_force_.publish(force_stamped);
	}
	else if(pub_cnt >= static_cast<int>(pub_cycle_ * 1000 + TOL) - 1)
	{
		pub_cnt = -1;
	}
	pub_cnt++;
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_WORLD_PLUGIN(gazebo::Hill)

// End Of File -----------------------------------------------------------------


