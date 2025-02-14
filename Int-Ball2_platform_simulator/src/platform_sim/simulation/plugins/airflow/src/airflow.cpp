
#include "airflow/airflow.h"

namespace
{
	/** 微小値 */
	const double EPS(1.0E-10);

	/** 許容誤差 */
	const double TOL(0.1);

	/** 探索近傍点数 */
	const int K(216);

	/** 頂点数 */
	const int VNUM(8);

	/** ロボット機体における抗力作用点数 */
	const int LNUM(6);

	/** ロボットの表面積 */
	double S(0.0);

	/** airflowプラグインのパス */
	std::string plugin_path;

	/** 現在読み込みPCDファイルのPostfix */
	int current_postfix(-1);

	/** 風速ベクトル取得カウンタ */
	int get_cnt(0);

	/** 抗力/風速Publishカウンタ */
	int pub_cnt(0);
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::Airflow::Airflow()
{
	wind_v_lp_.resize(LNUM);
}

//------------------------------------------------------------------------------
// デストラクタ.
gazebo::Airflow::~Airflow() = default;

//------------------------------------------------------------------------------
// プラグインのロード
void gazebo::Airflow::Load(physics::WorldPtr world, sdf::ElementPtr)
{

	// Initialize ros, if it has not already be initialized
	if(!ros::isInitialized())
	{
		int argc    = 0;
		char **argv = NULL;
		ros::init(argc, argv, "airflow",
		ros::init_options::NoSigintHandler);
	}

	// Store the World Pointer
	world_ = world;

	// Create ROS node.
	nh_    = ros::NodeHandle("airflow");

	// Create a name topic, and publish it.
	pub_drag_  = nh_.advertise<geometry_msgs::WrenchStamped>("/airflow/drag", 1);

	// Create a name topic, and publish it.
	pub_wind_  = nh_.advertise<geometry_msgs::TwistStamped>("/airflow/wind", 1);

	// Get Plugin Path
	plugin_path = ros::package::getPath("airflow");

	// Airflow Parameter Update Server
	af_param_server_ = nh_.advertiseService("/sim/airflow/update_params", &Airflow::updateParameter, this);

	// Get Parameter
	getParameter();

	// Read Fd Table
	readFdTable();

	// Set callback to add force in gazebo cycle
	update_ = event::Events::ConnectWorldUpdateBegin(
				std::bind(&Airflow::airflowCallBack, this));
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
void gazebo::Airflow::getParameter()
{
	double x, y, z;

	if(!nh_.getParam("/model_name/iss_name", iss_name_))
	{
		gzerr << "Cannot Get /model_name/iss_name in airflow plugin \n";
	}
	if(!nh_.getParam("/model_name/ib2_name", ib2_name_))
	{
		gzerr << "Cannot Get /model_name/ib2_name in airflow plugin \n";
	}

	if(
		!nh_.getParam("/jpm_pose/pos/x", x) ||
		!nh_.getParam("/jpm_pose/pos/y", y) ||
		!nh_.getParam("/jpm_pose/pos/z", z)
	)
	{
		gzerr << "Cannot Get /jpm_pose/pos/ in airflow plugin \n";
	}
	jpm_pos_.Set(x, y, z);
	if(
		!nh_.getParam("/jpm_pose/att/r", x) ||
		!nh_.getParam("/jpm_pose/att/p", y) ||
		!nh_.getParam("/jpm_pose/att/y", z)
	)
	{
		gzerr << "Cannot Get /jpm_pose/att/ in airflow plugin \n";
	}
	jpm_att_.Set(x, y, z);
	jpm_att_ = jpm_att_ * (M_PI / 180.0);

	for(int i = 0; i < LNUM; i++)
	{
		ignition::math::Vector3d p;
		std::ostringstream       ss;
		ss << i;
		std::string str = "/airflow_parameter/load_point/p" + ss.str();
		if(
			!nh_.getParam(str + "/x", x) ||
			!nh_.getParam(str + "/y", y) ||
			!nh_.getParam(str + "/z", z)
		)
		{
			gzerr << "Cannot Get /airflow_parameter/load_point/p" << i << "/ in airflow plugin \n";
		}
		p.Set(x, y, z);
		load_point_.push_back(p);
	}

	if(
		!nh_.getParam("/robot_mass_property/cg/x", x) || 
		!nh_.getParam("/robot_mass_property/cg/y", y) || 
		!nh_.getParam("/robot_mass_property/cg/z", z)
		)
	{
		gzerr << "Cannot Get /robot_mass_property/cg in airflow plugin \n";
	}
	cg_.Set(x, y, z);

	if(!nh_.getParam("/robot_mass_property/radius", radius_))
	{
		gzerr << "Cannot Get /robot_mass_property/radius in airflow plugin \n";
	}
	S       = 4.0 * M_PI * radius_ * radius_;

	if(!nh_.getParam("/airflow_parameter/force_stddev", force_stddev_))
	{
		gzerr << "Cannot Get /airflow_parameter/force_stddev in airflow plugin \n";
	}

	if(!nh_.getParam("/airflow_parameter/torque_stddev", torque_stddev_))
	{
		gzerr << "Cannot Get /airflow_parameter/torque_stddev in airflow plugin \n";
	}

	if(!nh_.getParam("/airflow_parameter/kappa", kappa_))
	{
		gzerr << "Cannot Get /airflow_parameter/kappa in airflow plugin \n";
	}

	if(!nh_.getParam("/airflow_parameter/pcd", pcd_))
	{
		gzerr << "Cannot Get /airflow_parameter/pcd in airflow plugin \n";
	}
	if(!nh_.getParam("/airflow_parameter/time_step", time_step_))
	{
		gzerr << "Cannot Get /airflow_parameter/time_step in airflow plugin \n";
	}
	if(!nh_.getParam("/airflow_parameter/table", table_))
	{
		gzerr << "Cannot Get /airflow_parameter/table in airflow plugin \n";
	}
	if(!nh_.getParam("/airflow_parameter/get_cycle", get_cycle_))
	{
		gzerr << "Cannot Get /airflow_parameter/get_cycle in airflow plugin \n";
	}
	if(!nh_.getParam("/airflow_parameter/pub_cycle", pub_cycle_))
	{
		gzerr << "Cannot Get /airflow_parameter/pub_cycle in airflow plugin \n";
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
		gzerr << " Could not read the parameters of \"/sim_common/random_seed\".\n";
	}

	logParameter();
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータを取得
bool gazebo::Airflow::updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res)
{
	// 推力/トルク誤差パラメータのみ更新可能とする
	if(!nh_.getParam("/airflow_parameter/force_stddev", force_stddev_))
	{
		gzerr << "Cannot Get /airflow_parameter/force_stddev in airflow plugin \n";
	}

	if(!nh_.getParam("/airflow_parameter/torque_stddev", torque_stddev_))
	{
		gzerr << "Cannot Get /airflow_parameter/torque_stddev in airflow plugin \n";
	}

	res.result = true;
	return true;
}

//------------------------------------------------------------------------------
// Model取得
void gazebo::Airflow::getModels()
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
// airflowコールバック関数
void gazebo::Airflow::airflowCallBack()
{
	ignition::math::Vector3d    force_cg_j;
	ignition::math::Vector3d    force_cg_b;
	ignition::math::Vector3d    torque_b;

	sim_time_ = world_->SimTime().Double();

	getModels();
	loadWindVecField();
	getPointIndex();

	// ロボットの重心に作用する風速ベクトル
	getWindVec(state_.pos, wind_v_cg_);
	// ロボットの作用点に作用する風速ベクトル
	for(int i = 0; i < LNUM; i++)
	{
		// JPM基準座標系での作用点位置(ロボットの姿勢に依らない)
		auto point = state_.pos - state_.rot.RotateVector(cg_) + load_point_[i];
		getWindVec(point, wind_v_lp_[i]);
	}

	getForce(wind_v_cg_,  force_cg_j);
	getTorque(wind_v_lp_, torque_b);

	force_cg_b = state_.rot.RotateVectorReverse(force_cg_j);
	addForceAndTorque(force_cg_b, torque_b);
	pubWrenchAndWind(force_cg_j, torque_b, wind_v_cg_);
}

//------------------------------------------------------------------------------
// JPM基準座標系でのロボット状態量を取得
void gazebo::Airflow::getRobotState()
{
	auto ib2_pose(ib2_link_[0]->WorldCoGPose());
	auto ib2_vel (ib2_link_[0]->WorldCoGLinearVel());
	auto iss_pose(iss_link_[0]->WorldCoGPose());

	auto iss_cg = ignition::math::Vector3d(iss_pose.Pos().X(), iss_pose.Pos().Y(), iss_pose.Pos().Z());
	coord_transformer_.set(iss_cg, jpm_pos_, jpm_att_);

	// Position
	auto iss_qtn   = ignition::math::Quaterniond(iss_pose.Rot().W(), iss_pose.Rot().X(), iss_pose.Rot().Y(), iss_pose.Rot().Z());
	auto world_pos = ignition::math::Vector3d(ib2_pose.Pos().X(), ib2_pose.Pos().Y(), ib2_pose.Pos().Z());
	state_.pos     = coord_transformer_.getJpmRefPosFromWorld(world_pos, iss_qtn);

	// Velocity
	auto iss_w     = iss_link_[0]->RelativeAngularVel();
	auto world_vel = ignition::math::Vector3d(ib2_vel.X(), ib2_vel.Y(), ib2_vel.Z());
	state_.vel     = coord_transformer_.getDsVelFromWorld(world_pos, world_vel, iss_qtn, iss_w);

	// Quaternion
	auto world_qtn = ignition::math::Quaterniond(ib2_pose.Rot().W(), ib2_pose.Rot().X(), ib2_pose.Rot().Y(), ib2_pose.Rot().Z());
	state_.rot     = coord_transformer_.getDsQtnFromWorld(world_qtn, iss_qtn);
}

//------------------------------------------------------------------------------
// PCDファイルからのJEM船内風速ベクトル場取得
void gazebo::Airflow::loadWindVecField()
{
	std::string pcd_file  = plugin_path + pcd_;

	// set postfix(0 fill)
	int step       = int(sim_time_ * 1000) / time_step_;
	int msec_now   = step       * time_step_;
	int msec_next1 = msec_now   + time_step_;
	int msec_next2 = msec_next1 + time_step_;

	if(current_postfix == msec_now)
	{
		return;
	}

	// Read PCD File
	if(msec_now == 0)
	{
		// Current
		std::ostringstream ossnow;
		ossnow << std::setw(8) << std::setfill('0') << msec_now;
		std::string postfix_now   = ossnow.str();
		std::string file_name_now = pcd_file + postfix_now + ".pcd";
		pcl::io::loadPCDFile (file_name_now, cloud_now_);

		// 1-STEP After
		std::ostringstream ossnext1;
		ossnext1 << std::setw(8) << std::setfill('0') << msec_next1;
		std::string postfix_next1   = ossnext1.str();
		std::string file_name_next1 = pcd_file + postfix_next1 + ".pcd";
		pcl::io::loadPCDFile (file_name_next1, cloud_next1_);
	}
	else
	{
		cloud_now_   = cloud_next1_;
		cloud_next1_ = cloud_next2_;
	}

	current_postfix = msec_now;

	// 2-STEP After
	std::ostringstream ossnext2;
	ossnext2 << std::setw(8) << std::setfill('0') << msec_next2;
	std::string postfix_next2   = ossnext2.str();
	std::string file_name_next2 = pcd_file + postfix_next2 + ".pcd";
	pcl::io::loadPCDFile (file_name_next2, cloud_next2_);
}

//------------------------------------------------------------------------------
// K近傍点のポイントインデックスを取得
void gazebo::Airflow::getPointIndex()
{
	if(get_cnt == 0)
	{
		getRobotState();

		// Search K-Nearest Wind Vector
		if(nnSearch(state_.pos, point_idx_) < VNUM)
		{
			gzerr << "Not Found at least 8-Nearst Neighbor\n";
		}
	}
	else if(get_cnt >= static_cast<int>(get_cycle_ * 1000 + TOL) - 1)
	{
			get_cnt = -1;
	}
	get_cnt++;
}

//------------------------------------------------------------------------------
// K-Nearest Neighbors Search
int gazebo::Airflow::nnSearch(const ignition::math::Vector3d& point, std::vector<int>& point_idx)
{
	point_idx.clear();

	// K-NN search
	float                                                        resolution = 0.1f;
	std::vector<float>                                           point_distance;
	pcl::PointWithViewpoint                                      search_point;
	pcl::octree::OctreePointCloudSearch<pcl::PointWithViewpoint> octree(resolution);

	octree.setInputCloud(cloud_now_.makeShared());
	octree.addPointsFromInputCloud();
	search_point.x = point.X();
	search_point.y = point.Y();
	search_point.z = point.Z();

	return octree.nearestKSearch(search_point, K, point_idx, point_distance);
}

//------------------------------------------------------------------------------
// Select (8) Vertices that contain a Robot CG Position
bool gazebo::Airflow::selectVertex(
	const ignition::math::Vector3d& point,
	const std::vector<pcl::PointWithViewpoint>& in_vertices, 
	      std::vector<pcl::PointWithViewpoint>& out_vertices
)
{
	int size = (int)in_vertices.size();
	std::vector<struct Vertex> vertex(size);

	// Study the position relation  
	for(int i = 0; i < size; i++)
	{
		double dx = point.X() - in_vertices[i].x;
		double dy = point.Y() - in_vertices[i].y;
		double dz = point.Z() - in_vertices[i].z;

		vertex[i].distance = sqrt(dx * dx + dy * dy + dz * dz);
		vertex[i].point    = in_vertices[i];

		vertex[i].is_positive_x = (dx < 0) ? true : false;
		vertex[i].is_positive_y = (dy < 0) ? true : false;
		vertex[i].is_positive_z = (dz < 0) ? true : false;
	}

	// Select points that contain Robot CG Position
	double dmin[VNUM];
	for(int i = 0; i < VNUM; i++)
	{
		dmin[i] = DBL_MAX;
	}

	for(int i = 0; i < size; i++)
	{
		unsigned char point_num = ( (vertex[i].is_positive_z << 2) | (vertex[i].is_positive_y << 1) | (vertex[i].is_positive_x << 0) );
		
		for(int pn = 0; pn < VNUM; pn++)
		{
			if(pn == (int)point_num)
			{
				if(vertex[i].distance < dmin[pn])
				{
					dmin[pn]         = vertex[i].distance;
					out_vertices[pn] = vertex[i].point;
				}
			}
		}
	}

	for(int i = 0; i < VNUM; i++)
	{
		if(std::abs(dmin[i] - DBL_MAX) < EPS)
		{
			return false;
		}
	}

	return true;
}

//------------------------------------------------------------------------------
// 線形補間
ignition::math::Vector3d gazebo::Airflow::lerp(const std::vector<pcl::PointWithViewpoint>& vertex, const ignition::math::Vector3d& point)
{
	int size = vertex.size();
	std::vector<ignition::math::Vector3d> v(size);

	for(int i = 0; i < size; i++)
	{
		v[i].X() = vertex[i].vp_x;
		v[i].Y() = vertex[i].vp_y;
		v[i].Z() = vertex[i].vp_z;
	}

	// Interpolate on -Z side
	double x10 = vertex[1].x - vertex[0].x;
	double x32 = vertex[3].x - vertex[2].x;
	double x1w = vertex[1].x - point.X();
	double xw0 = point.X()   - vertex[0].x;
	double x3w = vertex[3].x - point.X();
	double xw2 = point.X()   - vertex[2].x;
	double y01 = (x1w * vertex[0].y + xw0 * vertex[1].y) / x10;
	double z01 = (x1w * vertex[0].z + xw0 * vertex[1].z) / x10;
	double y23 = (x3w * vertex[2].y + xw2 * vertex[3].y) / x32;
	double z23 = (x3w * vertex[2].z + xw2 * vertex[3].z) / x32;

	auto v01 = (x1w * v[0] + xw0 * v[1]) / x10;
	auto v23 = (x3w * v[2] + xw2 * v[3]) / x32;

	auto v03 = ((y23 - point.Y()) * v01 + (point.Y() - y01) * v23) / (y23 - y01);

	// Interpolate on +Z side
	double x54 = vertex[5].x - vertex[4].x;
	double x76 = vertex[7].x - vertex[6].x;
	double x5w = vertex[5].x - point.X();
	double xw4 = point.X()   - vertex[4].x;
	double x7w = vertex[7].x - point.X();
	double xw6 = point.X()   - vertex[6].x;
	double y45 = (x5w * vertex[4].y + xw4 * vertex[5].y) / x54;
	double z45 = (x5w * vertex[4].z + xw4 * vertex[5].z) / x54;
	double y67 = (x7w * vertex[6].y + xw6 * vertex[7].y) / x76;
	double z67 = (x7w * vertex[6].z + xw6 * vertex[7].z) / x76;

	auto v45 = (x5w * v[4] + xw4 * v[5]) / x54;
	auto v67 = (x7w * v[6] + xw6 * v[7]) / x76;

	auto v47 = ((y67 - point.Y()) * v45 + (point.Y() - y45) * v67) / (y67 - y45);

	// Calculate Wind Vector at Robot position
	double z03 = ((y23 - point.Y()) * z01 + (point.Y() - y01) * z23) / (y23 - y01);
	double z47 = ((y67 - point.Y()) * z45 + (point.Y() - y45) * z67) / (y67 - y45);

	auto   vw = ((z47 - point.Z()) * v03 + (point.Z() - z03) * v47) / (z47 - z03);

	return vw;
}

//------------------------------------------------------------------------------
// 抗力テーブル読み込み
void gazebo::Airflow::readFdTable()
{
	std::string file_name = plugin_path + table_;
		
	// Table Open
	std::ifstream ifs;
	ifs.open(file_name, std::ios::in);
	if(!ifs)
	{
		gzerr << "Cannot Open " + file_name << std::endl;
		std::exit(1);
	}

	std::string line;
	while(getline(ifs, line))
	{
		// # means comment line
		if(line.find_first_of('#') == 0 || line.length() == 0)
		{
			continue;
		}

		double tbl[2];
		std::replace(line.begin(), line.end(), ',', ' ');
		std::istringstream iss(line);
		iss >> tbl[0] >> tbl[1];
		fd_table_.push_back({tbl[0], tbl[1]});
	}

	ifs.close();
}

//------------------------------------------------------------------------------
// JPM基準座標系での風速ベクトル取得
void gazebo::Airflow::getWindVec(const ignition::math::Vector3d& point, ignition::math::Vector3d& wind_v)
{
	// Select (8) vertices that contain Robot CG position
	std::vector<pcl::PointWithViewpoint> pv_vec_now;
	std::vector<pcl::PointWithViewpoint> pv_vec_next;
	for(int i = 0; i < K; i++)
	{
		pv_vec_now.push_back(cloud_now_.points[point_idx_[i]]);
		pv_vec_next.push_back(cloud_next1_.points[point_idx_[i]]);
	}
	
	std::vector<pcl::PointWithViewpoint> vertex_now(VNUM);
	std::vector<pcl::PointWithViewpoint> vertex_next(VNUM);
	if(!selectVertex(point, pv_vec_now,  vertex_now) || !selectVertex(point, pv_vec_next, vertex_next))
	{
		gzwarn << "Point (" << point.X() << ", " << point.Y() << ", " << point.Z() << ") is Out of Area prepared by PCD file(airflow Plugin)\n";
		wind_v = ignition::math::Vector3d::Zero;
		return;
	}

	// Linear Interpolation
	auto wind_now  = lerp(vertex_now,  point);
	auto wind_next = lerp(vertex_next, point);

	double l =   sim_time_ - (double)(current_postfix)              / 1000.0;
	double m = -(sim_time_ - (double)(current_postfix + time_step_) / 1000.0);

	wind_v   = (m * wind_now + l * wind_next) / (time_step_ / 1000.0);
}

//------------------------------------------------------------------------------
// 抗力を取得する
void gazebo::Airflow::getForce(const ignition::math::Vector3d& wind_v, ignition::math::Vector3d& force)
{
	// Relative Velocity
	auto   rel_vel(state_.vel - wind_v);
	double rel_vel_norm = rel_vel.Length();

	if(std::abs(rel_vel_norm) < EPS)
	{
		force = ignition::math::Vector3d::Zero;
		return;
	}

	double drag_force_norm = 0;
	for(int i = 0; i < (int)fd_table_.size() - 1; i++)
	{
		if(fd_table_[i][0] <= rel_vel_norm && rel_vel_norm < fd_table_[i + 1][0])
		{
			double l   = -(fd_table_[i][0]     - rel_vel_norm);
			double m   =   fd_table_[i + 1][0] - rel_vel_norm;
			double num =   fd_table_[i + 1][0] - fd_table_[i][0];

			if(num < EPS)
			{
				gzerr << "Divided By 0 in Drag Plugin\n";
				exit(1);
			}

			drag_force_norm = (l * fd_table_[i + 1][1] + m * fd_table_[i][1]) / num;

			continue;
		}

		if(i == (int)fd_table_.size() - 2 && drag_force_norm == 0.0)
		{
			drag_force_norm = fd_table_[i + 1][1];
			gzwarn << "Relative Velocity is over " << fd_table_[i + 1][0] << "\n";
		}
	}

	auto unit_wind_v(rel_vel);
	unit_wind_v.Normalize();

	double nx = ignition::math::Rand::DblNormal(0.0, 1.0);
	double ny = ignition::math::Rand::DblNormal(0.0, 1.0);
	double nz = ignition::math::Rand::DblNormal(0.0, 1.0);
	force     =  -drag_force_norm * unit_wind_v;
	force.X() = force.X() * (1.0 + force_stddev_ * nx);
	force.Y() = force.Y() * (1.0 + force_stddev_ * ny);
	force.Z() = force.Z() * (1.0 + force_stddev_ * nz);
}

//------------------------------------------------------------------------------
// 機体座標系でのトルク取得
void gazebo::Airflow::getTorque(const std::vector<ignition::math::Vector3d>& wind_v, ignition::math::Vector3d& torque)
{
	int  size = wind_v.size();
	auto t    = ignition::math::Vector3d::Zero;

	for(int i = 0; i < size; i++)
	{
		auto rel_vel_j = wind_v[i]      - state_.vel;
		auto arm_j     = load_point_[i] - state_.rot.RotateVector(cg_);
		t              = t + arm_j.Cross(rel_vel_j) / arm_j.Length();
	}

	double nx = ignition::math::Rand::DblNormal(0.0, 1.0);
	double ny = ignition::math::Rand::DblNormal(0.0, 1.0);
	double nz = ignition::math::Rand::DblNormal(0.0, 1.0);
	torque    = state_.rot.RotateVectorReverse(0.125 * radius_ * kappa_ * S * t);
	torque.X()= torque.X() * (1.0 + torque_stddev_ * nx);
	torque.Y()= torque.Y() * (1.0 + torque_stddev_ * ny);
	torque.Z()= torque.Z() * (1.0 + torque_stddev_ * nz);
 }

//------------------------------------------------------------------------------
// エアフローによる抗力、トルクを印加する
void gazebo::Airflow::addForceAndTorque(const ignition::math::Vector3d& force_body, const ignition::math::Vector3d& torque_body)
{
	ib2_link_[0]->AddRelativeForce(force_body);
	ib2_link_[0]->AddRelativeTorque(torque_body);
}

//------------------------------------------------------------------------------
// エアフローによる抗力、トルクおよび風速ベクトルをパブリッシュ
void gazebo::Airflow::pubWrenchAndWind(const ignition::math::Vector3d& f, const ignition::math::Vector3d& t, const ignition::math::Vector3d& w)
{
	if(pub_cnt == 0)
	{
		// Drag
		geometry_msgs::WrenchStamped wrench_stamped;
		wrench_stamped.header.seq      = 0;
		wrench_stamped.header.stamp    = ros::Time(sim_time_);
		wrench_stamped.header.frame_id = "";
		wrench_stamped.wrench.force.x  = f.X();
		wrench_stamped.wrench.force.y  = f.Y();
		wrench_stamped.wrench.force.z  = f.Z();
		wrench_stamped.wrench.torque.x = t.X();
		wrench_stamped.wrench.torque.y = t.Y();
		wrench_stamped.wrench.torque.z = t.Z();

		pub_drag_.publish(wrench_stamped);

		// Wind
		geometry_msgs::TwistStamped wind_stamped;
		wind_stamped.header.seq       = 0;
		wind_stamped.header.stamp     = ros::Time(sim_time_);
		wind_stamped.header.frame_id  = "";
		wind_stamped.twist.linear.x   = w.X();
		wind_stamped.twist.linear.y   = w.Y();
		wind_stamped.twist.linear.z   = w.Z();
		wind_stamped.twist.angular.x  = 0.0;
		wind_stamped.twist.angular.y  = 0.0;
		wind_stamped.twist.angular.z  = 0.0;

		pub_wind_.publish(wind_stamped);
	}
	else if(pub_cnt >= static_cast<int>(pub_cycle_ * 1000 + TOL) - 1)
	{
		pub_cnt = -1;
	}
	pub_cnt++;
}

//------------------------------------------------------------------------------
//  エアフロープラグインパラメータログ作成
void gazebo::Airflow::logParameter()
{
	gzlog << "***************** Airflow Parameter \n";
	
	for(int i = 0; i < LNUM; i++)
	{
		gzlog << "/airflow_parameter/load_point/p" << i << "/x : " << load_point_[i].X() << "\n";
		gzlog << "/airflow_parameter/load_point/p" << i << "/y : " << load_point_[i].Y() << "\n";
		gzlog << "/airflow_parameter/load_point/p" << i << "/z : " << load_point_[i].Z() << "\n";
	}

	gzlog << "/airflow_parameter/force_stddev    : " << force_stddev_  << "\n";
	gzlog << "/airflow_parameter/torque_stddev   : " << torque_stddev_ << "\n";
	gzlog << "/airflow_parameter/kappa           : " << kappa_         << "\n";
	gzlog << "/airflow_parameter/pcd             : " << pcd_           << "\n";
	gzlog << "/airflow_parameter/time_step       : " << time_step_     << "\n";
	gzlog << "/airflow_parameter/table           : " << table_         << "\n";
	gzlog << "/airflow_parameter/get_cycle       : " << get_cycle_     << "\n";
	gzlog << "/airflow_parameter/pub_cycle       : " << pub_cycle_     << "\n";	
}

//------------------------------------------------------------------------------
// Gazeboのモデルプラグインとして登録
GZ_REGISTER_WORLD_PLUGIN(gazebo::Airflow)

// End Of File -----------------------------------------------------------------


