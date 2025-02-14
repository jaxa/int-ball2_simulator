
#include "ctl/pos_profiler.h"

#include "guidance_control_common/Constants.h"
#include "guidance_control_common/RangeChecker.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::PosProfiler::PosProfiler() :
Fmax_(0.), vmax_(0.), xthr_(0.1), eta_(0.99), etamax_(0.99),
epsRm_(0.001), aip_(Eigen::Vector3d::Zero()), rdp_(Eigen::Vector3d::Zero())
{
	aip_ << 0.25, 0., 0.;
	rdp_ << 0.1 , 0., 0.;
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::PosProfiler::PosProfiler(const ros::NodeHandle& nh) :
Fmax_(0.), vmax_(0.), xthr_(0.1), eta_(0.99), etamax_(0.99),
epsRm_(0.001), aip_(Eigen::Vector3d::Zero()), rdp_(Eigen::Vector3d::Zero())
{
	using namespace ib2_mss;
	static const RangeCheckerD F_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.5, true);
	static const RangeCheckerD V_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.25, true);
	static const RangeCheckerD ETA_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 1., true);
	static const RangeCheckerD EPS_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.001, true);
	
	static const std::string ROSPARAM_F_MAX  ("/pos_profile/f_max");
	static const std::string ROSPARAM_V_MAX  ("/pos_profile/v_max");
	static const std::string ROSPARAM_X_THR  ("/pos_profile/x_threshold");
	static const std::string ROSPARAM_ETA    ("/pos_profile/eta" );
	static const std::string ROSPARAM_ETA_MAX("/pos_profile/eta_max");
	static const std::string ROSPARAM_EPS    ("/pos_profile/eps_rm");

	double Fmax  (-1.);
	double vmax  (-1.);
	double xthr  (-1.);
	double eta   (-1.);
	double etamax(-1.);
	double eps_rm (-1.);
	
	nh.getParam(ROSPARAM_F_MAX  , Fmax);
	nh.getParam(ROSPARAM_V_MAX  , vmax);
	nh.getParam(ROSPARAM_X_THR  , xthr);
	nh.getParam(ROSPARAM_ETA    , eta);
	nh.getParam(ROSPARAM_ETA_MAX, etamax);
	nh.getParam(ROSPARAM_EPS    , eps_rm);

	F_MAX_RANGE.valid(Fmax, ROSPARAM_F_MAX);
	V_MAX_RANGE.valid(vmax, ROSPARAM_V_MAX);
	RangeCheckerD::notNegative(xthr, true, ROSPARAM_X_THR);
	ETA_RANGE.valid(eta   , ROSPARAM_ETA);
	ETA_RANGE.valid(etamax, ROSPARAM_ETA_MAX);
	EPS_RANGE.valid(eps_rm , ROSPARAM_EPS);
	
	Fmax_   = Fmax;
	vmax_   = vmax;
	xthr_   = xthr;
	eta_    = eta;
	etamax_ = etamax;
	epsRm_  = eps_rm;
	
	nh.getParam("/pos_profile/aip/x"       , aip_.x());
	nh.getParam("/pos_profile/aip/y"       , aip_.y());
	nh.getParam("/pos_profile/aip/z"       , aip_.z());
	nh.getParam("/pos_profile/rdp/x"       , rdp_.x());
	nh.getParam("/pos_profile/rdp/y"       , rdp_.y());
	nh.getParam("/pos_profile/rdp/z"       , rdp_.z());

	ROS_INFO("******** Set Parameters in pos_profiler.cpp");
	ROS_INFO("/pos_profile/f_max           : %f", Fmax_);
	ROS_INFO("/pos_profile/v_max           : %f", vmax_);
	ROS_INFO("/pos_profile/x_threshold     : %f", xthr_);
	ROS_INFO("/pos_profile/eta             : %f", eta_);
	ROS_INFO("/pos_profile/eta_max         : %f", etamax_);
	ROS_INFO("/pos_profile/eps_rm          : %f", epsRm_);
	ROS_INFO("/pos_profile/aip/x           : %f", aip_.x());
	ROS_INFO("/pos_profile/aip/y           : %f", aip_.y());
	ROS_INFO("/pos_profile/aip/z           : %f", aip_.z());
	ROS_INFO("/pos_profile/rdp/x           : %f", rdp_.x());
	ROS_INFO("/pos_profile/rdp/y           : %f", rdp_.y());
	ROS_INFO("/pos_profile/rdp/z           : %f", rdp_.z());
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::PosProfiler::~PosProfiler() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::PosProfiler::PosProfiler
(const PosProfiler&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::PosProfiler&
ib2::PosProfiler::operator=(const PosProfiler&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::PosProfiler::PosProfiler(PosProfiler&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::PosProfiler&
ib2::PosProfiler::operator=(PosProfiler&&) = default;

//------------------------------------------------------------------------------
//  位置プロファイル最大推力の取得
double ib2::PosProfiler::Fmax() const
{
	return Fmax_;
}
//------------------------------------------------------------------------------
//  位置プロファイル最大速度の取得
double ib2::PosProfiler::vmax() const
{
	return vmax_;
}

//------------------------------------------------------------------------------
//  角速度制御を切る閾値の取得
double ib2::PosProfiler::xthr() const
{
	return xthr_;
}

//------------------------------------------------------------------------------
//  プロファイル作成用スラスタ能率の取得
double ib2::PosProfiler::eta() const
{
	return eta_;
}

//------------------------------------------------------------------------------
//  プロファイル作成用スラスタ能率の取得
double ib2::PosProfiler::etamax() const
{
	return etamax_;
}

//------------------------------------------------------------------------------
//  移動量微小数の取得
double ib2::PosProfiler::epsRm() const
{
	return epsRm_;
}

//------------------------------------------------------------------------------
//  AIPの参照
const Eigen::Vector3d& ib2::PosProfiler::aip() const
{
	return aip_;
}

//------------------------------------------------------------------------------
//  RDPの参照
const Eigen::Vector3d& ib2::PosProfiler::rdp() const
{
	return rdp_;
}

// End Of File -----------------------------------------------------------------
