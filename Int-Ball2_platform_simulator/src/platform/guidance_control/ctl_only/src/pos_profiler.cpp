
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
// パラメータによるコンストラクタ
ib2::PosProfiler::PosProfiler(rclcpp::Node* node) :
Fmax_(0.), vmax_(0.), xthr_(0.1), eta_(0.99), etamax_(0.99),
epsRm_(0.001), aip_(Eigen::Vector3d::Zero()), rdp_(Eigen::Vector3d::Zero())
{
	using namespace ib2_mss;

	auto get_param = [node](const std::string& name, auto& value) {
		using T = std::decay_t<decltype(value)>;
		if (!node->has_parameter(name)) {
			node->declare_parameter<T>(name, value);
		}
		node->get_parameter(name, value);
	};

	static const RangeCheckerD F_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.5, true);
	static const RangeCheckerD V_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.25, true);
	static const RangeCheckerD ETA_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 1., true);
	static const RangeCheckerD EPS_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.001, true);

	static const std::string PARAM_F_MAX  ("pos_profile.f_max");
	static const std::string PARAM_V_MAX  ("pos_profile.v_max");
	static const std::string PARAM_X_THR  ("pos_profile.x_threshold");
	static const std::string PARAM_ETA    ("pos_profile.eta" );
	static const std::string PARAM_ETA_MAX("pos_profile.eta_max");
	static const std::string PARAM_EPS    ("pos_profile.eps_rm");

	double Fmax  (-1.);
	double vmax  (-1.);
	double xthr  (-1.);
	double eta   (-1.);
	double etamax(-1.);
	double eps_rm (-1.);

	get_param(PARAM_F_MAX  , Fmax);
	get_param(PARAM_V_MAX  , vmax);
	get_param(PARAM_X_THR  , xthr);
	get_param(PARAM_ETA    , eta);
	get_param(PARAM_ETA_MAX, etamax);
	get_param(PARAM_EPS    , eps_rm);

	F_MAX_RANGE.valid(Fmax, PARAM_F_MAX);
	V_MAX_RANGE.valid(vmax, PARAM_V_MAX);
	RangeCheckerD::notNegative(xthr, true, PARAM_X_THR);
	ETA_RANGE.valid(eta   , PARAM_ETA);
	ETA_RANGE.valid(etamax, PARAM_ETA_MAX);
	EPS_RANGE.valid(eps_rm , PARAM_EPS);

	Fmax_   = Fmax;
	vmax_   = vmax;
	xthr_   = xthr;
	eta_    = eta;
	etamax_ = etamax;
	epsRm_  = eps_rm;

	get_param("pos_profile.aip.x", aip_.x());
	get_param("pos_profile.aip.y", aip_.y());
	get_param("pos_profile.aip.z", aip_.z());
	get_param("pos_profile.rdp.x", rdp_.x());
	get_param("pos_profile.rdp.y", rdp_.y());
	get_param("pos_profile.rdp.z", rdp_.z());

	RCLCPP_INFO(node->get_logger(), "******** Set Parameters in pos_profiler.cpp");
	RCLCPP_INFO(node->get_logger(), "pos_profile.f_max           : %f", Fmax_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.v_max           : %f", vmax_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.x_threshold     : %f", xthr_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.eta             : %f", eta_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.eta_max         : %f", etamax_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.eps_rm          : %f", epsRm_);
	RCLCPP_INFO(node->get_logger(), "pos_profile.aip.x           : %f", aip_.x());
	RCLCPP_INFO(node->get_logger(), "pos_profile.aip.y           : %f", aip_.y());
	RCLCPP_INFO(node->get_logger(), "pos_profile.aip.z           : %f", aip_.z());
	RCLCPP_INFO(node->get_logger(), "pos_profile.rdp.x           : %f", rdp_.x());
	RCLCPP_INFO(node->get_logger(), "pos_profile.rdp.y           : %f", rdp_.y());
	RCLCPP_INFO(node->get_logger(), "pos_profile.rdp.z           : %f", rdp_.z());
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
