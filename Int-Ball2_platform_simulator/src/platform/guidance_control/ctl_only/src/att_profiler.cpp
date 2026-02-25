
#include "ctl/att_profiler.h"

#include "guidance_control_common/Constants.h"
#include "guidance_control_common/RangeChecker.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::AttProfiler::AttProfiler() :
Tmax_(0.), wmax_(0.), qthr_(ib2_mss::DEG),
epsQm_(1.e-9), aia_(Eigen::Quaterniond::Identity()),
rda_(Eigen::Quaterniond::Identity()), scanAxes_(2, Eigen::Vector3d::UnitX())
{
}

//------------------------------------------------------------------------------
// パラメータによるコンストラクタ
ib2::AttProfiler::AttProfiler(rclcpp::Node* node) :
Tmax_(0.), wmax_(0.), qthr_(ib2_mss::DEG),
epsQm_(1.e-9), aia_(Eigen::Quaterniond::Identity()),
rda_(Eigen::Quaterniond::Identity()), scanAxes_(3, Eigen::Vector3d::UnitX())
{
	using namespace ib2_mss;

	auto get_param = [node](const std::string& name, auto& value) {
		using T = std::decay_t<decltype(value)>;
		if (!node->has_parameter(name)) {
			node->declare_parameter<T>(name, value);
		}
		node->get_parameter(name, value);
	};

	static const RangeCheckerD T_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.02, true);
	static const RangeCheckerD W_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.5236, true);
	static const RangeCheckerD EPS_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.0002, true);

	static const std::string PARAM_T_MAX("att_profile.t_max");
	static const std::string PARAM_W_MAX("att_profile.w_max");
	static const std::string PARAM_Q_THR("att_profile.theta_threshold");
	static const std::string PARAM_EPS  ("att_profile.eps_qm");

	double Tmax(-1.);
	double wmax(-1.);
	double qthr(-1.);
	double eps_qm(-1.);
	Eigen::Quaterniond aia(0., 0., 0., 0.);
	Eigen::Quaterniond rda(0., 0., 0., 0.);
	std::vector<Eigen::Vector3d> scan(3, Eigen::Vector3d::Zero());

	get_param(PARAM_T_MAX, Tmax);
	get_param(PARAM_W_MAX, wmax);
	get_param(PARAM_Q_THR, qthr);
	get_param(PARAM_EPS  , eps_qm);
	get_param("att_profile.aia.x", aia.x());
	get_param("att_profile.aia.y", aia.y());
	get_param("att_profile.aia.z", aia.z());
	get_param("att_profile.aia.w", aia.w());
	get_param("att_profile.rda.x", rda.x());
	get_param("att_profile.rda.y", rda.y());
	get_param("att_profile.rda.z", rda.z());
	get_param("att_profile.rda.w", rda.w());
	get_param("att_profile.scan.axis1.x", scan.at(0).x());
	get_param("att_profile.scan.axis1.y", scan.at(0).y());
	get_param("att_profile.scan.axis1.z", scan.at(0).z());
	get_param("att_profile.scan.axis2.x", scan.at(1).x());
	get_param("att_profile.scan.axis2.y", scan.at(1).y());
	get_param("att_profile.scan.axis2.z", scan.at(1).z());
	get_param("att_profile.scan.axis3.x", scan.at(2).x());
	get_param("att_profile.scan.axis3.y", scan.at(2).y());
	get_param("att_profile.scan.axis3.z", scan.at(2).z());

	T_MAX_RANGE.valid(Tmax, PARAM_T_MAX);
	W_MAX_RANGE.valid(wmax, PARAM_W_MAX);
	RangeCheckerD::notNegative(qthr, true, PARAM_Q_THR);
	EPS_RANGE.valid(eps_qm , PARAM_EPS);
	RangeCheckerD::positive(aia.norm(), true, "AIA quaternion magnitude");
	RangeCheckerD::positive(rda.norm(), true, "RDA quaternion magnitude");
	for (size_t i = 0; i < scan.size(); ++i)
	{
		std::ostringstream ss;
		ss << "scan axis " << i+1 << " magnitude";
		RangeCheckerD::positive(scan.at(i).norm(), true, ss.str());
		scan.at(i).normalize();
	}

	Tmax_ = Tmax;
	wmax_ = wmax;
	qthr_ = qthr;
	epsQm_ = eps_qm;
	aia_ = aia.normalized();
	rda_ = rda.normalized();
	scanAxes_ = scan;

	RCLCPP_INFO(node->get_logger(), "******** Set Parameters in att_profiler.cpp");
	RCLCPP_INFO(node->get_logger(), "att_profile.t_max           : %f", Tmax_);
	RCLCPP_INFO(node->get_logger(), "att_profile.w_max           : %f", wmax_);
	RCLCPP_INFO(node->get_logger(), "att_profile.theta_threshold : %f", qthr_);
	RCLCPP_INFO(node->get_logger(), "att_profile.eps_qm          : %f", epsQm_);
	RCLCPP_INFO(node->get_logger(), "att_profile.aia_x           : %f", aia_.x());
	RCLCPP_INFO(node->get_logger(), "att_profile.aia_y           : %f", aia_.y());
	RCLCPP_INFO(node->get_logger(), "att_profile.aia_z           : %f", aia_.z());
	RCLCPP_INFO(node->get_logger(), "att_profile.aia_w           : %f", aia_.w());
	RCLCPP_INFO(node->get_logger(), "att_profile.rda_x           : %f", rda_.x());
	RCLCPP_INFO(node->get_logger(), "att_profile.rda_y           : %f", rda_.y());
	RCLCPP_INFO(node->get_logger(), "att_profile.rda_z           : %f", rda_.z());
	RCLCPP_INFO(node->get_logger(), "att_profile.rda_w           : %f", rda_.w());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis1.x         : %f", scanAxes_.at(0).x());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis1.y         : %f", scanAxes_.at(0).y());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis1.z         : %f", scanAxes_.at(0).z());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis2.x         : %f", scanAxes_.at(1).x());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis2.y         : %f", scanAxes_.at(1).y());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis2.z         : %f", scanAxes_.at(1).z());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis3.x         : %f", scanAxes_.at(2).x());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis3.y         : %f", scanAxes_.at(2).y());
	RCLCPP_INFO(node->get_logger(), "att_profile.scan.axis3.z         : %f", scanAxes_.at(2).z());
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::AttProfiler::~AttProfiler() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::AttProfiler::AttProfiler
(const AttProfiler&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::AttProfiler&
ib2::AttProfiler::operator=(const AttProfiler&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::AttProfiler::AttProfiler(AttProfiler&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::AttProfiler&
ib2::AttProfiler::operator=(AttProfiler&&) = default;

//------------------------------------------------------------------------------
//  姿勢プロファイル最大トルクの取得
double ib2::AttProfiler::Tmax() const
{
	return Tmax_;
}
//------------------------------------------------------------------------------
//  姿勢プロファイル最大角速度の取得
double ib2::AttProfiler::wmax() const
{
	return wmax_;
}

//------------------------------------------------------------------------------
//  角速度制御を切る閾値の取得
double ib2::AttProfiler::qthr() const
{
	return qthr_;
}

//------------------------------------------------------------------------------
// 回転角微小数の取得
double ib2::AttProfiler::epsQm() const
{
	return epsQm_;
}

//------------------------------------------------------------------------------
// AIAの参照
const Eigen::Quaterniond& ib2::AttProfiler::aia() const
{
	return aia_;
}

//------------------------------------------------------------------------------
// RDAの参照
const Eigen::Quaterniond& ib2::AttProfiler::rda() const
{
	return rda_;
}

//------------------------------------------------------------------------------
// スキャン軸の参照
const std::vector<Eigen::Vector3d>& ib2::AttProfiler::scanAxes() const
{
	return scanAxes_;
}

// End Of File -----------------------------------------------------------------
