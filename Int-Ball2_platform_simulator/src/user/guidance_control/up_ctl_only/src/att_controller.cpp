
#include "ctl/att_controller.h"
#include "guidance_control_common/RangeChecker.h"

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 符号処理
	 * @param [in] a 検査値
	 * @retval 1. aがゼロ以上
	 * @retval -1. aが負
	 */
	double sign(double a)
	{
		return a >= 0 ? 1. : -1.;
	}
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::AttController::AttController() :
kp_(1.), kd_(1.)
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::AttController::AttController(const ros::NodeHandle& nh)
{
	using namespace ib2_mss;
	
	double kp(-1.);
	double kd(-1.);
	nh.getParam("/att_ctl/kp"   , kp);
	nh.getParam("/att_ctl/kd"   , kd);
	RangeCheckerD::notNegative(kp, true, "kp");
	RangeCheckerD::notNegative(kd, true, "kd");
	kp_ = kp;
	kd_ = kd;

	ROS_INFO("******** Set Parameters in att_controller.cpp");
	ROS_INFO("/att_ctl/kp   : %f", kp_);
	ROS_INFO("/att_ctl/kd   : %f", kd_);
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::AttController::~AttController() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::AttController::AttController(const AttController&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::AttController&
ib2::AttController::operator=(const AttController&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::AttController::AttController(AttController&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::AttController& ib2::AttController::operator=(AttController&&) = default;

//------------------------------------------------------------------------------
//  比例ゲインの取得
double ib2::AttController::kp() const
{
	return kp_;
}

//------------------------------------------------------------------------------
//  微分ゲインの取得
double ib2::AttController::kd() const
{
	return kd_;
}

//------------------------------------------------------------------------------
// トルクコマンド計算
Eigen::Vector3d ib2::AttController::torqueCommand
(const Eigen::Quaterniond &q, const Eigen::Vector3d &w, const CtlElements &p,
 const Eigen::Matrix3d& Is) const
{
	// 誤差量計算
	Eigen::Quaterniond qe = p.q().conjugate()*q;

	// ωc計算
	Eigen::Vector3d    wc    = -2.0 * kp_ / kd_ * sign(qe.w())
			* qe.vec() + qe.conjugate() * p.w();

	// 姿勢制御則
	Eigen::Vector3d troque = kd_ * Is * (wc - w) + w.cross(Is * w);

	return troque;
}
// End Of File -----------------------------------------------------------------
