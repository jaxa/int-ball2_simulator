
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
// パラメータによるコンストラクタ
ib2::AttController::AttController(rclcpp::Node* node)
{
	using namespace ib2_mss;

	auto get_param = [node](const std::string& name, auto& value) {
		using T = std::decay_t<decltype(value)>;
		if (!node->has_parameter(name)) {
			node->declare_parameter<T>(name, value);
		}
		node->get_parameter(name, value);
	};

	double kp(-1.);
	double kd(-1.);
	get_param("att_ctl.kp", kp);
	get_param("att_ctl.kd", kd);
	RangeCheckerD::notNegative(kp, true, "kp");
	RangeCheckerD::notNegative(kd, true, "kd");
	kp_ = kp;
	kd_ = kd;

	RCLCPP_INFO(node->get_logger(), "******** Set Parameters in att_controller.cpp");
	RCLCPP_INFO(node->get_logger(), "att_ctl.kp   : %f", kp_);
	RCLCPP_INFO(node->get_logger(), "att_ctl.kd   : %f", kd_);
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
