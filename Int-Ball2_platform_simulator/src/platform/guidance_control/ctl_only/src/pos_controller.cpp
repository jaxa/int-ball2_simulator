
#include "ctl/pos_controller.h"
#include "guidance_control_common/RangeChecker.h"

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 飽和処理
	 * @param [in] x  飽和処理対象
	 * @param [in] amax 飽和判定値
	 * @retval        飽和処理後の値
	 */
	Eigen::VectorXd saturation(const Eigen::VectorXd &x, const double amax)
	{
		Eigen::VectorXd y = x;
		size_t num = y.array().size();
		for (size_t i = 0; i < num; i++)
			y(i) = y(i) > amax ? amax : (y(i) < -amax ? -amax : y(i));
		return y;
	}
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::PosController::PosController() :
kp_(1.), ki_(1.), kd_(1.), Fmax_(1.), s_(Eigen::Vector3d::Zero()), ts_(0, 0)
{
}

//------------------------------------------------------------------------------
// パラメータによるコンストラクタ
ib2::PosController::PosController(rclcpp::Node* node) :
	s_(Eigen::Vector3d::Zero()), ts_(0, 0)
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
	double ki(-1.);
	double kd(-1.);
	double Fmax(-1.);

	get_param("pos_ctl.kp", kp);
	get_param("pos_ctl.ki", ki);
	get_param("pos_ctl.kd", kd);
	get_param("pos_ctl.fi_max", Fmax);
	RangeCheckerD::notNegative(kp, true, "kp");
	RangeCheckerD::notNegative(ki, true, "ki");
	RangeCheckerD::notNegative(kd, true, "kd");
	RangeCheckerD::notNegative(Fmax, true, "Fmax");
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	Fmax_ = Fmax;

	RCLCPP_INFO(node->get_logger(), "******** Set Parameters in pos_controller.cpp");
	RCLCPP_INFO(node->get_logger(), "pos_ctl.kp       : %f", kp);
	RCLCPP_INFO(node->get_logger(), "pos_ctl.ki       : %f", ki);
	RCLCPP_INFO(node->get_logger(), "pos_ctl.kd       : %f", kd);
	RCLCPP_INFO(node->get_logger(), "pos_ctl.fi_max   : %f", Fmax);

}

//------------------------------------------------------------------------------
// デストラクタ
ib2::PosController::~PosController() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::PosController::PosController(const PosController&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::PosController&
ib2::PosController::operator=(const PosController&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::PosController::PosController(PosController&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::PosController& ib2::PosController::operator=(PosController&&) = default;

//------------------------------------------------------------------------------
// 積分量のクリア
void ib2::PosController::flash()
{
	s_ = Eigen::Vector3d::Zero();
	ts_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

//------------------------------------------------------------------------------
//  比例ゲインの取得
double ib2::PosController::kp() const
{
	return kp_;
}

//------------------------------------------------------------------------------
//  積分ゲインの取得
double ib2::PosController::ki() const
{
	return ki_;
}

//------------------------------------------------------------------------------
//  微分ゲインの取得
double ib2::PosController::kd() const
{
	return kd_;
}

//------------------------------------------------------------------------------
//  位置制御最大推力の取得
double ib2::PosController::Fmax() const
{
	return Fmax_;
}

//------------------------------------------------------------------------------
// 力コマンド計算
Eigen::Vector3d ib2::PosController::forceCommand
(const rclcpp::Time& t, const Eigen::Vector3d &r,const Eigen::Vector3d& v,
 const Eigen::Quaterniond& q, const CtlElements &p, double m)
{
	// 誤差量計算
	Eigen::Vector3d re(r - p.r());
	Eigen::Vector3d ve(v - p.v());

	// 積分量計算と飽和処理
	auto dt(t - ts_);
	s_ = s_ + re * dt.seconds();
	s_ = saturation(s_, Fmax_);
	ts_ = t;

	//位置制御則
	Eigen::Vector3d a(p.a() - kp_ * re - ki_ * s_ - kd_ * ve);
	return q.conjugate() * a * m;
}

// End Of File -----------------------------------------------------------------
