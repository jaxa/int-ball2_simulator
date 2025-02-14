
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
kp_(1.), ki_(1.), kd_(1.), Fmax_(1.), s_(Eigen::Vector3d::Zero()), ts_()
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::PosController::PosController(const ros::NodeHandle& nh) :
	s_(Eigen::Vector3d::Zero()), ts_()
{
	using namespace ib2_mss;

	double kp(-1.);
	double ki(-1.);
	double kd(-1.);
	double Fmax(-1.);
	
	nh.getParam("/pos_ctl/kp"     , kp);
	nh.getParam("/pos_ctl/ki"     , ki);
	nh.getParam("/pos_ctl/kd"     , kd);
	nh.getParam("/pos_ctl/fi_max" , Fmax);
	RangeCheckerD::notNegative(kp, true, "kp");
	RangeCheckerD::notNegative(ki, true, "ki");
	RangeCheckerD::notNegative(kd, true, "kd");
	RangeCheckerD::notNegative(Fmax, true, "Fmax");
	kp_ = kp;
	ki_ = ki;
	kd_ = kd;
	Fmax_ = Fmax;

	ROS_INFO("******** Set Parameters in pos_controller.cpp");
	ROS_INFO("/pos_ctl/kp       : %f", kp);
	ROS_INFO("/pos_ctl/ki       : %f", ki);
	ROS_INFO("/pos_ctl/kd       : %f", kd);
	ROS_INFO("/pos_ctl/fi_max   : %f", Fmax);

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
	ts_ = ros::Time();
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
(const ros::Time& t, const Eigen::Vector3d &r,const Eigen::Vector3d& v,
 const Eigen::Quaterniond& q, const CtlElements &p, double m)
{
	// 誤差量計算
	Eigen::Vector3d re(r - p.r());
	Eigen::Vector3d ve(v - p.v());

	// 積分量計算と飽和処理
	auto dt(t - ts_);
	s_ = s_ + re * dt.toSec();
	s_ = saturation(s_, Fmax_);
	ts_ = t;

	//位置制御則
	Eigen::Vector3d a(p.a() - kp_ * re - ki_ * s_ - kd_ * ve);
	return q.conjugate() * a * m;
}

// End Of File -----------------------------------------------------------------
