
#include "ctl/thrust_allocator.h"

#include "guidance_control_common/Function1.h"
#include "guidance_control_common/Solver1.h"
#include "guidance_control_common/Log.h"
#include "guidance_control_common/RangeChecker.h"

#include <vector>
#include <stdexcept>
#include <algorithm>
#include <string>

//------------------------------------------------------------------------------
// 定数
namespace
{
	/** 制御変数の数（力３成分トルク３成分合計６成分） */
	const Eigen::Index N_CONTROL(6);

	/**
	 * 推力配分結果とスラスタの最大推力との差を計算する。
	 */
	class FmaxDif : public ib2_mss::Function1
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	private:
		/** デフォルトコンストラクタ. */
		FmaxDif() = delete;
		
	public:
		/** コンストラクタ.
		 * @param [in] Fmax 各ファンの最大推力
		 * @param [in] Wp 正方向の配列行列
		 * @param [in] Wm 負方向の配列行列
		 * @param [in] y 最大推力トルク基準値
		 * @param [in] eta スラスタ最大推力レート
		 */
		FmaxDif
		(double Fmax, const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm, const Eigen::VectorXd& fan_null,
		 const Eigen::VectorXd& y, double eta) :
		ib2_mss::Function1(), a_(Fmax, Wp, Wm, fan_null), y_(y), Fmax_(eta * Fmax)
		{
		}
		
		/** デストラクタ. */
		~FmaxDif() = default;
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		FmaxDif(const FmaxDif&) = delete;
		
		/** コピー代入演算子. */
		FmaxDif& operator=(const FmaxDif&) = delete;
		
		/** ムーブコンストラクタ. */
		FmaxDif(FmaxDif&&) = delete;
		
		/** ムーブ代入演算子. */
		FmaxDif& operator=(FmaxDif&&) = delete;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 一次元関数の評価.
		 * 推力配分結果とスラスタの最大推力との差を計算する。
		 * @param [in] p 最大推力倍率
		 * @return 推力配分結果とスラスタの最大推力との差
		 */
		double operator()(double p) const
		{
			Eigen::VectorXd y(y_ * p);
			Eigen::VectorXd z(a_.allocate(y));
			return z.maxCoeff() - Fmax_;
		}
		
		/** 一次元関数の導関数.
		 * @return 導関数評価結果
		 */
		double d(double) const
		{
			throw std::domain_error("not defined.");
		}
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 推力配分 */
		ib2::ThrustAllocator a_;

		/** 最大推力トルク基準値 */
		Eigen::VectorXd y_;

		/** 実効最大推力 */
		double Fmax_;
	};

	/** 配分行列の確認
	 * @param [in] Wp 正方向の配列行列
	 * @param [in] Wm 負方向の配列行列
	 */
	void checkW(const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm)
	{
		if (Wp.cols() != N_CONTROL ||
			Wm.cols() != N_CONTROL ||
			Wp.rows() != Wm.rows())
		{
			throw std::domain_error("invalid Wp and Wm Matrix Size");
		}
	}
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::ThrustAllocator::ThrustAllocator() :
Fmax_(0), 
Wp_(Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL)), 
Wm_(Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL)),
fan_null_(Eigen::VectorXd::Ones(8))
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::ThrustAllocator::ThrustAllocator(const ros::NodeHandle& nh)
{
	using namespace ib2_mss;
	static const RangeCheckerD F_MAX_RANGE
	(RangeCheckerD::TYPE::GT_LE, 0., 0.1, true);

	nh.getParam("/fan/Fmax", Fmax_);
	F_MAX_RANGE.valid(Fmax_, "/fan/Fmax");

	int nfan(0);
	nh.getParam("/fan/number", nfan);
	if (nfan != 8)
		throw std::domain_error("/fan/number must be 8");
	nfan_ = nfan;
	Wp_.resize(nfan, N_CONTROL);
	Wm_.resize(nfan, N_CONTROL);
	fan_null_.resize(nfan);

	std::string str_wp = "fan/Wp/fan0";
	std::string str_wm = "fan/Wm/fan0";

	for(int i = 0; i < 8; i++)
	{
		std::string str_fx = str_wp + std::to_string(i + 1) + "/Fx";
		std::string str_fy = str_wp + std::to_string(i + 1) + "/Fy";
		std::string str_fz = str_wp + std::to_string(i + 1) + "/Fz";
		std::string str_tx = str_wp + std::to_string(i + 1) + "/Tx";
		std::string str_ty = str_wp + std::to_string(i + 1) + "/Ty";
		std::string str_tz = str_wp + std::to_string(i + 1) + "/Tz";

		nh.getParam(str_fx, Wp_(i,0));
		nh.getParam(str_fy, Wp_(i,1));
		nh.getParam(str_fz, Wp_(i,2));
		nh.getParam(str_tx, Wp_(i,3));
		nh.getParam(str_ty, Wp_(i,4));
		nh.getParam(str_tz, Wp_(i,5));
	}

	for(int i = 0; i < 8; i++)
	{
		std::string str_fx = str_wm + std::to_string(i + 1) + "/Fx";
		std::string str_fy = str_wm + std::to_string(i + 1) + "/Fy";
		std::string str_fz = str_wm + std::to_string(i + 1) + "/Fz";
		std::string str_tx = str_wm + std::to_string(i + 1) + "/Tx";
		std::string str_ty = str_wm + std::to_string(i + 1) + "/Ty";
		std::string str_tz = str_wm + std::to_string(i + 1) + "/Tz";

		nh.getParam(str_fx, Wm_(i,0));
		nh.getParam(str_fy, Wm_(i,1));
		nh.getParam(str_fz, Wm_(i,2));
		nh.getParam(str_tx, Wm_(i,3));
		nh.getParam(str_ty, Wm_(i,4));
		nh.getParam(str_tz, Wm_(i,5));
	}
	
	RangeCheckerD::notNegative(Wp_.minCoeff(), true, "Wp");
	RangeCheckerD::notNegative(Wm_.minCoeff(), true, "Wm");

	nh.getParam("/fan/null_ratio/fan01", fan_null_(0));
	nh.getParam("/fan/null_ratio/fan02", fan_null_(1));
	nh.getParam("/fan/null_ratio/fan03", fan_null_(2));
	nh.getParam("/fan/null_ratio/fan04", fan_null_(3));
	nh.getParam("/fan/null_ratio/fan05", fan_null_(4));
	nh.getParam("/fan/null_ratio/fan06", fan_null_(5));
	nh.getParam("/fan/null_ratio/fan07", fan_null_(6));
	nh.getParam("/fan/null_ratio/fan08", fan_null_(7));

	RangeCheckerD::positive(fan_null_.minCoeff(), true, "fan_null");

	ROS_INFO("******** Set Parameters in thrust_allocator.cpp");
	ROS_INFO("/fan/Fmax      : %f", Fmax_);
	ROS_INFO("/fan/number    : %d", nfan);

	for(int i = 0; i < 8; i++)
	{
		std::string str_fx = str_wp + std::to_string(i + 1) + "/Fx";
		std::string str_fy = str_wp + std::to_string(i + 1) + "/Fy";
		std::string str_fz = str_wp + std::to_string(i + 1) + "/Fz";
		std::string str_tx = str_wp + std::to_string(i + 1) + "/Tx";
		std::string str_ty = str_wp + std::to_string(i + 1) + "/Ty";
		std::string str_tz = str_wp + std::to_string(i + 1) + "/Tz";

		ROS_INFO("%s     : %f", str_fx.c_str(), Wp_(i,0));
		ROS_INFO("%s     : %f", str_fy.c_str(), Wp_(i,1));
		ROS_INFO("%s     : %f", str_fz.c_str(), Wp_(i,2));
		ROS_INFO("%s     : %f", str_tx.c_str(), Wp_(i,3));
		ROS_INFO("%s     : %f", str_ty.c_str(), Wp_(i,4));
		ROS_INFO("%s     : %f", str_tz.c_str(), Wp_(i,5));
	}

	for(int i = 0; i < 8; i++)
	{
		std::string str_fx = str_wm + std::to_string(i + 1) + "/Fx";
		std::string str_fy = str_wm + std::to_string(i + 1) + "/Fy";
		std::string str_fz = str_wm + std::to_string(i + 1) + "/Fz";
		std::string str_tx = str_wm + std::to_string(i + 1) + "/Tx";
		std::string str_ty = str_wm + std::to_string(i + 1) + "/Ty";
		std::string str_tz = str_wm + std::to_string(i + 1) + "/Tz";

		ROS_INFO("%s     : %f", str_fx.c_str(), Wm_(i,0));
		ROS_INFO("%s     : %f", str_fy.c_str(), Wm_(i,1));
		ROS_INFO("%s     : %f", str_fz.c_str(), Wm_(i,2));
		ROS_INFO("%s     : %f", str_tx.c_str(), Wm_(i,3));
		ROS_INFO("%s     : %f", str_ty.c_str(), Wm_(i,4));
		ROS_INFO("%s     : %f", str_tz.c_str(), Wm_(i,5));
	}

	ROS_INFO("/fan/null_ratio/fan01   : %f",fan_null_(0));
	ROS_INFO("/fan/null_ratio/fan02   : %f",fan_null_(1));
	ROS_INFO("/fan/null_ratio/fan03   : %f",fan_null_(2));
	ROS_INFO("/fan/null_ratio/fan04   : %f",fan_null_(3));
	ROS_INFO("/fan/null_ratio/fan05   : %f",fan_null_(4));
	ROS_INFO("/fan/null_ratio/fan06   : %f",fan_null_(5));
	ROS_INFO("/fan/null_ratio/fan07   : %f",fan_null_(6));
	ROS_INFO("/fan/null_ratio/fan08   : %f",fan_null_(7));
}

//------------------------------------------------------------------------------
// 値によるコンストラクタ
ib2::ThrustAllocator::ThrustAllocator
(double Fmax, const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm, const Eigen::VectorXd& fan_null) :
Fmax_(Fmax), Wp_(Wp), Wm_(Wm), fan_null_(fan_null)
{
	checkW(Wp, Wm);
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::ThrustAllocator::~ThrustAllocator() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::ThrustAllocator::ThrustAllocator(const ThrustAllocator&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::ThrustAllocator&
ib2::ThrustAllocator::operator=(const ThrustAllocator&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::ThrustAllocator::ThrustAllocator(ThrustAllocator&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::ThrustAllocator&
ib2::ThrustAllocator::operator=(ThrustAllocator&&) = default;

//------------------------------------------------------------------------------
// メンバ変数の設定
ib2::ThrustAllocator& ib2::ThrustAllocator::set
(double Fmax, const Eigen::MatrixXd& Wp, const Eigen::MatrixXd& Wm)
{
	checkW(Wp, Wm);
	Fmax_ = Fmax;
	Wp_ = Wp;
	Wm_ = Wm;
	return *this;
}

//------------------------------------------------------------------------------
// ファンの数の取得
int ib2::ThrustAllocator::nfan() const
{
	return nfan_;
}

//------------------------------------------------------------------------------
// ファン推力最大値の取得
double ib2::ThrustAllocator::Fmax() const
{
	return Fmax_;
}

//------------------------------------------------------------------------------
// 推力配分
Eigen::VectorXd ib2::ThrustAllocator::allocate
(const Eigen::Vector3d& F, const Eigen::Vector3d& T) const
{
	Eigen::VectorXd y(N_CONTROL);
	y << F, T;
	return allocate(y);
}

//------------------------------------------------------------------------------
// 推力配分
Eigen::VectorXd ib2::ThrustAllocator::allocate(const Eigen::VectorXd& y) const
{
	Eigen::VectorXd pp(y.size());
	Eigen::VectorXd pm(y.size());
	for (Eigen::Index i = 0; i < y.size(); ++i)
	{
		pp(i) = y(i) < 0. ? 0. : std::abs(y(i));
		pm(i) = y(i) > 0. ? 0. : std::abs(y(i));
	}
	Eigen::VectorXd f(Wp_ * pp + Wm_ * pm);
	Eigen::VectorXd fn(f.size());
	fn = f.array() / fan_null_.array();
	f = f.array() - fn.minCoeff() * fan_null_.array();
	return f;
}

//------------------------------------------------------------------------------
// 実効最大推力の計算
double ib2::ThrustAllocator::effectiveFmax
(double Fmax, const Eigen::Vector3d& dir, double eta) const
{
	Eigen::Vector3d F(dir * Fmax);
	Eigen::Vector3d T(Eigen::Vector3d::Zero());
	Eigen::VectorXd y(N_CONTROL);
	y << F, T;
	return Fmax * effective(y, eta);
}

//------------------------------------------------------------------------------
// 実効最大トルクの計算
double ib2::ThrustAllocator::effectiveTmax
(double Tmax, const Eigen::Vector3d& axis, double eta) const
{
	Eigen::Vector3d F(Eigen::Vector3d::Zero());
	Eigen::Vector3d T(axis * Tmax);
	Eigen::VectorXd y(N_CONTROL);
	y << F, T;
	return Tmax * effective(y, eta);
}

//------------------------------------------------------------------------------
// 実効最大推力・トルクの計算
std::pair<double,double> ib2::ThrustAllocator::effectiveFTmax
(double Fmax, const Eigen::Vector3d& dir, 
 double Tmax, const Eigen::Vector3d& axis, double eta) const
{
	Eigen::Vector3d F(dir * Fmax);
	Eigen::Vector3d T(axis * Tmax);
	Eigen::VectorXd y(N_CONTROL);
	y << F, T;
	auto p(effective(y, eta));
	return std::make_pair(p * Fmax, p * Tmax);
}

//------------------------------------------------------------------------------
// 実効レートの計算
double ib2::ThrustAllocator::effective
(const Eigen::VectorXd& y, double eta) const
{
	/** 独立収束判定許容値 */
	static const double P_TOL(1.e-6);
	/** 従属収束判定許容値 */
	static const double D_TOL(1.e-6);

	ib2_mss::Solver1 solver(P_TOL, D_TOL);
	FmaxDif fdif(Fmax_, Wp_, Wm_, fan_null_, y, eta);
	return solver.brent(fdif, 0., 2.);
}

// End Of File -----------------------------------------------------------------
