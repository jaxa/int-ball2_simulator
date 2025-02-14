
#include "guidance_control_common/Utility.h"
#include "guidance_control_common/IntOverflow.h"
#include "guidance_control_common/RangeChecker.h"
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/Log.h"

#include <Eigen/Geometry>

#include <vector>
#include <string>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <cstdlib>
#include <iomanip>

//------------------------------------------------------------------------------
// 入力値を周期的な範囲内に修正
double ib2_mss::Utility::cyclicRange(double x, double min, double max)
{
	if ((min >= max) || (min <= x && x < max))
		return x;
	
	double d(x   - min);
	double w(max - min);
	double r(fmod(d, w));
	if (r < 0.)
		r += w;
	return r + min;
}

//------------------------------------------------------------------------------
// 多項式の計算
double ib2_mss::Utility::polynomial(double x, const std::vector<double>& c)
{
	double y(c.back());
	for (auto it = c.rbegin() + 1; it != c.rend(); ++it)
 	{
		y *= x;
		y += *it;
	}
	return y;
}

//------------------------------------------------------------------------------
// dB値計算
double ib2_mss::Utility::dB(double r)
{
	if (r <= std::numeric_limits<double>::min())
		r  = std::numeric_limits<double>::min();
	return 10. * log10(r);
}

//------------------------------------------------------------------------------
// 真数計算
double ib2_mss::Utility::dBinverse(double dB)
{
	return pow(10., 0.1 * dB);
}

//------------------------------------------------------------------------------
// １区間の台形積分
double ib2_mss::Utility::trapz(double x0, double x1, double y0, double y1)
{
	return (y1 + y0) * std::abs(x1 - x0) * 0.5;
}

//------------------------------------------------------------------------------
// 台形積分
std::vector<double> ib2_mss::Utility::trapz
(const std::vector<double>& x, const std::vector<double>& y)
{
	// 各区間の台形面積
	std::vector<double> s;
	s.reserve(x.size());
	for (size_t i = 1; i < x.size(); ++i)
		s.push_back(trapz(x.at(i-1), x.at(i), y.at(i-1), y.at(i)));
	auto smax(std::max_element(s.begin(), s.end()));
	std::vector<double> sum;
	sum.reserve(x.size());
	sum.push_back(0.);
	for (auto& si: s)
	{
		if (std::abs(si / *smax) < std::numeric_limits<double>::epsilon())
			si = 0.;
		sum.push_back(sum.back() + si);
	}
	return sum;
}

//------------------------------------------------------------------------------
// 二円の交わる面積
double ib2_mss::Utility::area2c(double ra, double rb, double ab)
{
	if (ra + rb <= ab)
		return 0.;				///< 二円が交わらない
	else if (ra - rb >= ab)
		return rb * rb * M_PI;	///< 円Bが円Aに含まれるので円Bの面積
	else if (rb - ra >= ab)
		return ra * ra * M_PI;	///< 円Aが円Bに含まれるので円Aの面積
	
	Eigen::Vector2d pa(0., 0.);
	Eigen::Vector2d pb(ab, 0.);
	Eigen::Vector2d pc(intersection2c(ra, pa, rb, pb));
	double a(atan2(pc.y(),      pc.x()));
	double b(atan2(pc.y(), ab - pc.x()));
	return ra * ra * a + rb * rb * b - ab * pc.y();
}

//------------------------------------------------------------------------------
// 三円の交わる面積
double ib2_mss::Utility::area3c
(double ra, double rb, double rc, double ab, double bc, double ca)
{
	if (ra + rb <= ab || rb + rc <= bc || rc + ra <= ca)
		return 0.;								///< 三円の交わる領域がない
	else if (rc - ra >= ca || rc - rb >= bc)	///< 円AorBが円Cに含まれる
		return area2c(ra, rb, ab);				///< 円A, Bの交わる面積
	else if (ra - rb >= ab || ra - rc >= ca)	///< 円BorCが円Aに含まれる
		return area2c(rb, rc, bc);				///< 円B, Cの交わる面積
	else if (rb - rc >= bc || rb - ra >= ab)	///< 円CorAが円Bに含まれる
		return area2c(rc, ra, ca);				///< 円C, Aの交わる面積
	
	/// 円A,B,Cの中心位置
	Eigen::Vector2d pa(0., 0.);
	Eigen::Vector2d pb(ab, 0.);
	Eigen::Vector2d pc(intersection2c(ca, pa, bc, pb));
	
	/// 円AB,BC,CAの交点D,E,Fの位置
	Eigen::Vector2d pd1(intersection2c(ra, pa, rb, pb, false));
	if ((pd1 - pc).norm() <= rc)	///< 円A,Bの重なりが円Cに含まれる
		return area2c(ra, rb, ab);
	Eigen::Vector2d pe1(intersection2c(rb, pb, rc, pc, false));
	if ((pe1 - pa).norm() <= ra)	///< 円B,Cの重なりが円Aに含まれる
		return area2c(rb, rc, bc);
	Eigen::Vector2d pf1(intersection2c(rc, pc, ra, pa, false));
	if ((pf1 - pb).norm() <= rb)	///< 円C,Aの重なりが円Bに含まれる
		return area2c(rc, ra, ca);
	
	Eigen::Vector2d pd(intersection2c(ra, pa, rb, pb, true));
	Eigen::Vector2d pe(intersection2c(rb, pb, rc, pc, true));
	Eigen::Vector2d pf(intersection2c(rc, pc, ra, pa, true));
	if ((pd - pc).norm() >= rc ||	///< 円A,Bの重なりが円Cに含まれない
		(pe - pa).norm() >= ra ||	///< 円B,Cの重なりが円Aに含まれない
		(pf - pb).norm() >= rb)		///< 円C,Aの重なりが円Bに含まれない
		return 0.;
	
	Eigen::Vector2d bd((pd - pb).normalized());
	Eigen::Vector2d be((pe - pb).normalized());
	Eigen::Vector2d ce((pe - pc).normalized());
	Eigen::Vector2d cf((pf - pc).normalized());
	Eigen::Vector2d af((pf - pa).normalized());
	Eigen::Vector2d ad((pd - pa).normalized());
	double cosb(bd.dot(be));
	double cosc(ce.dot(cf));
	double cosa(af.dot(ad));
	double sinb(bd.x() * be.y() - bd.y() * be.x());
	double sinc(ce.x() * cf.y() - ce.y() * cf.x());
	double sina(af.x() * ad.y() - af.y() * ad.x());
	
	double a(cyclicRange(atan2(sina, cosa), 0., TWOPI));
	double b(cyclicRange(atan2(sinb, cosb), 0., TWOPI));
	double c(cyclicRange(atan2(sinc, cosc), 0., TWOPI));
	double TA(areaTriangle(pa, pf, pd) * (a > M_PI ? -1. : 1.));
	double TB(areaTriangle(pb, pd, pe) * (b > M_PI ? -1. : 1.));
	double TC(areaTriangle(pc, pe, pf) * (c > M_PI ? -1. : 1.));
	
	double A(ra * ra * a * 0.5 - TA);
	double B(rb * rb * b * 0.5 - TB);
	double C(rc * rc * c * 0.5 - TC);
	
	return A + B + C + areaTriangle(pd, pe, pf);
}

//------------------------------------------------------------------------------
// 円の交点
Eigen::Vector2d ib2_mss::Utility::intersection2c
(double ra, const Eigen::Vector2d& pa,
 double rb, const Eigen::Vector2d& pb, bool ccw)
{
	Eigen::Vector2d ab(pb - pa);
	double dab(ab.norm());
	try
	{
		if (ra + rb <= dab || std::abs(ra - rb) >= dab)
			throw std::domain_error("circle A don't intersect circle B");
		double xc((dab + (ra + rb) * (ra - rb) / dab) * 0.5);
		double yc(sqrt((ra + xc) * (ra - xc)) * (ccw ? 1. : -1.));
		Eigen::Vector2d pc(xc, yc);
		Eigen::Rotation2Dd r(atan2(ab.y(), ab.x()));
		return std::move(pa + r * pc);
	}
	catch (const std::exception& e)
	{
		std::ostringstream ss;
		ss << "caught exception : "<< e.what();
		ss << ", ra:" << ra << ", rb:" << rb << ", ab:" << dab;
		ss << ", pa:" << pa.transpose() << ", pb:" << pb.transpose();
		LOG_ERROR(ss.str());
		throw;
	}
}

//------------------------------------------------------------------------------
// 三角形の面積
double ib2_mss::Utility::areaTriangle
(const Eigen::Vector2d& pa, const Eigen::Vector2d& pb,
 const Eigen::Vector2d& pc)
{
	Eigen::Vector2d ab(pb - pa);
	Eigen::Vector2d ac(pc - pa);
	return std::abs(ab.x() * ac.y() - ab.y() * ac.x()) * 0.5;
}

//------------------------------------------------------------------------------
// Levenberg Marquardt法の状態の確認
void ib2_mss::Utility::checkStatusLM
(const Eigen::LevenbergMarquardtSpace::Status& status,
 const std::string& file, const std::string& function, unsigned long lineno)
{
	static const std::vector<std::string> LM_STATUS
	{
		"NotStarted",
		"Running",
		"ImproperInputParameters",
		"RelativeReductionTooSmall",
		"RelativeErrorTooSmall",
		"RelativeErrorAndReductionTooSmall",
		"CosinusTooSmall",
		"TooManyFunctionEvaluation",
		"FtolTooSmall",
		"XtolTooSmall",
		"GtolTooSmall",
		"UserAsked"
	};
	std::string what(LM_STATUS.at(static_cast<size_t>(status + 2)));
	std::ostringstream ss;
	ss << "Eigen::LevenbergMarquardtSpace::Status : " << status;
	ss << " (" << what << ")";
	if (status <= Eigen::LevenbergMarquardtSpace::ImproperInputParameters)
	{
		Log::error(ss.str(), file, function, lineno);
		throw std::domain_error(what);
	}
	else if (status >= Eigen::LevenbergMarquardtSpace::CosinusTooSmall)
		Log::warn(ss.str(), file, function, lineno);
	else
		Log::debug(ss.str(), file, function, lineno);
}

//------------------------------------------------------------------------------
// カウンタ文字列の作成
std::string ib2_mss::Utility::counterString(size_t count, int digits, char filler)
{
	std::ostringstream ss;
	ss << std::setw(digits) << std::setfill(filler) << count;
	return ss.str();
}

//------------------------------------------------------------------------------
// 回帰平均
double ib2_mss::Utility::recmean(size_t& n, double m, double c)
{
	double n0(static_cast<double>(n));
	double n1(static_cast<double>(++n));
	return m * (n0 / n1) + c / n1;
}

//------------------------------------------------------------------------------
// 整数指数のべき乗
template <typename T>
T ib2_mss::Utility::powint(T base, int32_t e)
{
	if (e <= 0)
		return 1;
	auto h(e / 2);
	auto r(Utility::powint(base, h));
	auto l(r);
	if (e - h * 2 > 0)
	{
		MULTIPLIABLE(l, base, true);
		l *= base;
	}
	MULTIPLIABLE(l, r, true);
	return l * r;
}

//------------------------------------------------------------------------------
// 整数への変換
template <typename T>
int64_t ib2_mss::Utility::int64(T o, bool round)
{
	static const int64_t BASE(10);
	RangeChecker<T> int64range
	(RangeChecker<T>::TYPE::GE_LE,
	 static_cast<T>(std::numeric_limits<int64_t>::min()),
	 static_cast<T>(std::numeric_limits<int64_t>::max()), true);

	auto eps(std::abs(o) * std::numeric_limits<T>::epsilon());
	auto digits(static_cast<int32_t>(std::floor(std::log10(eps))));
	auto denom(Utility::powint(BASE, digits));
	o /= static_cast<T>(denom);
	if (round)
		o = std::round(o);
	int64range.valid(o);
	auto oi(static_cast<int64_t>(o));
	MULTIPLIABLE(oi, denom, true);
	return oi * denom;
}

//------------------------------------------------------------------------------
// 明示的テンプレートのインスタンス化
template int16_t ib2_mss::Utility::powint(int16_t base, int32_t e);
template int32_t ib2_mss::Utility::powint(int32_t base, int32_t e);
template int64_t ib2_mss::Utility::powint(int64_t base, int32_t e);
template uint16_t ib2_mss::Utility::powint(uint16_t base, int32_t e);
template uint32_t ib2_mss::Utility::powint(uint32_t base, int32_t e);
template uint64_t ib2_mss::Utility::powint(uint64_t base, int32_t e);

template int64_t ib2_mss::Utility::int64(float o, bool round);
template int64_t ib2_mss::Utility::int64(double o, bool round);
template int64_t ib2_mss::Utility::int64(long double o, bool round);

// End Of File -----------------------------------------------------------------
