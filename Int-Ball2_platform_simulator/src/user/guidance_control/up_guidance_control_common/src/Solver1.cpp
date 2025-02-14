
#include "guidance_control_common/Solver1.h"
#include "guidance_control_common/Log.h"

#include <string>
#include <sstream>
#include <stdexcept>
#include <cmath>
#include <limits>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 収束しなかった文字列 */
	const std::string MAX_ITERATION("Maximum number of iterations exceeded.");
	
	/** Brent法のデバッグ文字列出力
	 * @param [in] it 反復回数
	 * @param [in] a 独立変数1
	 * @param [in] b 独立変数2
	 * @param [in] fa 従属変数1
	 * @param [in] fb 従属変数2
	 * @return デバッグ文字列
	 */
	std::string debugBrent(size_t it, double a, double b, double fa, double fb)
	{
		std::stringstream ss;
		ss.precision(16);
		ss << "iteration=" << it << ", {a,b}={" << a << ", " << b;
		ss << "}, {f(a),f(b)}={" << fa << ", " << fb << "}";
		return ss.str();
	}
	
	/** Newton法のデバッグ文字列出力
	 * @param [in] it 反復回数
	 * @param [in] x 独立変数
	 * @param [in] fx 従属変数
	 * @param [in] fdx 導関数
	 * @param [in] xname 独立変数名
	 * @return デバッグ文字列
	 */
	std::string debugNewton
	(size_t it, double x, double fx, double fdx, const std::string& xname = "x")
	{
		std::stringstream ss;
		ss.precision(16);
		ss << "iteration=" << it << ", " << xname << "=" << x;
		ss << ", f(" << xname << ")=" << fx << ", f'(" << xname << ")=" << fdx;
		ss << ", dx=" << fx / fdx;
		return ss.str();
	}
}

//------------------------------------------------------------------------------
// コンストラクタ
ib2_mss::Solver1::Solver1(double xtol, double ytol, size_t itmax) :
xtol_(xtol), ytol_(ytol), itmax_(itmax)
{
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::Solver1::~Solver1() = default;

//------------------------------------------------------------------------------
// 独立変数収束判定許容値の取得
double ib2_mss::Solver1::xtol() const
{
	return xtol_;
}

//------------------------------------------------------------------------------
// 従属変数収束判定許容値の取得
double ib2_mss::Solver1::ytol() const
{
	return ytol_;
}

//------------------------------------------------------------------------------
// 最大反復計算回数の取得
size_t ib2_mss::Solver1::itmax() const
{
	return itmax_;
}

//------------------------------------------------------------------------------
// Van Wijngaarden-Dekker-Brent法
double ib2_mss::Solver1::brent(const Function1& f, double x1, double x2) const
{
	static const double EPS(std::numeric_limits<double>::epsilon());
	double a = x1;
	double b = x2;
	double fa = f(a);
	double fb = f(b);
	if (fa * fb > 0.)
	{
		std::string what("Root must be bracketed");
		LOG_ERROR(what + debugBrent(0, x1, x2, fa, fb));
		throw std::domain_error(what);
	}
	
	double c  = b;
	double fc = fb;
	double d = b - a;
	double e = d;
	for (size_t it = 0; it < itmax_; ++it)
	{
		if (Log::isDebug())
			LOG_DEBUG(debugBrent(it, a, b, fa, fb));
		
		// a,b,cの名前を付け替え、区間幅を調整
		if (fb * fc > 0.)
		{
			c  = a;
			fc = fa;
			d  = b - a;
			e  = d;
		}
		if (std::abs(fc) < std::abs(fb))
		{
			a  = b;
			b  = c;
			c  = a;
			fa = fb;
			fb = fc;
			fc = fa;
		}
		
		// 収束の確認
		double tol1 = 2. * EPS * std::abs(b);
		tol1 += 0.5 * xtol_;
		double xm = 0.5 * (c - b);
		if (std::abs(xm) <= tol1 || std::abs(fb) < ytol_)
			return b;
		
		if (std::abs(e) >= tol1 && std::abs(fa) > std::abs(fb))
		{	// 逆2乗補間を試みる
			double s = fb / fa;
			double p, q;
			if (std::abs(1. - c / a) < EPS)
			{
				p = 2. * xm * s;
				q = 1. - s;
			}
			else
			{
				double r = fb / fc;
				q = fa / fc;
				p = s * (2. * xm * q * (q - r) - (b - a) * (r - 1.));
				q = (q - 1.) * (r - 1.) * (s - 1.);
			}
			
			// 区間内かどうかのチェック
			if (p > 0.)
				q = -q;
			p = std::abs(p);
			double min1 = 3. * xm * q - std::abs(tol1 * q);
			double min2 = std::abs(e * q);
			if (2. * p < (min1 < min2 ? min1 : min2))
			{	// 補間値を採用
				e = d;
				d = p / q;
			}
			else
			{	// 補間失敗、ニ分法を使う
				d = xm;
				e = d;
			}
		}
		else
		{	// 区間幅の減少が遅すぎるので二分法を使う
			d = xm;
			e = d;
		}
		
		// 前回の最良値をaに移す
		a  = b;
		fa = fb;
		
		// 新しい根の候補を計算
		b += std::abs(d) > tol1 ? d : xm > 0. ? tol1 : -tol1;
		fb = f(b);
	}
	LOG_ERROR(MAX_ITERATION + debugBrent(0, x1, x2, f(x1), f(x2)));
	throw std::domain_error(MAX_ITERATION);
}

//------------------------------------------------------------------------------
// Newton-Raphson法
double ib2_mss::Solver1::newton(const Function1& f, double x0) const
{
	double x(x0);
	for (size_t it = 0; it < itmax_; ++it)
	{
		double fx = f(x);
		double fdx = f.d(x);
		double dx = fx / fdx;
		
		if (Log::isDebug())
			LOG_DEBUG(debugNewton(it, x, fx, fdx));
		
		x -= dx;
		dx /= std::abs(x) > 1. ? x : 1.;
		if (std::abs(dx) < xtol_ || std::abs(fx) < ytol_)
			return x;
	}
	LOG_ERROR(MAX_ITERATION + debugNewton(0, x0, f(x0), f.d(x0), "x0"));
	throw std::domain_error(MAX_ITERATION);
}

// End Of File -----------------------------------------------------------------
