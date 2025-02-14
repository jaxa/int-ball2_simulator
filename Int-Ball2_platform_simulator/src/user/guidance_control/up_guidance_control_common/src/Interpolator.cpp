
#include "guidance_control_common/Interpolator.h"
#include "guidance_control_common/RangeChecker.h"
#include "guidance_control_common/Utility.h"
#include "guidance_control_common/Log.h"

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <limits>
#include <cmath>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 補間区間の探索
	 * @param [in] ob 範囲外の計算方法
	 * @param [in] xa 補間対象データ配列
	 * @param [in, out] x 独立変数
	 * @return 補間点を含む区間の上側イテレータ
	 */
	std::deque<double>::const_iterator
	section(const ib2_mss::Interpolator::OUT_RANGE& ob,
			const std::deque<double>& xa, double& x)
	{
		using namespace ib2_mss;
		auto x1(std::upper_bound(xa.begin(), xa.end(), x));
		if(ob == Interpolator::OUT_RANGE::ZERO)
		{
			if (x1 == xa.end() && x <= xa.back())
				--x1;
			return x1;
		}
		if (ob == Interpolator::OUT_RANGE::EXCEPT)
		{
			if (x1 == xa.begin() || x1 == xa.end())
			{
				std::string what("out of range");
				std::ostringstream ss;
				ss << what << ", x=" << x;
				ss << " must be between " << xa.front() << " and " << xa.back();
				LOG_ERROR(ss.str());
				throw std::domain_error(what);
			}
		}
		else
		{
			if (x1 == xa.end())
			{
				--x1;
				if (ob == Interpolator::OUT_RANGE::BOUND)
					x = xa.back();
			}
			else if (x1 == xa.begin())
			{
				++x1;
				if (ob == Interpolator::OUT_RANGE::BOUND)
					x = xa.front();
			}
		}
		return x1;
	}
}

//------------------------------------------------------------------------------
// 配列を設定しないコンストラクタ.
ib2_mss::Interpolator::Interpolator
(size_t size, double xmin, double xmax, double ymin, double ymax) :
x_(0), y_(0), maxSize_(size),
xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax)
{
	checkRange();
}

//------------------------------------------------------------------------------
// 配列を設定するコンストラクタ.
ib2_mss::Interpolator::Interpolator
(const std::deque<double>& x, const std::deque<double>& y,
 size_t size, double xmin, double xmax, double ymin, double ymax) :
x_(x), y_(y), maxSize_(size),
xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax)
{
	set();
}

//------------------------------------------------------------------------------
// 配列を設定するコンストラクタ.
ib2_mss::Interpolator::Interpolator
(const std::vector<double>& x, const std::vector<double>& y,
 size_t size, double xmin, double xmax, double ymin, double ymax) :
x_(Utility::to_deque(x)), y_(Utility::to_deque(y)), maxSize_(size),
xmin_(xmin), xmax_(xmax), ymin_(ymin), ymax_(ymax)
{
	set();
}

//------------------------------------------------------------------------------
// デストラクタ.
ib2_mss::Interpolator::~Interpolator() = default;

//------------------------------------------------------------------------------
// データ配列への補間点の追加
size_t ib2_mss::Interpolator::insert(double x, double y)
{
	x = Utility::cyclicRange(x, xmin_, xmax_);
	y = Utility::cyclicRange(y, ymin_, ymax_);
	
	// 挿入位置の探索
	auto range = std::equal_range(x_.begin(), x_.end(), x);
	if (std::distance(range.first, range.second) > 0)
	{
		size_t i0(std::distance(x_.begin(), range.first));
		size_t i1(std::distance(x_.begin(), range.second));
		std::string what("same dipendent variable exists.");
		std::ostringstream ss;
		ss << what << "cannot insert dipendent variable x=" << x << ". ";
		ss <<   "x[" << i0 << "]=" << *range.first;
		ss << ", y[" << i0 << "]=" << y_.at(i0);
		ss <<   "x[" << i1 << "]=" << *range.second;
		ss << ", y[" << i1 << "]=" << y_.at(i1);
		LOG_ERROR(ss.str());
		throw std::invalid_argument(what);
	}
	
	// 補間点の挿入
	size_t index = std::distance(x_.begin(), range.first);
	x_.insert(range.first, x);
	y_.insert(y_.begin() + index, y);
	
	// 挿入点より遠い側のデータを削除
	size_t nx = x_.size();
	if (maxSize_ > 0 && nx > maxSize_)
	{
		size_t is = indexStart();
		size_t di = index >= is ? index - is : nx + index - is;
		if (di < nx/2)	// 前半へ挿入
		{
			if (is == 0)
			{
				erase(nx-1);
				return index;
			}
			erase(is-1);
		}
		else			// 後半へ挿入
		{
			erase(is);
		}
		if (index >= is)
			--index;
	}
	return index;
}

//------------------------------------------------------------------------------
// 指定インデックスの補間点をデータ配列から削除.
void ib2_mss::Interpolator::erase(size_t index)
{
	RangeCheckerUI64 ARRAY_INDEX(RangeCheckerUI64::TYPE::LT, x_.size(), true);
	ARRAY_INDEX.valid(index, "index");
	x_.erase(x_.begin() + index);
	y_.erase(y_.begin() + index);
}

//------------------------------------------------------------------------------
// 補間データ配列をクリア.
void ib2_mss::Interpolator::clear()
{
	x_.clear();
	y_.clear();
}

//------------------------------------------------------------------------------
// メンバ変数の設定.
void ib2_mss::Interpolator::set()
{
	RangeCheckerUI64::equal(x_.size(), y_.size(), true,"x_size","y_size");
	if (maxSize_ > 0)
	{
		RangeCheckerUI64 X_SIZE(RangeCheckerUI64::TYPE::LE, maxSize_, true);
		X_SIZE.valid(x_.size(), "x_size");
	}
	checkRange();
	for (auto& xi : x_)
		xi = Utility::cyclicRange(xi, xmin_, xmax_);
	for (auto& yi : y_)
		yi = Utility::cyclicRange(yi, ymin_, ymax_);
	
	sort();
	if (!isSorted())
	{
		std::string what("same dipendent variables exist.");
		LOG_ERROR(what + " : " + string(16));
		throw std::invalid_argument(what);
	}
}

//------------------------------------------------------------------------------
// メンバ変数の並べ替え.
void ib2_mss::Interpolator::sort()
{
	size_t n = x_.size();
	std::deque<std::pair<double, double>> xy(n);
	for (size_t i = 0; i < n; ++i)
	{
		xy[i] = std::make_pair(x_[i], y_[i]);
	}
	std::sort(xy.begin(), xy.end());
	
	// 並べ替え結果の代入
	for (size_t i = 0; i < n; ++i)
	{
		x_[i] = xy[i].first;
		y_[i] = xy[i].second;
	}
}

//------------------------------------------------------------------------------
// 補間データの独立変数配列.
const std::deque<double>& ib2_mss::Interpolator::x() const
{
	return x_;
}

//------------------------------------------------------------------------------
// 補間データの従属変数配列.

const std::deque<double>& ib2_mss::Interpolator::y() const
{
	return y_;
}

//------------------------------------------------------------------------------
// 補間データの独立変数配列指定要素の取得.
double ib2_mss::Interpolator::x(size_t index) const
{
	return x_.at(index);
}

//------------------------------------------------------------------------------
// 補間データの従属変数配列指定要素の取得.
double ib2_mss::Interpolator::y(size_t index) const
{
	return y_.at(index);
}

//------------------------------------------------------------------------------
// 補間データ配列のサイズ.
size_t ib2_mss::Interpolator::size() const
{
	return x_.size();
}

//------------------------------------------------------------------------------
// 補間データ配列の最大サイズ.
size_t ib2_mss::Interpolator::maxSize() const
{
	return maxSize_;
}

//------------------------------------------------------------------------------
// 独立変数範囲最小値の取得.
double ib2_mss::Interpolator::xmin() const
{
	return xmin_;
}

//------------------------------------------------------------------------------
// 独立変数範囲最大値の取得.
double ib2_mss::Interpolator::xmax() const
{
	return xmax_;
}

//------------------------------------------------------------------------------
// 従属変数範囲最小値の取得.
double ib2_mss::Interpolator::ymin() const
{
	return ymin_;
}

//------------------------------------------------------------------------------
// 従属変数範囲最大値の取得.
double ib2_mss::Interpolator::ymax() const
{
	return ymax_;
}

//------------------------------------------------------------------------------
// 補間データの文字列取得
std::string ib2_mss::Interpolator::string(std::streamsize precision) const
{
	std::ostringstream ss;
	ss.precision(precision);
	ss << "(x,y) =";
	for (size_t i = 0; i < x_.size(); ++i)
		ss << " [" << i << "]:(" << x_[i] << "," << y_[i] << ")";
	return ss.str();
}

//------------------------------------------------------------------------------
// 同じ独立変数の確認
bool ib2_mss::Interpolator::hasX(double x) const
{
	x = Utility::cyclicRange(x, xmin_, xmax_);
	auto range(std::equal_range(x_.begin(), x_.end(), x));
	return range.first != range.second;
}

//------------------------------------------------------------------------------
// 補間データ開始点のインデックスを取得.
size_t ib2_mss::Interpolator::indexStart() const
{
	size_t is(0);
	double px(xmax_ - xmin_);
	if (px > 0.)
	{
		double dxmax(x_.front() + px - x_.back());
		for (size_t i = 1; i < x_.size(); ++i)
		{
			double dx(x_[i] - x_[i-1]);
			if (dx > dxmax)
			{
				dxmax = dx;
				is = i;
			}
		}
	}
	return is;
}

//------------------------------------------------------------------------------
// 補間のための修正データ配列の取得
std::pair<std::deque<double>, std::deque<double>>
ib2_mss::Interpolator::modifiedArray(double x) const
{
	std::deque<double> xa, ya;
	
	// 独立変数の修正
	size_t is(indexStart());
	if (is > 0)
	{
		auto itsx(x_.begin() + is);
		auto itsy(y_.begin() + is);
		std::copy(itsx, x_.end(), std::back_inserter(xa));
		std::copy(itsy, y_.end(), std::back_inserter(ya));
		std::copy(x_.begin(), itsx, std::back_inserter(xa));
		std::copy(y_.begin(), itsy, std::back_inserter(ya));
		double xrange(xmax_ - xmin_);
		size_t n(xa.size() - is);
		if (x < *itsx)
		{
			for (size_t i = 0; i < n; ++i)
				xa[i] -= xrange;
		}
		else
		{
			for (size_t i = n; i < xa.size(); ++i)
				xa[i] += xrange;
		}
	}
	else
	{
		xa = x_;
		ya = y_;
	}
	
	// 従属変数の修正
	double yrange = ymax_ - ymin_;
	if (yrange > 0.)
	{
		double yrangehalf = 0.5 * yrange;
		for (size_t i = 1; i < ya.size(); ++i)
		{
			double dy = ya[i] - ya[i-1];
			if (dy >  yrangehalf)
				ya[i] -= yrange;
			else if (dy < -yrangehalf)
				ya[i] += yrange;
		}
	}
	return std::make_pair(xa, ya);
}

//------------------------------------------------------------------------------
// 循環範囲のメンバ変数の確認.
void ib2_mss::Interpolator::checkRange() const
{
	RangeCheckerD XMIN(RangeCheckerD::TYPE::LE, xmax_, true);
	RangeCheckerD YMIN(RangeCheckerD::TYPE::LE, ymax_, true);
	XMIN.valid(xmin_, "xmin");
	YMIN.valid(ymin_, "ymin");
}

//------------------------------------------------------------------------------
// 循環範囲のメンバ変数の確認.
bool ib2_mss::Interpolator::isSorted() const
{
	for (size_t i = 1; i < x_.size(); ++i)
	{
		if (x_[i] <= x_[i-1])
			return false;
	}
	return true;
}

//------------------------------------------------------------------------------
// 線型補間.
double ib2_mss::Interpolator::linear
(double x, double *dydx, const OUT_RANGE& ob) const
{
	x = Utility::cyclicRange(x, xmin_, xmax_);
	auto a(modifiedArray(x));
	double y(linear(a.first, a.second, x, dydx, ob));
	return Utility::cyclicRange(y, ymin_, ymax_);
}

//------------------------------------------------------------------------------
// 線型補間.
std::vector<double> ib2_mss::Interpolator::linear
(const std::vector<double>& x, std::vector<double> *dydx,
 const OUT_RANGE& ob) const
{
	std::vector<double>  y(x.size());
	std::vector<double> dy(x.size());
	for (size_t i = 0; i < x.size(); ++i)
		y[i] = linear(x[i], &dy[i], ob);
	if (dydx != nullptr)
		*dydx = std::move(dy);
	return y;
}

//------------------------------------------------------------------------------
// 多項式補間(Lagrange補間).
double ib2_mss::Interpolator::polynomial
(double x, double *dy) const
{
	x = Utility::cyclicRange(x, xmin_, xmax_);
	auto a(modifiedArray(x));
	double y(polynomial(a.first, a.second, x, dy));
	return Utility::cyclicRange(y, ymin_, ymax_);
}

//------------------------------------------------------------------------------
// 有理関数補間.
double ib2_mss::Interpolator::rational
(double x, double *dy) const
{
	x = Utility::cyclicRange(x, xmin_, xmax_);
	auto a(modifiedArray(x));
	double y(rational(a.first, a.second, x, dy));
	return Utility::cyclicRange(y, ymin_, ymax_);
}

//------------------------------------------------------------------------------
// 線型補間
std::pair<double, double> ib2_mss::Interpolator::linear
(double x0, double x1, double y0, double y1, double x)
{
	double dx(x1 - x0);
	if (std::abs(dx) > 0.)
	{
		double dydx((y1 - y0) / dx);
		return std::make_pair(dydx * (x - x0) + y0, dydx);
	}
	else
	{
		std::string what("zero divide");
		std::vector<std::pair<std::string, double>> info
		{{"x0", x0}, {"x1", x1}, {"dx", dx}};
		LOG_ERROR(what + ", " + Log::stringDebug(info));
		throw std::domain_error(what);
	}
}

//------------------------------------------------------------------------------
// 線型補間
double ib2_mss::Interpolator::linear
(const std::deque<double>& xa, const std::deque<double>& ya,
 double x, double *dydx, const OUT_RANGE& ob)
{
	try
	{
		RangeCheckerUI64::equal(xa.size(), ya.size(), true,"xa_size","ya_size");
		if (xa.empty())
			throw std::domain_error("arrays are empty.");
		else if (xa.size() == 1)
			return ya.front();
		
		// 補間区間の探索
		auto x1(section(ob, xa, x));
		if (x1 == xa.begin() || x1 == xa.end())
		{
			if (dydx != nullptr)
				*dydx = 0.;
			return 0.;
		}
		
		size_t i(x1 - xa.begin());
		double y, a;
		std::tie(y, a) = linear(xa[i-1], xa[i], ya[i-1], ya[i], x);
		if (dydx)
			*dydx = a;
		return y;
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 線型補間
double ib2_mss::Interpolator::linear
(const std::vector<double>& xa, const std::vector<double>& ya,
 double x, double *dydx, const OUT_RANGE& ob)
{
	return linear(Utility::to_deque(xa), Utility::to_deque(ya), x, dydx, ob);
}

//------------------------------------------------------------------------------
// 多項式補間
double ib2_mss::Interpolator::polynomial
(const std::deque<double>& xa, const std::deque<double>& ya,
 double x, double *dy)
{
	try
	{
		RangeCheckerUI64::equal(xa.size(), ya.size(), true,"xa_size","ya_size");
		if (xa.empty())
			throw std::domain_error("arrays are empty.");
		else if (xa.size() == 1)
			return ya.front();
		
		// 最近接点の探索
		typename std::deque<double>::const_iterator
		it = std::lower_bound(xa.begin(), xa.end(), x);
		if (it == xa.end()) --it;
		typename std::deque<double>::const_iterator it0 = it;
		if (std::abs(*it0 - x) < std::abs(*it - x)) --it;
		long ns = static_cast<long>(std::distance(xa.begin(), it));
		
		// 補間計算
		std::deque<double> c(ya);
		std::deque<double> d(ya);
		double y = ya[ns--];
		long n = static_cast<long>(xa.size());
		for (long m = 1; m < n; ++m)
		{
			for (long i = 1; i <= n - m; ++i)
			{
				double ho = xa[i  -1] - x;
				double hp = xa[i+m-1] - x;
				double den = ho - hp;
				if (std::abs(den) < std::numeric_limits<double>::epsilon())
				{
					std::ostringstream ss;
					ss << "same dependent variable exists";
					ss << ". x[" << i  -1 << "]=" << xa[i  -1];
					ss << ", x[" << i+m-1 << "]=" << xa[i+m-1];
					LOG_ERROR(ss.str());
					throw std::domain_error("devided by zero");
				}
				den = (c[i] - d[i-1]) / den;
				c[i-1] = ho * den;
				d[i-1] = hp * den;
			}
			double e = (2*ns < n-m-1) ? c[ns+1] : d[ns--];
			y  += e;
			if (dy != nullptr)
				*dy = e;
		}
		return y;
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 多項式補間
double ib2_mss::Interpolator::polynomial
(const std::vector<double>& xa, const std::vector<double>& ya,
 double x, double *dy)
{
	return polynomial(Utility::to_deque(xa), Utility::to_deque(ya), x, dy);
}

//------------------------------------------------------------------------------
// 有理関数補間
double ib2_mss::Interpolator::rational
(const std::deque<double>& xa, const std::deque<double>& ya,
 double x, double *dy)
{
	try
	{
		RangeCheckerUI64::equal(xa.size(), ya.size(), true,"xa_size","ya_size");
		if (xa.empty())
			throw std::domain_error("arrays are empty.");
		else if (xa.size() == 1)
			return ya.front();
		
		// 最近接点の探索
		typename std::deque<double>::const_iterator
		it = std::lower_bound(xa.begin(), xa.end(), x);
		if (it == xa.end()) --it;
		typename std::deque<double>::const_iterator it0 = it;
		if (std::abs(*it0 - x) < std::abs(*it - x)) --it;
		if (std::abs(*it - x) < std::numeric_limits<double>::epsilon())
		{
			if (dy != nullptr)
				*dy = 0.;
			return ya[std::distance(xa.begin(), it)];
		}
		long ns = static_cast<long>(std::distance(xa.begin(), it));
		
		// 係数の初期化
		std::deque<double> c(ya);
		std::deque<double> d(ya);
		static const double TINY(1.e-25);
		for (auto& z: d) z += TINY;
		
		// 補間計算
		double y = ya[ns--];
		long n = static_cast<long>(xa.size());
		for (long m = 1; m < n; ++m)
		{
			for (long i = 1; i <= n - m; ++i)
			{
				double t = (xa[i-1] - x) * d[i-1] / (xa[i+m-1] - x);
				double dd = t - c[i];
				if (std::abs(dd) < std::numeric_limits<double>::epsilon())
				{
					std::ostringstream ss;
					ss << "extreme at x = " << x;
					ss << ". x[" << i  -1 << "]=" << xa[i  -1];
					ss << ", x[" << i+m-1 << "]=" << xa[i+m-1];
					ss << ", c[" << i     << "]=" <<  c[i    ];
					ss << ", d[" << i  -1 << "]=" <<  d[i  -1];
					ss << ", t=" << t << ", dd=" << dd;
					LOG_ERROR(ss.str());
					throw std::domain_error("extreme of rational function");
				}
				dd = (c[i] - d[i-1]) / dd;
				d[i-1] = c[i] * dd;
				c[i-1] = t       * dd;
			}
			double e = (2*ns < n-m-1) ? c[ns+1] : d[ns--];
			y  += e;
			if (dy != nullptr)
				*dy = e;
		}
		return y;
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 有理関数補間
double ib2_mss::Interpolator::rational
(const std::vector<double>& xa, const std::vector<double>& ya,
 double x, double *dy)
{
	return rational(Utility::to_deque(xa), Utility::to_deque(ya), x, dy);
}

//------------------------------------------------------------------------------
// 範囲外種別の取得
ib2_mss::Interpolator::OUT_RANGE
ib2_mss::Interpolator::outrange(const std::string& type)
{
	static const std::vector<std::string> OUT_RANGE_STRING
	{"EXTRA", "BOUND", "ZERO", "EXCEPT"};
	
	for (unsigned int i = 0; i < OUT_RANGE_STRING.size(); ++i)
	{
		if (type == OUT_RANGE_STRING.at(i))
			return static_cast<OUT_RANGE>(i);
	}
	std::string what("invalid outrange type");
	std::ostringstream ss;
	ss << what << ", inputed:" << type << ", required:";
	for (auto& str: OUT_RANGE_STRING)
		ss << str << ",";
	LOG_ERROR(ss.str());
	throw std::invalid_argument(what);
}

// End Of File -----------------------------------------------------------------
