
#include "guidance_control_common/MovingAverage.h"

#include <limits>
#include <cmath>

//------------------------------------------------------------------------------
// コンストラクタ.
ib2_mss::MovingAverage::MovingAverage(size_t nmax) :
nmax_(nmax > 0 ? nmax : std::numeric_limits<size_t>::max()),
x_(0), m_(0.), v_(0.)
{
}

//------------------------------------------------------------------------------
// デストラクタ.
ib2_mss::MovingAverage::~MovingAverage() = default;

//------------------------------------------------------------------------------
// データの追加.
void ib2_mss::MovingAverage::push_back(double x)
{
	if (!std::isfinite(x))
		return;

	if (x_.size() >= nmax_)
		pop_front();

	double n(static_cast<size_t>(x_.size()));
	double n1(n + 1.);
	double rnn1(n  / n1);
	double r1n1(1. / n1);
	double dx(x - m_);
	v_ = rnn1 * (v_ + r1n1 * dx * dx);
	m_ = rnn1 *  m_ + r1n1 * x;
	x_.push_back(x);
	if (v_ < 0.)
		v_ = 0.;
}

//------------------------------------------------------------------------------
// 先頭データの取り出し.
void ib2_mss::MovingAverage::pop_front()
{
	if (x_.empty())
		return;

	double n(static_cast<size_t>(x_.size()));
	double x(x_.front());
	x_.pop_front();

	if (x_.empty())
	{
		m_ = 0.;
		v_ = 0.;
	}
	else if (x_.size() == 1)
	{
		m_ = x_.front();
		v_ = 0.;
	}
	else
	{
		double n1(n - 1.);
		double rnn1(n  / n1);
		double r1n1(1. / n1);
		double dx(x - m_);
		v_ = rnn1 * (v_ - r1n1 * dx * dx);
		m_ = rnn1 *  m_ - r1n1 * x;
		if (v_ < 0.)
			v_ = 0.;
	}
}

//------------------------------------------------------------------------------
// データの最大個数のの取得.
size_t ib2_mss::MovingAverage::nmax() const
{
	return nmax_;
}

//------------------------------------------------------------------------------
// データ配列の参照.
const std::deque<double>& ib2_mss::MovingAverage::x() const
{
	return x_;
}

//------------------------------------------------------------------------------
// 平均の取得.
double ib2_mss::MovingAverage::m() const
{
	return m_;
}

//------------------------------------------------------------------------------
// 分散の取得.
double ib2_mss::MovingAverage::v(bool unbiased) const
{
	if (unbiased)
	{
		if (x_.size() < 2)
			return 0.;
		double n(static_cast<double>(x_.size()));
		return n / (n - 1.) * v_;
	}
	return v_;
}

//------------------------------------------------------------------------------
// 標準偏差の取得.
double ib2_mss::MovingAverage::s(bool unbiased) const
{
	return std::sqrt(v(unbiased));
}

// End Of File -----------------------------------------------------------------
