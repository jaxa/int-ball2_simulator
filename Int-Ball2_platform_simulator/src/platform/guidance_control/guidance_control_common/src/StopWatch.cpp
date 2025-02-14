
#include "guidance_control_common/StopWatch.h"

#include <iomanip>
#include <sstream>
#include <cmath>

//------------------------------------------------------------------------------
// コンストラクタ
ib2_mss::StopWatch::StopWatch(const std::string& name) :
name_(name), start_(std::chrono::steady_clock::now())
{
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::StopWatch::~StopWatch() = default;

//------------------------------------------------------------------------------
// 識別名称の取得
const std::string& ib2_mss::StopWatch::name() const
{
	return name_;
}

//------------------------------------------------------------------------------
// 計測開始からの経過時間の出力
double ib2_mss::StopWatch::elapsedsec() const
{
	std::chrono::duration<double> s(std::chrono::steady_clock::now() - start_);
	return s.count();
}

//------------------------------------------------------------------------------
// 計測開始からの経過時間コメントの出力
std::string ib2_mss::StopWatch::elapsed(size_t digits) const
{
	std::chrono::duration<double> dt(std::chrono::steady_clock::now() - start_);
	auto hour(std::chrono::duration_cast<std::chrono::hours  >(dt));
	auto min (std::chrono::duration_cast<std::chrono::minutes>(dt));
	
	auto h(hour.count());
	auto m(min .count());
	double s((dt - (hour + min)).count());
	
	static const int DIGIT(2);
	int digitsec(DIGIT+static_cast<int>(digits) + 1);
	
	std::ostringstream ss;
	ss.precision(digits);
	ss << name_ << " took "                                      << h << "h ";
	ss << std::setw(DIGIT)    << std::setfill('0')               << m << "m ";
	ss << std::setw(digitsec) << std::setfill('0') << std::fixed << s << "s.";
	return ss.str();
}

// End Of File -----------------------------------------------------------------
