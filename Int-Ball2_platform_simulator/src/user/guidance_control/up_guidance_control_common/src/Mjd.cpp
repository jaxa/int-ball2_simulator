
#include "guidance_control_common/Mjd.h"
#include "guidance_control_common/TaiUtc.h"
#include "guidance_control_common/Utility.h"
#include "guidance_control_common/RangeChecker.h"
#include "guidance_control_common/IntOverflow.h"
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Log.h"

#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <cstdint>
#include <limits>
#include <cstring>
#include <cstdlib>
#include <cassert>
#include <cmath>

//------------------------------------------------------------------------------
// 定数
namespace
{
	/** 十進数の基数 */
	const int64_t BASE(10);

	/** 少数桁数 */
	const uint16_t DIGITS(ib2_mss::Mjd::PRECISION);
	
	/** 小数秒の分母 */
	const int64_t DENOMINATOR(ib2_mss::Utility::powint(BASE, DIGITS));
	
	/** ミリ秒の分母 */
	const int64_t DENOMINATOR_MSEC(ib2_mss::Utility::powint(BASE, 3));

	/** マイクロ秒の分母 */
	const int64_t DENOMINATOR_USEC(ib2_mss::Utility::powint(BASE, 6));

	/** ナノ秒の分母 */
	const int64_t DENOMINATOR_NSEC(ib2_mss::Utility::powint(BASE, 9));

	/** 日 */
	const int64_t DECIMAL_PER_DAY(DENOMINATOR * ib2_mss::Mjd::SECONDS_PER_DAY);

	/** ミリ秒 */
	const int64_t DECIMAL_PER_MSEC(DENOMINATOR / DENOMINATOR_MSEC);
	
	/** マイクロ秒 */
	const int64_t DECIMAL_PER_USEC(DENOMINATOR / DENOMINATOR_USEC);
	
	/** ナノ秒 */
	const int64_t DECIMAL_PER_NSEC(DENOMINATOR / DENOMINATOR_NSEC);

	/** ２桁年の境界 */
	const int64_t BOUND_2DIGITS_YEAR(69);

	/** TimeZone環境変数識別文字列 */
	static const char *TZ("TZ");

	/** 時刻系文字列(ローカルを除く) */
	const std::vector<std::string> SCALE_STRING
	{"DUR", "TDB", "TDT", "TAI", "GPS", "UT1", "UTC"};

	/** ローカル時刻系文字列 */
	const std::vector<std::string> LOCAL_STRING
	{
		"LOCAL", "JST-9",
		"EET-2EETDST", "MET-1METDST", "MEZ-1MESZ", "GMT0BST",
		"AST4ADT", "EST5EDT", "CST6CDT", "MST7MDT", "PST8PDT",
#ifndef __WIN32__
		"NAST9NADT", "Asia/Tokyo",
#endif
	};

	/** int64範囲 */
	const ib2_mss::RangeCheckerD INT64
	(ib2_mss::RangeCheckerD::TYPE::GE_LE,
	 static_cast<double>(std::numeric_limits<int64_t>::min()),
	 static_cast<double>(std::numeric_limits<int64_t>::max()), true);
	
	/** 小数秒分母
	 * @param [in] precision 小数秒桁数
	 * @return 小数秒分母
	 */
	int64_t denominator(uint16_t precision)
	{
		return ib2_mss::Utility::powint(BASE, std::min(precision, DIGITS));	
	}

	/** 小数秒単位
	 * @param [in] precision 小数秒桁数
	 * @return 小数秒単位
	 * @return 小数秒分母
	 */
	std::pair<int64_t, int64_t> lsbDecimal(uint16_t precision)
	{
		int64_t den(denominator(precision));
		return std::make_pair(DENOMINATOR / den, den);
	}

	/** 文字列から時刻要素を取り出す
	 * @param [in] time 時刻文字列
	 * @param [in] format 時刻フォーマットの文字
	 * @param [in] successive 時刻フォーマットの連続フラグ(trueのとき区切り文字がない)
	 * @param [in] ndelimit 区切り文字までの文字数
	 * @param [in] start 時刻文字列の読み込み開始場所
	 * @param [in,out] year 年
	 * @param [in,out] month 月
	 * @param [in,out] mday 日(月初からの)
	 * @param [in,out] yday 日(年初からの,1月1日->1)
	 * @param [in,out] hour 時
	 * @param [in,out] minute 分
	 * @param [in,out] second 秒
	 * @param [in,out] psec ピコ秒
	 * @return 時刻文字列読み込み文字数
	 */
	std::string::size_type ptime
	(const std::string& time, char format, bool successive,
	 std::string::size_type ndelimit, std::string::size_type start,
	 int64_t& year, int64_t& month, int64_t& mday, int64_t& yday,
	 int64_t& hour, int64_t& minute, int64_t& second, int64_t& psec)
	{
		using namespace ib2_mss;
		std::string::size_type n
		(successive ? (format == 'Y' ? 4 : format == 'j' ? 3 : 2) : ndelimit);
		std::string str(time.substr(start, n));
		size_t ei(str.length());
		int64_t value(std::stoll(str, &ei));
		switch (format)
		{
			case 'Y':	year = value;	break;
			case 'y':
				year = value + (value >= BOUND_2DIGITS_YEAR ? 1900 : 2000);
				break;
			case 'm':	month = value;	break;
			case 'd':	mday = value;	break;
			case 'j':	yday = value;	break;
			case 'H':	hour = value;	break;
			case 'M':	minute = value;	break;
			case 'S':
			{
				second = value;
				if (ei < str.length())
				{
					str = str.substr(ei + 1);
					ei = str.length();
					psec = std::stoll(str, &ei);
					uint16_t ex(static_cast<uint16_t>(ei));
					if (ex > DIGITS)
						psec /= Utility::powint(BASE, ex) / DENOMINATOR;
					else
						psec *= DENOMINATOR / Utility::powint(BASE, ex);
				}
				break;
			}
			default:
			{
				std::string what("invalid charactor for ptime");
				LOG_ERROR(what + ". input char : " + format);
				throw std::domain_error(what);
			}
		}
		if (ei < str.length())
		{
			std::string what("invalid number");
			LOG_ERROR(what + ", target:" + str + ", error:" + str.substr(ei));
			throw std::domain_error(what);
		}
		return n;
	}
	
#ifdef __WIN32__
	/** 時刻文字列の作成
	 * @param [in] t 時刻構造体
	 * @param [in] format 時刻フォーマットの文字
	 * @return 時刻文字列
	 */
	std::string ftime1(const struct tm& t, char format)
	{
		std::stringstream ss;
		ss << std::setw(2) << std::setfill('0');
		switch (format)
		{
			case 'Y': return std::to_string(1900 + t.tm_year);
			case 'y':
			{
				ss << t.tm_year % 100;
				return ss.str();
			}
			case 'm':
			{
				ss << t.tm_mon + 1;
				return ss.str();
			}
			case 'd':
			{
				ss << t.tm_mday;
				return ss.str();
			}
			case 'j':
			{
				std::stringstream ssj;
				ssj << std::setw(3) << std::setfill('0') << t.tm_yday + 1;
				return ssj.str();
			}
			case 'H':
			{
				ss << t.tm_hour;
				return ss.str();
			}
			case 'M':
			{
				ss << t.tm_min;
				return ss.str();
			}
			case 'S':
			{
				ss << t.tm_sec;
				return ss.str();
			}
			default:
				break;
		}
		std::string what("invalid charactor for ftime");
		LOG_ERROR(what + ". input char : " + format);
		throw std::domain_error(what);
	}
	
	/** 時刻文字列の作成
	 * @param [in] t 時刻構造体
	 * @param [in] format 時刻フォーマットの文字
	 * @return 時刻文字列
	 */
	std::string ftime(const struct tm& t, const std::string& format)
	{
		std::string result("");
		std::string::size_type is(0);
		while (is != std::string::npos)
		{
			std::string::size_type ie(format.find_first_of('%', is));
			if (ie == std::string::npos)
			{
				result += format.substr(is);
				is = ie;
			}
			else
			{
				result += format.substr(is, ie - is);
				result += ftime1(t, format.at(ie + 1));
				is = ie + 2;
			}
		}
		return result;
	}

	/** 文字列の置き換え
	 * @param [in,out] t 操作対象
	 * @param [in] x 置き換え前の文字列
	 * @param [in] o 置き換え後の文字列
	 */
	void replace(std::string& t, const std::string& x, const std::string& o)
	{
		auto it(t.find(x));
		if (it != std::string::npos)
			t = t.substr(0, it) + o + t.substr(it+2);
	}
#endif
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2_mss::Mjd::Mjd() :
scale_(DUR()), day_(0), psec_(0)
{
}

//------------------------------------------------------------------------------
// time_t時刻によるコンストラクタ
ib2_mss::Mjd::Mjd(time_t t) :
scale_(UTC())
{
	std::tie(day_, psec_) = daypsec(t);
}

//------------------------------------------------------------------------------
// time_point時刻によるコンストラクタ.
ib2_mss::Mjd::Mjd
(const std::chrono::time_point
 <std::chrono::system_clock, std::chrono::nanoseconds>& tp) :
scale_(UTC())
{
	using namespace std::chrono;
	auto tpsec(time_point_cast<seconds>(tp));
	std::tie(day_, psec_) = daypsec(system_clock::to_time_t(tpsec));
	addnsec(duration_cast<nanoseconds>(tp - tpsec).count());
}

//------------------------------------------------------------------------------
// duration時刻によるコンストラクタ.
ib2_mss::Mjd::Mjd(const std::chrono::nanoseconds& d) :
scale_(DUR()), day_(0), psec_(0)
{
	using namespace std::chrono;
	auto s(duration_cast<seconds>(d));
	addSeconds(s.count());
	addnsec((d-s).count());
}

//------------------------------------------------------------------------------
// 時刻構造体によるコンストラクタ.
ib2_mss::Mjd::Mjd(const std::string& scale, const std::tm& t) :
scale_(scale)
{
	try
	{
		validScale(scale, true);
		setTm(t);
		carry();
	}
	catch (const std::exception &e)
	{
		std::ostringstream ss;
		ss << "scale="      << scale;
		ss << ", tm_year="  << t.tm_year;
		ss << ", tm_mon="   << t.tm_mon;
		ss << ", tm_mday="  << t.tm_mday;
		ss << ", tm_hour="  << t.tm_hour;
		ss << ", tm_min="   << t.tm_min;
		ss << ", tm_sec="   << t.tm_sec;
		ss << ", tm_wday="  << t.tm_wday;
		ss << ", tm_yday="  << t.tm_yday;
		ss << ", tm_isdst=" << t.tm_isdst;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 実数によるコンストラクタ
ib2_mss::Mjd::Mjd(const std::string& scale, double day) :
scale_(scale), day_(0), psec_(0)
{
	double eps(std::numeric_limits<double>::epsilon());
	try
	{
		if (!isGrobal())
			throw std::domain_error("inputed scale is not supported");
		if (!std::isfinite(day))
			throw std::domain_error("inputed day is not finite number");
		eps *= std::abs(day);
		if (eps > 1.)
			day_ = Utility::int64(day);
		else
		{
			double dayint(floor(day));
			day_ = static_cast<int64_t>(dayint);
			eps *= DAY;
			double sec((day - dayint) * secday());
			if (eps > 1.)
				psec_ = Utility::int64(sec) * DENOMINATOR;
			else
			{
				uint16_t digits(static_cast<uint16_t>(-floor(log10(eps))));
				psec_ = psec(sec, true,  digits);
			}
			carry();
		}
	}
	catch (const std::exception &e)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << "scale=" << scale << ", day=" << day << ", eps=" << eps;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 値指定によるコンストラクタ
ib2_mss::Mjd::Mjd(const std::string& scale, int64_t day, int64_t psec) :
scale_(scale), day_(day), psec_(psec)
{
	try
	{
		validScale(scale, true);
		carry();
	}
	catch (const std::exception &e)
	{
		std::ostringstream ss;
		ss << "scale=" << scale << ", day=" << day << ", psec=" << psec;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 値指定によるコンストラクタ(GPS週、週秒)
ib2_mss::Mjd::Mjd(int64_t nroll, int64_t wn, int64_t tow, uint16_t precision) :
scale_(GPS()), day_(0), psec_(0)
{
	int64_t lsb, denom;
	std::tie(lsb, denom) = lsbDecimal(precision);
	auto d(lldiv(tow, SECONDS_PER_DAY * denom));
	psec_ = d.rem * lsb;
	day_ = d.quot + (nroll * MAX_GPS_WN + wn) * DAYS_PER_WEEK + MJD_GPS_EPOCH;
}

//------------------------------------------------------------------------------
// 年月日時分秒によるコンストラクタ
ib2_mss::Mjd::Mjd
(const std::string& scale, int64_t year, int64_t month, int64_t mday,
 int64_t hour, int64_t minute, int64_t second, int64_t decimal,
 uint16_t precision) :
scale_(scale)
{
	try
	{
		validScale(scale, true);
		if (year < 0)
			year = 0;
		if (year < BOUND_2DIGITS_YEAR)
			year += 2000;
		else if (year < 100)
			year += 1900;
		
		if (month <= 0)
			month = 1;
		
		if (mday <= 0)
			mday = 1;
		
		setYMDHMS(year, month, mday, hour, minute, second, decimal, precision);
		carry();
	}
	catch (const std::exception& e)
	{
		static const char DDT(' ');
		static const char DD('/');
		static const char DT(':');
		std::ostringstream ss;
		ss << "scale=" << scale;
		ss << ", calender=" << year << DD << month << DD << mday << DDT;
		ss << hour << DT << minute << DT << second << '.';
		ss << std::setw(precision) << std::setfill('0') << decimal;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 文字列によるコンストラクタ
ib2_mss::Mjd::Mjd
(const std::string& scale, const std::string& calendar,
 const std::string& format) :
scale_(scale)
{
	auto it(format.find("%T"));
	std::string form
	(it == std::string::npos ? format :
	 format.substr(0, it) + std::string("%H:%M:%S") + format.substr(it+2));
	
	std::string::size_type ic(0);
	std::string::size_type is(0);
	try
	{
		validScale(scale, true);
		int64_t year  (0);
		int64_t month (1);
		int64_t mday  (1);
		int64_t yday  (0);
		int64_t hour  (0);
		int64_t minute(0);
		int64_t second(0);
		int64_t decimal(0);
		std::string::size_type iend(form.size() - 1);
		while (is != std::string::npos)
		{
			auto ie(form.find_first_of('%', is));
			if (ie != std::string::npos && ie < iend)
			{
				ic += ie - is;
				++ie;
				bool successive(ie < iend && form.at(ie + 1) == '%');
				auto in(ie == iend ? std::string::npos :
						calendar.find_first_of(form.at(ie + 1), ic));
				if (in != std::string::npos)
					in -= ic;
				ic += ptime(calendar, form.at(ie), successive, in, ic,
							year, month, mday, yday,
							hour, minute, second, decimal);
				++ie;
				if (ie >= iend)
					ie = std::string::npos;
			}
			is = ie;
		}
		if (year == 0)
			throw std::domain_error("year is not inputed.");
		if (yday > 0 && month == 1 && mday == 1)
			std::tie(year,month,mday) = ymd(mjd(year, 1, 1) + yday - 1);
		setYMDHMS(year, month, mday, hour, minute, second, decimal);
		carry();
	}
	catch (const std::exception& e)
	{
		std::ostringstream ss;
		ss << "scale=" << scale << ", calendar=" << calendar;
		ss << ", format=" << format << ", ic=" << ic;
		ss << ", remain=" << calendar.substr(ic);
		ss << ", form="   << form << ", is=" << is;
		ss << ", remain=" << form.substr(is);
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// デストラクタ
ib2_mss::Mjd::~Mjd() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2_mss::Mjd::Mjd(const Mjd&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator=(const Mjd&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2_mss::Mjd::Mjd(Mjd&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator=(Mjd&&) = default;

//------------------------------------------------------------------------------
// 加算代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator+=(const Mjd& r)
{
	try
	{
		if (r.scale_ != DUR())
		{
			std::string what("invalid time scale");
			std::string comment(", right scale must be "+ DUR());
			LOG_ERROR(what + comment);
			throw std::domain_error(what);
		}
		int64_t oldday(day_);
		ADDABLE(day_, r.day_, true);
		ADDABLE(psec_, r.psec_, true);
		day_  += r.day_;
		psec_ += r.psec_;
		if (withLeap())
			psec_ -= psec(TaiUtc::leapsec(oldday, day_));
		carry();
		return *this;
	}
	catch (const std::exception& e)
	{
		std::ostringstream ss;
		ss << "left:" << *this << ", right:" << r;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 加算代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator+=(double r)
{
	try
	{
		int64_t dayll, psec;
		std::tie(dayll, psec) = daypsec(r);
		operator+=(Mjd(DUR(), dayll, psec));
		return *this;
	}
	catch (const std::exception& e)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << "left:" << *this << ", right:" << r << "[sec]";
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 減算代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator-=(const Mjd& r)
{
	try
	{
		if (r.scale_ != DUR())
		{
			checkSameScale(r);
			if (withLeap())
			{
				int64_t leap(psec(TaiUtc::leapsec(r.day_, day_)));
				ADDABLE(psec_, leap, true);
				psec_ += leap;
			}
			scale_ = DUR();
			SUBTRACTABLE(psec_, r.psec_, true);
			SUBTRACTABLE( day_, r. day_, true);
			psec_ -= r.psec_;
			day_  -= r. day_;
			carry();
			return *this;
		}
		return operator+=(Mjd(DUR(), -r.day_, -r.psec_));
	}
	catch (const std::exception& e)
	{
		std::ostringstream ss;
		ss << "left:" << *this << ", right:" << r;
		LOG_ERROR(Log::caughtException(e.what(), ss.str()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 減算代入演算子
ib2_mss::Mjd& ib2_mss::Mjd::operator-=(double r)
{
	operator+=(-r);
	return *this;
}

//------------------------------------------------------------------------------
// 日数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addDays(int64_t d)
{
	Mjd r(DUR(), d, 0);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// 時間数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addHours(int64_t h)
{
	auto d(lldiv(h, HOURS_PER_DAY));
	auto psec(d.rem * SECONDS_PER_HOUR * DENOMINATOR);
	Mjd r(DUR(), d.quot, psec);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// 分数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addMinutes(int64_t m)
{
	auto d(lldiv(m, MINUTES_PER_DAY));
	auto psec(d.rem * SECONDS_PER_MINUTE * DENOMINATOR);
	Mjd r(DUR(), d.quot, psec);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// 秒数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addSeconds(int64_t s)
{
	auto d(lldiv(s, SECONDS_PER_DAY));
	Mjd r(DUR(), d.quot, d.rem * DENOMINATOR);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// ミリ秒数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addmsec(int64_t ms)
{
	auto d(lldiv(ms, SECONDS_PER_DAY * DENOMINATOR_MSEC));
	Mjd r(DUR(), d.quot, d.rem * DECIMAL_PER_MSEC);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// マイクロ秒数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addusec(int64_t us)
{
	auto d(lldiv(us, SECONDS_PER_DAY * DENOMINATOR_NSEC));
	Mjd r(DUR(), d.quot, d.rem * DECIMAL_PER_USEC);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// ナノ秒数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addnsec(int64_t ns)
{
	auto d(lldiv(ns, SECONDS_PER_DAY * DENOMINATOR_NSEC));
	Mjd r(DUR(), d.quot, d.rem * DECIMAL_PER_NSEC);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// ピコ秒数の加算
ib2_mss::Mjd& ib2_mss::Mjd::addpsec(int64_t ps)
{
	auto d(lldiv(ps, SECONDS_PER_DAY * DENOMINATOR));
	Mjd r(DUR(), d.quot, d.rem);
	operator+=(r);
	return *this;
}

//------------------------------------------------------------------------------
// 時刻系の設定
ib2_mss::Mjd& ib2_mss::Mjd::setScale(const std::string& scale)
{
	validScale(scale, true);
	scale_ = scale;
	return *this;
}

//------------------------------------------------------------------------------
// カレンダー日による設定
ib2_mss::Mjd& ib2_mss::Mjd::setYMDHMS
(int64_t year, int64_t month, int64_t mday,
 int64_t hour, int64_t minute, int64_t second,
 int64_t decimal, uint16_t precision)
{
	int64_t lsb;
	std::tie(lsb, std::ignore) = lsbDecimal(precision);
	MULTIPLIABLE(decimal, lsb, true);
	decimal *= lsb;

	if (isGrobal())
	{
		day_  = mjd(year, month, mday);
		psec_ = psec(hour, minute, second, decimal);
	}
	else
	{
		std::tm t;
		t.tm_year = static_cast<int>(year - 1900);
		t.tm_mon  = static_cast<int>(month - 1);
		t.tm_mday = static_cast<int>(mday);
		t.tm_hour = static_cast<int>(hour);
		t.tm_min  = static_cast<int>(minute);
		t.tm_sec  = static_cast<int>(second);
		t.tm_wday = 0;
		t.tm_yday = 0;
		t.tm_isdst = -1;
		setTm(t);
		addpsec(decimal);
	}
	carry();
	return *this;
}

//------------------------------------------------------------------------------
// 時刻構造体による設定
ib2_mss::Mjd& ib2_mss::Mjd::setTm(const std::tm& t)
{
	if (scale_ == DUR())
		throw std::domain_error("scale DUR is not supported for struct tm");
	else if (isGrobal())
	{
		day_  = mjd(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
		psec_ = psec(t.tm_hour, t.tm_min, t.tm_sec, 0);
	}
	else
	{
		auto tm(t);
		tm.tm_sec = 0;
		std::string tzOld(getenv(TZ) == nullptr ? "" : getenv(TZ));
		bool setTZ(scale_ != LOCAL_STRING.front());
#ifdef __WIN32__
		if (setTZ)
		{
			_putenv_s(TZ, scale_.c_str());
			_tzset();
		}
		std::tie(day_, psec_) = daypsec(mktime(&tm));
		if (tm.tm_isdst > 0)
			addHours(-1);
		if (setTZ)
		{
			if (tzOld.empty())
				_putenv(TZ);
			else
				_putenv_s(TZ, tzOld.c_str());
			_tzset();
		}
#else
		if (setTZ)
		{
			setenv(TZ, scale_.c_str(), 1);
			tzset();
		}
		std::tie(day_, psec_) = daypsec(mktime(&tm));
		if (setTZ)
		{
			if (tzOld.empty())
				unsetenv(TZ);
			else
				setenv(TZ, tzOld.c_str(), 1);
			tzset();
		}
#endif
		addSeconds(t.tm_sec);
	}
	return *this;
}

//------------------------------------------------------------------------------
// 繰り上げ
void ib2_mss::Mjd::carry()
{
	try
	{
		bool leap(withLeap());
		if (psec_ < 0 || DECIMAL_PER_DAY <= psec_)
		{
			int64_t day(psec_ / DECIMAL_PER_DAY);
			if (psec_ < 0)
				--day;
			ADDABLE(day_, day, true);
			int64_t nday(day_ + day);
			psec_ -= day * DECIMAL_PER_DAY;
			if (leap)
				psec_ -= psec(TaiUtc::leapsec(day_, nday));
			day_ = nday;
		}
		if (leap)
		{
			int64_t psecday(psec(secday()));
			if (psec_ < 0)
			{
				--day_;
				psec_ += psec(secday());
			}
			else if(psec_ >= psecday)
			{
				++day_;
				psec_ -= psecday;
			}
		}
	}
	catch (const std::exception &e)
	{
		LOG_ERROR(Log::caughtException(e.what(), "this:" + debug()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 時刻系の参照
const std::string& ib2_mss::Mjd::scale() const
{
	return scale_;
}

//------------------------------------------------------------------------------
// 修正ユリウス日の取得
int64_t ib2_mss::Mjd::day() const
{
	return day_;
}

//------------------------------------------------------------------------------
// 0時からの経過時間の取得
int64_t ib2_mss::Mjd::psec() const
{
	return psec_;
}

//------------------------------------------------------------------------------
// 0時からの経過秒の取得
double ib2_mss::Mjd::sec() const
{
	return static_cast<double>(psec_) / static_cast<double>(DENOMINATOR);
}

//------------------------------------------------------------------------------
// 切り捨て日数の取得
int64_t ib2_mss::Mjd::floorDay() const
{
	return day_;
}

//------------------------------------------------------------------------------
// 切り捨て日数の取得
int64_t ib2_mss::Mjd::ceilDay() const
{
	return day_ + 1;
}

//------------------------------------------------------------------------------
// トータル秒数の取得
double ib2_mss::Mjd::totalSec() const
{
	if (scale_ == DUR())
		return sec() + static_cast<double>(day_) * DAY;

	throw std::domain_error("invalid scale for totalSec");
}

//------------------------------------------------------------------------------
// トータル日数の取得
double ib2_mss::Mjd::totalDay() const
{
	if (scale_ == DUR())
		return sec() / DAY + static_cast<double>(day_);

	throw std::domain_error("invalid scale for totalDay");
}

//------------------------------------------------------------------------------
// 経過日数
double ib2_mss::Mjd::elapsedDay(double epoch) const
{
	Mjd e(scale_, epoch);
	Mjd d(*this - e);
	return d.totalDay();
}

//------------------------------------------------------------------------------
// 経過ユリウス世紀の取得
double ib2_mss::Mjd::elapsedJC(double epoch) const
{
	return elapsedDay(epoch) / DAYS_PER_JULIAN_CENTURY;
}

//------------------------------------------------------------------------------
// 修正ユリウス日の取得
std::pair<double, double> ib2_mss::Mjd::mjd2d() const
{
	double dayi(static_cast<double>(day_));
	double dayf(sec() / secday());
	return std::make_pair(dayi, dayf);
}

//------------------------------------------------------------------------------
// time_t時刻の取得
std::pair<time_t, int64_t> ib2_mss::Mjd::tt(uint16_t precision) const
{
	if (scale_ == DUR())
		throw std::domain_error("DUR is invalid scale for funrtion tt");
	
	time_t t;
	int64_t d, lsb;
	std::tie(t, d) = tt(day_, psec_);
	std::tie(lsb, std::ignore) = lsbDecimal(precision);
	return std::make_pair(t, d / lsb);
}

//------------------------------------------------------------------------------
// 時刻構造体の取得
std::pair<std::tm, int64_t> ib2_mss::Mjd::tm(uint16_t precision) const
{
	if (scale_ == DUR())
		throw std::domain_error("DUR is invalid scale for function tm");
	
	auto psec(psec_ < DECIMAL_PER_DAY ? psec_ : DECIMAL_PER_DAY - DENOMINATOR);
	auto rem (psec_ - psec);
	time_t t;
	int64_t d;
	std::tie(t, d) = tt(day_, psec);
	std::tm tm;
	if (isGrobal())
	{
#ifdef __WIN32__
		gmtime_s(&tm, &t);
#else	//__WIN32__
		gmtime_r(&t, &tm);
#endif	//__WIN32__
	}
	else
	{
		std::string tzOld(getenv(TZ) == nullptr ? "" : getenv(TZ));
		bool setTZ(scale_ != LOCAL_STRING.front());
#ifdef __WIN32__
		if (setTZ)
		{
			_putenv_s(TZ, scale_.c_str());
			_tzset();
		}
		localtime_s(&tm, &t);
		if (tm.tm_isdst > 0)	// windowsはtm_hourで夏時間を考慮しない
		{
			t += SECONDS_PER_HOUR;
			localtime_s(&tm, &t);
		}
		if (setTZ)
		{
			if (tzOld.empty())
				_putenv(TZ);
			else
				_putenv_s(TZ, tzOld.c_str());
			_tzset();
		}
#else
		if (setTZ)
		{
			setenv(TZ, scale_.c_str(), 1);
			tzset();
		}
		localtime_r(&t, &tm);
		if (setTZ)
		{
			if (tzOld.empty())
				unsetenv(TZ);
			else
				setenv(TZ, tzOld.c_str(), 1);
			tzset();
		}
#endif
	}
	int64_t lsb, denom;
	std::tie(lsb, denom) = lsbDecimal(precision);
	auto l(lldiv((d + rem) / lsb, denom));
	tm.tm_sec += static_cast<int>(l.quot);
	return std::make_pair(std::move(tm), l.rem);
}

//------------------------------------------------------------------------------
// time_point時刻の取得
std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
ib2_mss::Mjd::tp() const
{
	using namespace std::chrono;
	static const RangeCheckerI64 TT_RANGE
	(RangeCheckerI64::TYPE::GT_LT,
	 duration_cast<seconds>
	 (time_point<system_clock, nanoseconds>::min().time_since_epoch()).count(),
	 duration_cast<seconds>
	 (time_point<system_clock, nanoseconds>::max().time_since_epoch()).count(),
	 true);
	
	if (scale_ == DUR())
		throw std::domain_error("DUR is invalid scale for function tp");
	
	time_t t;
	int64_t d;
	std::tie(t, d) = tt(9);
	TT_RANGE.valid(t, "time_t");
	auto l(lldiv(d, DENOMINATOR_NSEC));
	time_point<system_clock, nanoseconds> tp(system_clock::from_time_t(t));
	tp += nanoseconds(l.rem);
	return tp;
}

//------------------------------------------------------------------------------
// duration時刻の取得
std::chrono::nanoseconds ib2_mss::Mjd::duration() const
{
	static const int64_t NSEC_PER_DAY(SECONDS_PER_DAY * DENOMINATOR_NSEC);
	if (scale_ != DUR())
		throw std::domain_error("scale must be DUR for function duration");
	MULTIPLIABLE(day_, NSEC_PER_DAY, true);
	int64_t nsday(day_ * NSEC_PER_DAY);
	int64_t ns(psec_ / DECIMAL_PER_NSEC);
	ADDABLE(nsday, ns, true);
	return std::chrono::nanoseconds(nsday + ns);
}

//------------------------------------------------------------------------------
// GPS週番号の取得
int64_t ib2_mss::Mjd::wn(bool roll) const
{
	int64_t w((day_ - MJD_GPS_EPOCH) / DAYS_PER_WEEK);
	return roll ? w % MAX_GPS_WN : w;
}

//------------------------------------------------------------------------------
// GPS週秒の取得
int64_t ib2_mss::Mjd::tow(uint16_t precision) const
{
	int64_t lsb, denom;
	std::tie(lsb, denom) = lsbDecimal(precision);
	int64_t dow(day_ - wn() * DAYS_PER_WEEK - MJD_GPS_EPOCH);
	return psec_ / lsb + dow * SECONDS_PER_DAY * denom;
}

//------------------------------------------------------------------------------
// カレンダー日付文字列の取得
std::string ib2_mss::Mjd::string
(uint16_t precision, const std::string& format) const
{
	std::tm t;
	int64_t d;
	precision = std::min(precision, DIGITS);
	std::tie(t, d) = tm(precision);
	std::string form(format);
#ifdef __WIN32__
	replace(form, "%T", "%H:%M:%S");
	replace(form, "%F", "%Y-%m-%d");
#endif
	std::string::size_type is(form.find("%S"));
#ifndef __WIN32__
	if (is == std::string::npos)
		is = form.find("%T");
#endif
	if (is != std::string::npos)
		is += 2;
	char tstr1[FILENAME_MAX] = "";
	char tstr2[FILENAME_MAX] = "";
#ifdef __WIN32__
	strcpy(tstr1, ftime(t, form.substr(0, is)).c_str());
	if (is < form.length())
		strcpy(tstr2, ftime(t, form.substr(is)).c_str());
#else
	strftime(tstr1, FILENAME_MAX, form.substr(0, is).c_str(), &t);
	if (is < form.length())
		strftime(tstr2, FILENAME_MAX, form.substr(is).c_str(), &t);
#endif
	std::ostringstream ss, ssf;
	if (precision > 0)
		ssf << "." << std::setw(precision) << std::setfill('0') << d;
	ss << tstr1 << ssf.str() << tstr2;
	return ss.str();
}

//------------------------------------------------------------------------------
// デバッグ文字列の取得
std::string ib2_mss::Mjd::debug() const
{
	auto s(lldiv(psec_, DENOMINATOR));
	std::ostringstream ss;
	ss << day_ << "day " << s.quot << ".";
	ss << std::setw(DIGITS) << std::setfill('0') << s.rem << "sec ";
	ss << "(" << scale_ << ")";
	return ss.str();
}

//------------------------------------------------------------------------------
// 真夜中の時刻の取得
ib2_mss::Mjd ib2_mss::Mjd::midnight() const
{
	if (scale_ == DUR())
		throw std::domain_error("DUR is invalid scale for function midnight");
	else if (isGrobal())
		return Mjd(scale_, day_, 0);

	std::tm t;
	int64_t d;
	std::tie(t, d) = tm();
	t.tm_hour = 0;
	t.tm_min = 0;
	t.tm_sec = 0;
	return Mjd(scale_, t);
}

//------------------------------------------------------------------------------
// 整数時へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intHour() const
{
	int64_t s(psec_ / DENOMINATOR);
	if (s >= SECONDS_PER_DAY)
		s  = SECONDS_PER_DAY - 1;
	int64_t h(s / SECONDS_PER_HOUR);
	return Mjd(scale_, day_, h * SECONDS_PER_HOUR * DENOMINATOR);
}

//------------------------------------------------------------------------------
// 整数分へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intMinute() const
{
	int64_t s(psec_ / DENOMINATOR);
	if (s >= SECONDS_PER_DAY)
		s  = SECONDS_PER_DAY - 1;
	int64_t m(s / SECONDS_PER_MINUTE);
	return Mjd(scale_, day_, m * SECONDS_PER_MINUTE * DENOMINATOR);
}

//------------------------------------------------------------------------------
// 整数秒へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intSecond() const
{
	int64_t s(psec_ / DENOMINATOR);
	return Mjd(scale_, day_, s * DENOMINATOR);
}

//------------------------------------------------------------------------------
// ミリ秒へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intmsec() const
{
	int64_t ms(psec_ / DECIMAL_PER_MSEC);
	return Mjd(scale_, day_, ms * DECIMAL_PER_MSEC);
}

//------------------------------------------------------------------------------
// マイクロ秒へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intusec() const
{
	int64_t us(psec_ / DECIMAL_PER_USEC);
	return Mjd(scale_, day_, us * DECIMAL_PER_USEC);
}

//------------------------------------------------------------------------------
// ナノ秒へ切り捨てた結果の取得
ib2_mss::Mjd ib2_mss::Mjd::intnsec() const
{
	int64_t ns(psec_ / DECIMAL_PER_NSEC);
	return Mjd(scale_, day_, ns * DECIMAL_PER_NSEC);
}

//------------------------------------------------------------------------------
// 比較対象と等しい
bool ib2_mss::Mjd::equals(const Mjd& target) const
{
	return (scale_ == target.scale_ &&
			day_   == target.day_   &&
			psec_  == target.psec_);
}

//------------------------------------------------------------------------------
// 比較対象より大きい
bool ib2_mss::Mjd::isGreaterThan(const Mjd& target) const
{
	checkSameScale(target);
	if (day_ == target.day_)
		return psec_ >  target.psec_;
	return day_ >  target.day_;
}

//------------------------------------------------------------------------------
// 比較対象より小さい
bool ib2_mss::Mjd::isLessThan(const Mjd& target) const
{
	checkSameScale(target);
	if (day_ == target.day_)
		return psec_ <  target.psec_;
	return day_ < target.day_;
}

//------------------------------------------------------------------------------
// Localでない適正な時系
bool ib2_mss::Mjd::isGrobal() const
{
	auto s(std::find(SCALE_STRING.begin(), SCALE_STRING.end(), scale_));
	return s != SCALE_STRING.end();
}

//------------------------------------------------------------------------------
// 時系がLocal
bool ib2_mss::Mjd::isLocal() const
{
	if (scale_ == LOCAL() || scale_ == JST())
		return true;
	if (isGrobal())
		return false;
	auto s(std::find(LOCAL_STRING.begin(), LOCAL_STRING.end(), scale_));
	return s != LOCAL_STRING.end();
}

//------------------------------------------------------------------------------
// 閏秒を含む時系
bool ib2_mss::Mjd::withLeap() const
{
	return scale_ == UTC() || isLocal();
}

//------------------------------------------------------------------------------
// 一日の長さの取得
double ib2_mss::Mjd::secday() const
{
	return DAY + (withLeap() ? TaiUtc::leapsec(day_) : 0.);
}

//------------------------------------------------------------------------------
// 同じ時刻系であることを確認.
void ib2_mss::Mjd::checkSameScale(const Mjd& target) const
{
	if (scale_ != target.scale_)
	{
		std::string what("Time Scale is not match.");
		std::ostringstream ss;
		ss << what << ", this=" << scale_ << ", target=" << target.scale_;
		LOG_ERROR(ss.str());
		throw std::domain_error(what);
	}
}

//------------------------------------------------------------------------------
// 現在時刻の取得
ib2_mss::Mjd ib2_mss::Mjd::now()
{
	using namespace std::chrono;
	auto tp(system_clock::now());
	auto tpsec(time_point_cast<seconds>(tp));
	Mjd mjdnow(system_clock::to_time_t(tp));
	mjdnow.addnsec(duration_cast<nanoseconds>(tp - tpsec).count());
	return mjdnow;
}

//------------------------------------------------------------------------------
// 現在時刻の取得
bool ib2_mss::Mjd::validScale(const std::string& scale, bool throws)
{
	auto s(std::find(SCALE_STRING.begin(), SCALE_STRING.end(), scale));
	if (s == SCALE_STRING.end())
	{
		auto l(std::find(LOCAL_STRING.begin(), LOCAL_STRING.end(), scale));
		if (l == LOCAL_STRING.end())
		{
			if (throws)
			{
				std::string what("inputed scale is not supported");
				LOG_ERROR(what + ", inputed scale=" + scale);
				throw std::domain_error(what);
			}
			return false;
		}
	}
	return true;
}

//------------------------------------------------------------------------------
// 年月日から修正ユリウス日への変換
int64_t ib2_mss::Mjd::mjd(int64_t y, int64_t m, int64_t d)
{
	static const int64_t YEAR_MAX(Utility::powint(BASE, 15));
	static const RangeCheckerI64 YEAR
	(RangeCheckerI64::TYPE::GE_LE, -YEAR_MAX, YEAR_MAX, true);
	YEAR.valid(y, "year");

	int64_t c((14 - m) / 12);
	int64_t mjd((-c + y + 4800) * 1461 / 4);
	mjd += (c * 12 + m - 2) * 367 / 12 + d;
	mjd -= ((-c + y + 4900) / 100) * 3 / 4 + 2432076;
	return mjd;
}

//------------------------------------------------------------------------------
// 修正ユリウス日から年月日への変換
std::tuple<int16_t, int16_t, int16_t> ib2_mss::Mjd::ymd(int64_t mjd)
{
	static const RangeCheckerI64 MJD
	(RangeCheckerI64::TYPE::GE_LE,
	 Mjd::mjd(std::numeric_limits<int16_t>::min(),  1,  1),
	 Mjd::mjd(std::numeric_limits<int16_t>::max(), 12, 31), true);
	MJD.valid(mjd, "mjd");

	int64_t jd   = mjd + 2400001;
	int64_t y    = 4*(jd-1720754+3*(4*(jd-1721119)/146097+1)/4)/1461-1;
	int64_t yday = jd - 36525 * y / 100 + 3 * (y / 100 + 1) / 4 - 1721119;
	int64_t m    = (5 * yday - 3) / 153 + 3;
	int64_t d    = yday - (153 * (m - 3) + 2) / 5;
	if (m > 12)
	{
		m -= 12;
		y +=  1;
	}
	return std::make_tuple(static_cast<int16_t>(y),
						   static_cast<int16_t>(m),
						   static_cast<int16_t>(d));
}

//------------------------------------------------------------------------------
// 時分秒から0時からの経過秒への変換
int64_t ib2_mss::Mjd::psec
(int64_t h, int64_t m, int64_t s, int64_t decimal, uint16_t precision)
{
	static const int64_t DECIMAL_PER_MINUTES(SECONDS_PER_MINUTE * DENOMINATOR);
	static const int64_t DECIMAL_PER_HOUR   (SECONDS_PER_HOUR   * DENOMINATOR);
	int64_t lsb, denom;
	std::tie(lsb, denom) = lsbDecimal(precision);
	MULTIPLIABLE(h, DECIMAL_PER_HOUR, true);
	MULTIPLIABLE(m, DECIMAL_PER_MINUTES, true);
	MULTIPLIABLE(s, DENOMINATOR, true);
	MULTIPLIABLE(decimal, lsb, true);
	int64_t hp(h * DECIMAL_PER_HOUR);
	int64_t mp(m * DECIMAL_PER_MINUTES);
	int64_t sp(s * DENOMINATOR);
	int64_t psec(decimal * lsb);
	ADDABLE(psec, sp, true);
	psec += sp;
	ADDABLE(psec, mp, true);
	psec += mp;
	ADDABLE(psec, hp, true);
	psec += hp;
	return psec;
}

//------------------------------------------------------------------------------
// 0時からの経過秒から時分秒への変換
std::tuple<int64_t, int16_t, int16_t, int16_t, int64_t> ib2_mss::Mjd::hms
(int64_t decimal, uint16_t precision)
{
	int64_t denom(denominator(precision));
	auto d(lldiv(decimal, SECONDS_PER_DAY    * denom));
	auto h(lldiv(d.rem,   SECONDS_PER_HOUR   * denom));
	auto m(lldiv(h.rem,   SECONDS_PER_MINUTE * denom));
	auto s(lldiv(m.rem, denom));
	int16_t hq(static_cast<int16_t>(h.quot));
	int16_t mq(static_cast<int16_t>(m.quot));
	int16_t sq(static_cast<int16_t>(s.quot));
	return std::make_tuple(d.quot, hq, mq, sq, s.rem);
}

//------------------------------------------------------------------------------
// 実数秒をピコ秒に変換
int64_t ib2_mss::Mjd::psec(double s, bool round, uint16_t precision)
{
	double eps(std::abs(s) * std::numeric_limits<double>::epsilon());
	uint16_t digits(static_cast<uint16_t>(-std::ceil(std::log10(eps))));	
	int64_t lsb, denom;
	std::tie(lsb, denom) = lsbDecimal(std::min(digits, precision));
	s *= static_cast<double>(denom);
	if (round)
		s = std::round(s);
	INT64.valid(s, "s * denom");
	int64_t si(static_cast<int64_t>(s));
	MULTIPLIABLE(si, lsb, true);
	return si * lsb;
}

//------------------------------------------------------------------------------
// 実数秒を日とピコ秒に変換
std::pair<int64_t, int64_t> ib2_mss::Mjd::daypsec
(double s, bool round, uint16_t precision)
{
	double eps(std::abs(s) * std::numeric_limits<double>::epsilon());
	if (eps > DAY)
		return std::make_pair(Utility::int64(s / DAY), 0);

	double d(floor(s / DAY));
	INT64.valid(d, "d");
	int64_t day(static_cast<int64_t>(d));
	s -= d * DAY;
	if (eps > 1.)
		return std::make_pair(day, Utility::int64(s) * DENOMINATOR);

	auto digits(static_cast<uint16_t>(-std::ceil(std::log10(eps))));
	auto ps(psec(s, round, std::min(digits, precision)));
	if (ps >= DECIMAL_PER_DAY)
	{
		ps -= DECIMAL_PER_DAY;
		++day;
	}
	return std::make_pair(day, ps);
}

//------------------------------------------------------------------------------
// time_tを日とピコ秒に変換
std::pair<int64_t, int64_t> ib2_mss::Mjd::daypsec(time_t t)
{
	auto d(lldiv(static_cast<int64_t>(t), SECONDS_PER_DAY));
	if (d.rem < 0)
	{
		--d.quot;
		d.rem += SECONDS_PER_DAY;
	}
	auto day(d.quot + MJD_TIMET_EPOCH);
	auto psec(d.rem * DENOMINATOR);
	return std::make_pair(day, psec);
}

//------------------------------------------------------------------------------
// 日とピコ秒をtime_tと小数秒(ピコ秒)に変換
std::pair<time_t, int64_t> ib2_mss::Mjd::tt(int64_t mjd, int64_t psec)
{
	static RangeCheckerI64 MJD_RANGE
	(RangeCheckerI64::TYPE::GT_LT,
	 std::numeric_limits<int64_t>::min() / SECONDS_PER_DAY + MJD_TIMET_EPOCH,
	 std::numeric_limits<int64_t>::max() / SECONDS_PER_DAY - MJD_TIMET_EPOCH, 
	 true);
	MJD_RANGE.valid(mjd, "mjd");

	auto s(lldiv(psec, DENOMINATOR));
	int64_t t((mjd - MJD_TIMET_EPOCH) * SECONDS_PER_DAY + s.quot);
	return std::make_pair(static_cast<time_t>(t), s.rem);
}

//------------------------------------------------------------------------------
// 時分秒実数から時分秒小数秒への分割
std::tuple<int16_t, int16_t, int16_t, int64_t> ib2_mss::Mjd::separate
(double o, bool round, uint16_t precision)
{
	static const int64_t DENOMINATOR_2DIGITS(100);
	int64_t denom(denominator(precision)); 
	o *= static_cast<double>(denom);
	if (round)
		o = std::round(o);
	INT64.valid(o, "o * denom");
	int64_t oi(static_cast<int64_t>(o));
	auto s(lldiv(oi, denom));
	auto m(lldiv(s.quot, DENOMINATOR_2DIGITS));
	auto h(lldiv(m.quot, DENOMINATOR_2DIGITS));
	return std::make_tuple(h.quot, h.rem, m.rem, s.rem);
}

//------------------------------------------------------------------------------
// 時分秒実数から時分秒小数秒への分割
double ib2_mss::Mjd::sec(int64_t d, uint16_t precision)
{
	double lsb(pow(BASE, -static_cast<double>(precision)));
	return static_cast<double>(d) * lsb;
}

//------------------------------------------------------------------------------
// DUR時系文字列の参照
const std::string& ib2_mss::Mjd::DUR()
{
	return SCALE_STRING.front();
}

//------------------------------------------------------------------------------
// TDB時系文字列の参照
const std::string& ib2_mss::Mjd::TDB()
{
	return SCALE_STRING.at(1);
}

//------------------------------------------------------------------------------
// TDT時系文字列の参照
const std::string& ib2_mss::Mjd::TDT()
{
	return SCALE_STRING.at(2);
}

//------------------------------------------------------------------------------
// TAI時系文字列の参照
const std::string& ib2_mss::Mjd::TAI()
{
	return SCALE_STRING.at(3);
}

//------------------------------------------------------------------------------
// GPS時系文字列の参照
const std::string& ib2_mss::Mjd::GPS()
{
	return SCALE_STRING.at(4);
}

//------------------------------------------------------------------------------
// UT1時系文字列の参照
const std::string& ib2_mss::Mjd::UT1()
{
	return SCALE_STRING.at(5);
}

//------------------------------------------------------------------------------
// UTC時系文字列の参照
const std::string& ib2_mss::Mjd::UTC()
{
	return SCALE_STRING.back();
}

//------------------------------------------------------------------------------
// LOCAL時系文字列の参照
const std::string& ib2_mss::Mjd::LOCAL()
{
	return LOCAL_STRING.front();
}

//------------------------------------------------------------------------------
// TDB時系文字列の参照
const std::string& ib2_mss::Mjd::JST()
{
	return LOCAL_STRING.at(1);
}

//------------------------------------------------------------------------------
// 加算演算子
ib2_mss::Mjd operator+(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	using namespace ib2_mss;
	if (left.scale() == Mjd::DUR())
	{
		Mjd result(right);
		result += left;
		return result;
	}
	else
	{
		Mjd result(left);
		result += right;
		return result;
	}
}

//------------------------------------------------------------------------------
// 加算演算子
ib2_mss::Mjd operator+(const ib2_mss::Mjd& left, double right)
{
	ib2_mss::Mjd result(left);
	result += right;
	return result;
}

//------------------------------------------------------------------------------
// 減算演算子
ib2_mss::Mjd operator-(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	ib2_mss::Mjd result(left);
	result -= right;
	return result;
}

//------------------------------------------------------------------------------
// 減算演算子
ib2_mss::Mjd operator-(const ib2_mss::Mjd& left, double right)
{
	ib2_mss::Mjd result(left);
	result -= right;
	return result;
}

//------------------------------------------------------------------------------
// 等号比較演算子.
bool operator==(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return left.equals(right);
}

//------------------------------------------------------------------------------
// 不等比較演算子.
bool operator!=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return !left.equals(right);
}

//------------------------------------------------------------------------------
// 大なり不等号比較演算子.
bool operator>(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return left.isGreaterThan(right);
}

//------------------------------------------------------------------------------
// 小なり不等号比較演算子.
bool operator<(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return left.isLessThan(right);
}

//------------------------------------------------------------------------------
// 以上不等号比較演算子.
bool operator>=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return !left.isLessThan(right);
}

//------------------------------------------------------------------------------
// 以下不等号比較演算子.
bool operator<=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right)
{
	return !left.isGreaterThan(right);
}

//------------------------------------------------------------------------------
// 出力演算子.
std::ostream& operator<<(std::ostream& os, const ib2_mss::Mjd& t)
{
	using namespace ib2_mss;
	if (t.scale() == Mjd::DUR())
		os << t.debug();
	else
		os << t.string() << "(" << t.scale() << ")";
	return os;
}

// End Of File -----------------------------------------------------------------
