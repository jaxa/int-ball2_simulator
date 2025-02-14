
#include "guidance_control_common/RangeChecker.h"
#include "guidance_control_common/Mjd.h"
#include "guidance_control_common/Log.h"

#include <stdexcept>
#include <sstream>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 範囲外の時の例外メッセージ */
	const std::string INVALID_RANGE("values is out of range");

	/** 判定条件文字列小なり */
	const std::string LT(" < ");

	/** 判定条件文字列以下 */
	const std::string LE(" <= ");

	/** 判定条件文字列大なり */
	const std::string GT(" > ");

	/** 判定条件文字列以上 */
	const std::string GE(" >= ");

	/** 判定値１つの時の範囲外ログ出力
	 * @param [in] x 検査対象値
	 * @param [in] thr 判定値
	 * @param [in] name 検査対象値の名前
	 * @param [in] condition 判定条件文字列
	 * @param [in] file ログに出力するソースファイル名
	 * @param [in] function ログに出力する関数名
	 * @param [in] lineno ログに出力するソースファイル行番号
	 * @param [in] throws 例外を投げるフラグ
	 */
	template <typename T>
	void invalidlog1
	(T x, T thr, const std::string& name, const std::string& condition,
	 const std::string& file, const std::string& function,
	 unsigned long lineno, bool throws)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << name << " inputed:" << x << ", " << name << condition << thr;
		ib2_mss::Log::warn(ss.str(), file, function, lineno);
		if (throws)
			throw std::domain_error(INVALID_RANGE);
	}

	/** 判定値２つの時の範囲外ログ出力
	 * @param [in] x 検査対象値
	 * @param [in] min 最小値
	 * @param [in] max 最大値
	 * @param [in] name 検査対象値の名前
	 * @param [in] c1 判定条件文字列１
	 * @param [in] c2 判定条件文字列２
	 * @param [in] file ログに出力するソースファイル名
	 * @param [in] function ログに出力する関数名
	 * @param [in] lineno ログに出力するソースファイル行番号
	 * @param [in] throws 例外を投げるフラグ
	 */
	template <typename T>
	void invalidlog2
	(T x, T min, T max, const std::string& name,
	 const std::string& c1, const std::string& c2,
	 const std::string& file, const std::string& function,
	 unsigned long lineno, bool throws)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << name << " inputed:" << x << ", ";
		ss << min << c1 << name << c2 << max;
		ib2_mss::Log::warn(ss.str(), file, function, lineno);
		if (throws)
			throw std::domain_error(INVALID_RANGE);
	}

	/** 判定値２つで外側が指定範囲の時の範囲外ログ出力
	 * @param [in] x 検査対象値
	 * @param [in] min 最小値
	 * @param [in] max 最大値
	 * @param [in] name 検査対象値の名前
	 * @param [in] c1 判定条件文字列１
	 * @param [in] c2 判定条件文字列２
	 * @param [in] file ログに出力するソースファイル名
	 * @param [in] function ログに出力する関数名
	 * @param [in] lineno ログに出力するソースファイル行番号
	 * @param [in] throws 例外を投げるフラグ
	 */
	template <typename T>
	void invalidlog2o
	(T x, T min, T max, const std::string& name,
	 const std::string& c1, const std::string& c2,
	 const std::string& file, const std::string& function,
	 unsigned long lineno, bool throws)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << name << " inputed:" << x << ", ";
		ss << name << c1 << min << ", " << max << c2 << name;
		ib2_mss::Log::warn(ss.str(), file, function, lineno);
		if (throws)
			throw std::domain_error(INVALID_RANGE);
	}
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ.
namespace ib2_mss
{
	template <typename T>
	RangeChecker<T>::RangeChecker() :
	type_(TYPE::LT), min_(0), max_(0), throws_(false)
	{
	}
	template <>
	RangeChecker<Mjd>::RangeChecker() :
	type_(TYPE::LT), min_(), max_(), throws_(false)
	{
	}
}

//------------------------------------------------------------------------------
// 判定値１つのコンストラクタ.
template <typename T>
ib2_mss::RangeChecker<T>::RangeChecker
(const TYPE& type, T thr, bool throws) :
type_(type), min_(thr), max_(thr), throws_(throws)
{
}

//------------------------------------------------------------------------------
// コンストラクタ.
template <typename T>
ib2_mss::RangeChecker<T>::RangeChecker
(const TYPE& type, T min, T max, bool throws) :
type_(type), min_(min), max_(max), throws_(throws)
{
}

//------------------------------------------------------------------------------
// デストラクタ.
template <typename T>
ib2_mss::RangeChecker<T>::~RangeChecker() = default;

//------------------------------------------------------------------------------
// 範囲内の確認
template <typename T>
bool ib2_mss::RangeChecker<T>::valid(T x, const std::string& name) const
{
	try
	{
		switch (type_)
		{
//			case TYPE::LT: return lt(x, name);
			case TYPE::LE: return le(x, name);
			case TYPE::GT: return gt(x, name);
			case TYPE::GE: return ge(x, name);
			case TYPE::GT_LT: return gtlt(x, name);
			case TYPE::GT_LE: return gtle(x, name);
			case TYPE::GE_LT: return gelt(x, name);
			case TYPE::GE_LE: return gele(x, name);
			case TYPE::LT_GT: return ltgt(x, name);
			case TYPE::LT_GE: return ltge(x, name);
			case TYPE::LE_GT: return legt(x, name);
			case TYPE::LE_GE: return lege(x, name);
			default: return lt(x, name);
		}
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
		throw;
	}
}

//------------------------------------------------------------------------------
// 最小値より小さい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::lt(T x, const std::string& name) const
{
	if (x < min_)
		return true;
	invalidlog1(x, min_, name, LT, __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以下の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::le(T x, const std::string& name) const
{
	if (x <= min_)
		return true;
	invalidlog1(x, min_, name, LE, __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値より大きい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::gt(T x, const std::string& name) const
{
	if (x > min_)
		return true;
	invalidlog1(x, min_, name, GT, __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以上の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::ge(T x, const std::string& name) const
{
	if (x >= min_)
		return true;
	invalidlog1(x, min_, name, GE, __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値より大きく最大値より小さい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::gtlt(T x, const std::string& name) const
{
	if (min_ < x && x < max_)
		return true;
	invalidlog2(x, min_, max_, name, LT, LT,
				__FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値より大きく最大値以下の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::gtle(T x, const std::string& name) const
{
	if (min_ < x && x <= max_)
		return true;
	invalidlog2(x, min_, max_, name, LT, LE,
				__FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以上で最大値より小さい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::gelt(T x, const std::string& name) const
{
	if (min_ <= x && x < max_)
		return true;
	invalidlog2(x, min_, max_, name, LE, LT,
				__FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以上で最大値以下の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::gele(T x, const std::string& name) const
{
	if (min_ <= x && x <= max_)
		return true;
	invalidlog2(x, min_, max_, name, LE, LE,
				__FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値より小さいまたは最大値より大きい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::ltgt(T x, const std::string& name) const
{
	if (x < min_ || max_ < x)
		return true;
	invalidlog2o(x, min_, max_, name, LT, LT,
				 __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値より小さいまたは最大値以上の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::ltge(T x, const std::string& name) const
{
	if (x < min_ || max_ <= x)
		return true;
	invalidlog2o(x, min_, max_, name, LT, LE,
				 __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以下または最大値より大きい判定
template <typename T>
bool ib2_mss::RangeChecker<T>::legt(T x, const std::string& name) const
{
	if (x <= min_ || max_ < x)
		return true;
	invalidlog2o(x, min_, max_, name, LE, LT,
				 __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

//------------------------------------------------------------------------------
// 最小値以下または最大値以上の判定
template <typename T>
bool ib2_mss::RangeChecker<T>::lege(T x, const std::string& name) const
{
	if (x <= min_ || max_ <= x)
		return true;
	invalidlog2o(x, min_, max_, name, LE, LE,
				 __FILE__, __FUNCTION__, __LINE__, throws_);
	return false;
}

namespace ib2_mss
{
	//--------------------------------------------------------------------------
	// 正値判定
	template <typename T>
	bool RangeChecker<T>::positive
	(T x, bool throws, const std::string& name)
	{
		RangeChecker<T> RC(TYPE::GT, 0, throws);
		return RC.valid(x, name);
	}
	template <>
	bool RangeChecker<ib2_mss::Mjd>::positive
	(Mjd x, bool throws, const std::string& name)
	{
		RangeCheckerT RC(TYPE::GT, Mjd(), throws);
		return RC.valid(x, name);
	}

	//--------------------------------------------------------------------------
	// 負値判定
	template <typename T>
	bool RangeChecker<T>::negative
	(T x, bool throws, const std::string& name)
	{
		RangeChecker<T> RC(TYPE::LT, 0, throws);
		return RC.valid(x, name);
	}
	template <>
	bool RangeChecker<ib2_mss::Mjd>::negative
	(Mjd x, bool throws, const std::string& name)
	{
		RangeCheckerT RC(TYPE::LT, Mjd(), throws);
		return RC.valid(x, name);
	}

	//--------------------------------------------------------------------------
	// 非負判定
	template <typename T>
	bool RangeChecker<T>::notPositive
	(T x, bool throws, const std::string& name)
	{
		RangeChecker<T> RC(TYPE::LE, 0, throws);
		return RC.valid(x, name);
	}
	template <>
	bool RangeChecker<ib2_mss::Mjd>::notPositive
	(Mjd x, bool throws, const std::string& name)
	{
		RangeCheckerT RC(TYPE::LE, Mjd(), throws);
		return RC.valid(x, name);
	}

	//--------------------------------------------------------------------------
	// 非正判定
	template <typename T>
	bool RangeChecker<T>::notNegative
	(T x, bool throws, const std::string& name)
	{
		RangeChecker<T> RC(TYPE::GE, 0, throws);
		return RC.valid(x, name);
	}
	template <>
	bool RangeChecker<ib2_mss::Mjd>::notNegative
	(Mjd x, bool throws, const std::string& name)
	{
		RangeCheckerT RC(TYPE::GE, Mjd(), throws);
		return RC.valid(x, name);
	}
}
//------------------------------------------------------------------------------
// 等値判定
template <typename T>
bool ib2_mss::RangeChecker<T>::equal
(T l, T r, bool throws, const std::string& namel, const std::string& namer)
{
	if (l < r || l > r)
	{
		std::ostringstream ss;
		ss.precision(16);
		ss << namel << " :" << l << " must equal to " << namer << r;
		LOG_WARN(ss.str());
		if (throws)
			throw std::domain_error(INVALID_RANGE);
		return false;
	}
	else
	{
		return true;
	}
}

//------------------------------------------------------------------------------
// 単調増加/単調減少判定
template <typename T>
bool ib2_mss::RangeChecker<T>::monotonic
(const TYPE &type, const std::vector<T>& x, bool throws,
 const std::string& name)
{
	for (size_t i = 1; i < x.size(); ++i)
	{
		bool ok(true);
		switch (type)
		{
			case TYPE::LT:	ok = x.at(i-1) <  x.at(i);	break;
			case TYPE::LE:	ok = x.at(i-1) <= x.at(i);	break;
			case TYPE::GT: 	ok = x.at(i-1) >  x.at(i);	break;
			case TYPE::GE: 	ok = x.at(i-1) >= x.at(i);	break;
			default:
				throw std::domain_error("invalid type for monotonic");
		}
		if (!ok)
		{
			std::string what("arrays are not monotonic");
			std::ostringstream ss;
			ss.precision(16);
			ss << what << ", " << name << "[" << i-1 << "]=" << x.at(i-1);
			ss << (type == TYPE::LT ? LT :
				   type == TYPE::LE ? LE :
				   type == TYPE::GT ? GT : GE);
			ss << name << "[" << i   << "]=" << x.at(i);
			LOG_WARN(ss.str());
			if (throws)
				throw std::domain_error(what);
			return false;
		}
	}
	return true;
}

//------------------------------------------------------------------------------
// 明示的テンプレートのインスタンス化
template class ib2_mss::RangeChecker<int8_t>;
template class ib2_mss::RangeChecker<int16_t>;
template class ib2_mss::RangeChecker<int32_t>;
template class ib2_mss::RangeChecker<int64_t>;

template class ib2_mss::RangeChecker<uint8_t>;
template class ib2_mss::RangeChecker<uint16_t>;
template class ib2_mss::RangeChecker<uint32_t>;
template class ib2_mss::RangeChecker<uint64_t>;

template class ib2_mss::RangeChecker<float>;
template class ib2_mss::RangeChecker<double>;
template class ib2_mss::RangeChecker<long double>;
template class ib2_mss::RangeChecker<ib2_mss::Mjd>;

// End Of File -----------------------------------------------------------------
