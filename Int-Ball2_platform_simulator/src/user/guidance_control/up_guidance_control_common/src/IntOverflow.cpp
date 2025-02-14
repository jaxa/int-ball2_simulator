
#include "guidance_control_common/IntOverflow.h"
#include "guidance_control_common/Log.h"

#include <sstream>
#include <stdexcept>
#include <limits>
#include <cstdlib>

//------------------------------------------------------------------------------
// 加算可能判定
template <typename T>
bool ib2_mss::IntOverflow::addable
(T l, T r, bool throws, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	if (r >= 0)
	{
		if (l <= std::numeric_limits<T>::max() - r)
			return true;
	}
	else if (l >= std::numeric_limits<T>::min() - r)
		return true;
	if (!throws)
		return false;
	std::string what("integer overflow occured at addition");
	std::ostringstream ss;
	ss << what << ", left=" << l << ", right=" << r;
	Log::error(ss.str(), file, function, lineno);
	throw std::overflow_error(what);
}

//------------------------------------------------------------------------------
// 減算可能判定
template <typename T>
bool ib2_mss::IntOverflow::subtractable
(T l, T r, bool throws, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	if (r >= 0)
	{
		if (l >= std::numeric_limits<T>::min() + r)
			return true;
	}
	else if (l <= std::numeric_limits<T>::max() + r)
		return true;
	if (!throws)
		return false;
	std::string what("integer overflow occured at subtraction");
	std::ostringstream ss;
	ss << what << ", left=" << l << ", right=" << r;
	Log::error(ss.str(), file, function, lineno);
	throw std::overflow_error(what);
}

//------------------------------------------------------------------------------
// 乗算可能判定
template <typename T>
bool ib2_mss::IntOverflow::multipliable
(T l, T r, bool throws, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	if (l == 0 || r == 0)
		return true;
	else if (r > 0)
	{
		if (l > 0)
		{
			if (l <= std::numeric_limits<T>::max() / r)
				return true;
		}
		else if(l >= std::numeric_limits<T>::min() / r)
			return true;
	}
	else if(l > 0)
	{
		if (r >= std::numeric_limits<T>::min() / l)
			return true;
	}
	else
	{
		auto nmin(std::numeric_limits<T>::min());
		auto nmax(std::numeric_limits<T>::max());
		if (nmin + nmax <= 0)
		{
			if (nmax / l <= r)
				return true;
		}
		else
		{
			if (nmax / -r >= -l)
				return true;
		}
	}
	if (!throws)
		return false;
	std::string what("integer overflow occured at multiplication");
	std::ostringstream ss;
	ss << what << ", left=" << l << ", right=" << r;
	Log::error(ss.str(), file, function, lineno);
	throw std::overflow_error(what);
}

//------------------------------------------------------------------------------
// 除算可能判定
template <typename T>
bool ib2_mss::IntOverflow::dividable
(T l, T r, bool throws, const std::string& file,
 const std::string& function, unsigned long lineno)
{
	if (r > 0)
		return true;
	else if (r < 0)
	{
		auto nmin(std::numeric_limits<T>::min());
		auto nmax(std::numeric_limits<T>::max());
		auto flag(nmin + nmax);
		if (flag == 0)
			return true;
		else if (flag < 0)
		{
			if (l > r || r > nmin / nmax || nmax * r < l - r)
				return true;
		}
		else
		{
			if (l <= 0 || nmax / -nmin < -r || l + r < nmin * r)
				return true;
		}
	}
	if (!throws)
		return false;
	std::string what("integer overflow occured at division");
	std::ostringstream ss;
	ss << what << ", left=" << l << ", right=" << r;
	Log::error(ss.str(), file, function, lineno);
	throw std::overflow_error(what);
}

//------------------------------------------------------------------------------
// 明示的テンプレートのインスタンス化
template bool ib2_mss::IntOverflow::addable
(int16_t l, int16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::addable
(int32_t l, int32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::addable
(int64_t l, int64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::addable
(uint16_t l, uint16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::addable
(uint32_t l, uint32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::addable
(uint64_t l, uint64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);

template bool ib2_mss::IntOverflow::subtractable
(int16_t l, int16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::subtractable
(int32_t l, int32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::subtractable
(int64_t l, int64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::subtractable
(uint16_t l, uint16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::subtractable
(uint32_t l, uint32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::subtractable
(uint64_t l, uint64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);

template bool ib2_mss::IntOverflow::multipliable
(int16_t l, int16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::multipliable
(int32_t l, int32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::multipliable
(int64_t l, int64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::multipliable
(uint16_t l, uint16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::multipliable
(uint32_t l, uint32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::multipliable
(uint64_t l, uint64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);

template bool ib2_mss::IntOverflow::dividable
(int16_t l, int16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::dividable
(int32_t l, int32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::dividable
(int64_t l, int64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::dividable
(uint16_t l, uint16_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::dividable
(uint32_t l, uint32_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);
template bool ib2_mss::IntOverflow::dividable
(uint64_t l, uint64_t r, bool throws,
 const std::string& file, const std::string& function, unsigned long lineno);

// End Of File -----------------------------------------------------------------
