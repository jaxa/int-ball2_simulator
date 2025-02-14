
#include "guidance_control_common/TaiUtc.h"
#include "guidance_control_common/RangeChecker.h"
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Log.h"

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <cassert>

//------------------------------------------------------------------------------
// 定数
namespace
{
	/** TAI開始時修正ユリウス日(1958年1月1日) */
	const long long MJD_START_TAI = 36204;
	
	/** 係数最小値 */
	const double EPS_COEF = 0.00000005;
	
	/** 参照番号のデフォルト値 */
	const size_t DEFAULT_INDEX(std::numeric_limits<size_t>::max());
}

/**
 * @brief TAI-UTC実装クラス
 */
class ib2_mss::TaiUtc::Implement
{
	//--------------------------------------------------------------------------
	// 列挙子/構造体
private:
	/** 
	 * @brief TAI-UTCデータ構造体
	 */
	struct DATA
	{
		long long mjd;	///< 修正ユリウス日
		double taiutc;	///< TAI-UTC[sec]
		double epoch;	///< 係数の元期の修正ユリウス日
		double coef;	///< 係数
	};
	
	/** 
	 * @brief TAI-UTCデータ時刻の比較構造体
	 */
	struct LESS_MJD
	{
		bool operator() (const DATA& left, long long right) const
		{
			return (left.mjd < right);
		}
		
		bool operator () (long long left, const DATA& right) const
		{
			return (left < right.mjd);
		}
		
		bool operator () (const DATA& left, const DATA& right) const
		{
			return (left.mjd < right.mjd);
		}
	};

	//--------------------------------------------------------------------------
	// コンストラクタ/デストラクタ
private:
	/** デフォルトコンストラクタ. */
	Implement()  :
	list_(0), index_(DEFAULT_INDEX), step_(DEFAULT_INDEX)
	{
	}
	
	/** デストラクタ */
	~Implement() = default;

	//--------------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	Implement(const Implement&) = delete;
	
	/** コピー代入演算子. */
	Implement& operator=(const Implement&) = delete;

	/** ムーブコンストラクタ. */
	Implement(Implement&&) = delete;
	
	/** ムーブ代入演算子. */
	Implement& operator=(Implement&&) = delete;
	
	//--------------------------------------------------------------------------
	// 操作(Setter)
public:
	/** TAI-UTCデータの構成.
	 * TAI-UTCデータファイルを読み込む。
	 * @param [in] filename TAI-UTCデータファイルパス
	 */
	void configure(const std::string& filename)
	{
		list_.clear();
		index_ = DEFAULT_INDEX;
		step_  = DEFAULT_INDEX;
		if (filename.empty())
			return;
		
		size_t lineno(0);
		try
		{
			FileReader f(filename);
			list_.reserve(f.lines());
			for (auto& line: f.line())
			{
				DATA d(readline(line));
				list_.push_back(d);
				if (step_ > list_.size() && d.coef <= 0.)
					step_ = lineno;
				++lineno;
			}
		}
		catch (const std::exception& e)
		{
			static const std::string MESSAGE("TAI-UTC data file");
			LOG_ERROR(Log::errorFileLine(e.what(), MESSAGE, filename, lineno));
			throw;
		}
	}

	//--------------------------------------------------------------------------
	// 属性(Getter)
public:
	/** インスタンスの取得(シングルトンパターン).
	 * @return インスタンスへの参照
	 */
	static Implement& instance()
	{
		static Implement instance;
		return instance;
	}
	
	/** UTCMJDが入力されたときのTAI-UTCの計算
	 * @param [in] utcmjd 修正ユリウス日(UTC)
	 * @param [in] secday 0時からの経過秒(UTC)
	 * @return TAI-UTC [sec]
	 */
	double dAT(long long utcmjd, double secday) const
	{
		if (list_.empty() || utcmjd < MJD_START_TAI)
			return 0.;
		
		auto it(search(utcmjd));
		if (std::abs(it->coef) < EPS_COEF)
			return it->taiutc;
		
		double d((static_cast<double>(utcmjd) - it->epoch) + secday / DAY);
		return d * it->coef + it->taiutc;
	}
	
	/** 閏秒の取得.
	 * @param [in]  utcmjd 修正ユリウス日(UTC)
	 * @return 閏秒 [sec]
	 */
	double leapsec(long long utcmjd) const
	{
		if (list_.empty())
			return 0.;
		auto it(search(utcmjd));
		if (std::abs((*it).coef) >= EPS_COEF)
			return 0.;
		auto next(it + 1);
		if (next == list_.end())
			return 0.;
		if (utcmjd+1 < (*next).mjd)
			return 0.;
		return next->taiutc - it->taiutc;
	}
	
	/** 閏秒の取得.
	 * @param [in]  from 開始修正ユリウス日(UTC)
	 * @param [in]  to   終了修正ユリウス日(UTC)
	 * @return 閏秒 [sec]
	 */
	double leapsec(long long from, long long to) const
	{
		if (list_.empty())
			return 0.;
		
		auto it1(search(to));
		auto it0(search(from));
		size_t i1(std::distance(list_.begin(), it1));
		size_t i0(std::distance(list_.begin(), it0));
		double dt1(i1 < step_ ? list_.at(step_).taiutc : it1->taiutc);
		double dt0(i0 < step_ ? list_.at(step_).taiutc : it0->taiutc);
		return dt1 - dt0;
	}
	
	/** 閏秒の存在確認.
	 * @param [in] utcmjd 修正ユリウス日(UTC)
	 * @retval true 閏秒がある
	 * @retval false 閏秒がない
	 */
	bool step(long long utcmjd) const
	{
		if (list_.empty())
			return false;
		return (list_.at(step_).mjd <= utcmjd && utcmjd < list_.back().mjd);
	}
	
	/** 直前の閏秒挿入日の取得
	 * @param [in] utcmjd 修正ユリウス日(UTC)
	 * @return 直前の閏秒挿入日の修正ユリウス日(UTC)
	 */
	long long before(long long utcmjd) const
	{
		if (list_.empty() || utcmjd < list_.at(step_).mjd)
			return std::numeric_limits<long long>::lowest();
		return search(utcmjd)->mjd;
	}
	
	/** 直後の閏秒挿入日の取得
	 * @param [in] utcmjd 修正ユリウス日(UTC)
	 * @return 直後の閏秒挿入日の修正ユリウス日(UTC)
	 */
	long long after(long long utcmjd) const
	{
		if (list_.empty() || utcmjd >= list_.back().mjd)
			return std::numeric_limits<long long>::max();
		auto it(search(utcmjd));
		return (++it)->mjd;
	}
	
	//--------------------------------------------------------------------------
	// 実装
private:
	/** TAI-UTCデータの読み込み.
	 * TAI-UTCデータ１行文字列からTAI_UTC構造体を取得
	 * @param [in] line TAI-UTCデータ１行文字列
	 * @return TAI_UTC構造体
	 */
	static DATA readline(const std::string& line)
	{
		try
		{
			std::string::size_type ijd(line.find("JD"));
			std::string::size_type idt(line.find("TAI-UTC"));
			std::string::size_type iep(line.find("MJD"));
			RangeCheckerUI64::equal(ijd, 14, true, "ijd", "JD");
			RangeCheckerUI64::equal(idt, 28, true, "idt", "TAI-UTC");
			RangeCheckerUI64::equal(iep, 54, true, "iep", "MJD");
			
			double jd    (FileReader::value(line.substr(16, 12)));
			double taiutc(FileReader::value(line.substr(36, 13)));
			double epoch (FileReader::value(line.substr(59,  7)));
			double coef  (FileReader::value(line.substr(69, 10)));
			long long mjd(static_cast<long long>(jd -  2400000.5));
			return DATA{mjd, taiutc, epoch, coef};
		}
		catch (const std::exception& e)
		{
			LOG_ERROR(Log::caughtException(e.what(), "line:"+line));
			throw;
		}
	}
	
	/** TAI-UTCデータの探索
	 * 入力された修正ユリウス日以下の最大テーブルデータを取得する。
	 * @param [in] mjd 修正ユリウス日(UTC)
	 * @return TAI_UTC構造体
	 * @pre list_がからでないこと
	 */
	std::vector<DATA>::const_iterator search(long long mjd) const
	{
		assert(!list_.empty());
		if (index_ < list_.size())
		{
			auto it(list_.begin() + index_);
			while (it != list_.begin() && mjd < it->mjd)
				--it;
			while (it+1 != list_.end() && mjd >= (it+1)->mjd)
				++it;
			index_ = std::distance(list_.begin(), it);
			return it;
		}
		else
		{
			auto it(std::upper_bound(list_.begin(),list_.end(),mjd,LESS_MJD()));
			if (it != list_.begin())
				--it;
			index_ = std::distance(list_.begin(), it);
			return it;
		}
	}
	
	//--------------------------------------------------------------------------
	// メンバー変数
	/** TAI-UTCデータ */
	std::vector<DATA> list_;
	
	/** TAI-UTCデータ参照インデックス */
	mutable size_t index_;
	
	/** 階段状UTC開始インデックス */
	size_t step_;
};

//------------------------------------------------------------------------------
// TAI-UTCデータの構成
void ib2_mss::TaiUtc::configure(const std::string& filename)
{
	Implement::instance().configure(filename);
}

//------------------------------------------------------------------------------
// TAI-UTCの取得
double ib2_mss::TaiUtc::dAT(long long utcmjd, double secday)
{
	return Implement::instance().dAT(utcmjd, secday);
}

//------------------------------------------------------------------------------
// 閏秒の取得
double ib2_mss::TaiUtc::leapsec(long long utcmjd)
{
	return Implement::instance().leapsec(utcmjd);
}

//------------------------------------------------------------------------------
// 閏秒の取得
double ib2_mss::TaiUtc::leapsec(long long from, long long to)
{
	return Implement::instance().leapsec(from, to);
}

//------------------------------------------------------------------------------
// 閏秒の存在確認
bool ib2_mss::TaiUtc::step(long long utcmjd)
{
	return Implement::instance().step(utcmjd);
}

//------------------------------------------------------------------------------
// 直前の閏秒挿入日の取得
long long ib2_mss::TaiUtc::before(long long utcmjd)
{
	return Implement::instance().before(utcmjd);
}

//------------------------------------------------------------------------------
// 直後の閏秒挿入日の取得
long long ib2_mss::TaiUtc::after(long long utcmjd)
{
	return Implement::instance().after(utcmjd);
}

// End Of File -----------------------------------------------------------------
