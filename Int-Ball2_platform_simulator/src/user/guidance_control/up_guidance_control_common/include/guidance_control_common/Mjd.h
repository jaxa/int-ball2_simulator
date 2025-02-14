
#pragma once

#include <cstdint>
#include <ctime>
#include <tuple>
#include <utility>
#include <string>
#include <vector>
#include <chrono>

namespace ib2_mss
{
	/**
	 * @brief 修正ユリウス日.
	 *
	 * 修正ユリウス日と0時からの経過秒数と小数秒により時刻を表現する。
	 */
	class Mjd final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Mjd();
		
		/** time_t時刻によるコンストラクタ.
		 * @param [in] t 1970年1月1日0時0分0秒UTCからの経過秒(閏秒非考慮)
		 */
		explicit Mjd(time_t t);
		
		/** duration時刻によるコンストラクタ.
		 * @param [in] d duration時刻
		 */
		explicit Mjd(const std::chrono::nanoseconds& d);
		
		/** time_point時刻によるコンストラクタ.
		 * @param [in] tp time_point時刻
		 */
		explicit Mjd(const std::chrono::time_point
					 <std::chrono::system_clock, std::chrono::nanoseconds>& tp);
		
		/** 時刻構造体によるコンストラクタ.
		 * @param [in] scale 時刻系
		 * @param [in] t 時刻構造体(local時刻)
		 */
		Mjd(const std::string& scale, const std::tm& t);
		
		/** 実数値によるコンストラクタ.
		 * @param [in] scale 時刻系
		 * @param [in] day 修正ユリウス日実数値
		 */
		Mjd(const std::string& scale, double day);
		
		/** 値指定によるコンストラクタ.
		 * @param [in] scale 時刻系
		 * @param [in] day 修正ユリウス日
		 * @param [in] psec 0時からの経過時間[psec]
		 */
		Mjd(const std::string& scale, int64_t day, int64_t psec);
		
		/** GPS時刻フォーマットによるコンストラクタ.
		 * @param [in] nroll ロールオーバ回数
		 * @param [in] wn GPS週番号
		 * @param [in] tow GPS週時間
		 * @param [in] precision 小数秒の桁数
		 * @pre precisionは9以下
		 */
		Mjd(int64_t nroll, int64_t wn, int64_t tow,
			uint16_t precision = PRECISION);

		/** 年月日時分秒によるコンストラクタ.
		 * @param [in] scale 時刻系
		 * @param [in] year 年
		 * @param [in] month 月
		 * @param [in] mday 日
		 * @param [in] hour 時
		 * @param [in] minute 分
		 * @param [in] second 秒
		 * @param [in] decimal 小数秒
		 * @param [in] precision 小数秒の桁数
		 * @pre precisionは12以下
		 */
		Mjd(const std::string& scale,
			int64_t year, int64_t month, int64_t mday,
			int64_t hour, int64_t minute, int64_t second, int64_t decimal = 0,
			uint16_t precision = PRECISION);

		/** 文字列によるコンストラクタ.
		 * @param [in] scale 時刻系
		 * @param [in] calendar カレンダー文字列(YYYY/MM/DD hh:mm:ss.fff)
		 * @param [in] format strftime関数の出力フォーマット
		 */
		Mjd(const std::string& scale, const std::string& calendar,
			const std::string& format = "%Y/%m/%d %T");
		
		/** デストラクタ. */
		~Mjd();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		Mjd(const Mjd&);
		
		/** コピー代入演算子. */
		Mjd& operator=(const Mjd&);
		
		/** ムーブコンストラクタ. */
		Mjd(Mjd&&);
		
		/** ムーブ代入演算子. */
		Mjd& operator=(Mjd&&);
		
		//----------------------------------------------------------------------
		// 演算子
	public:
		/** 加算代入演算子.
		 * @param [in] r 演算子右辺
		 * @return 加算結果への参照
		 * @throw overflow_error 桁溢れ
		 */
		Mjd& operator+=(const Mjd& r);
		
		/** 加算代入演算子.
		 * @param [in] r 演算子右辺[sec]
		 * @return 加算結果への参照
		 * @throw overflow_error 桁溢れ
		 */
		Mjd& operator+=(double r);
		
		/** 減算代入演算子.
		 * @param [in] r 演算子右辺
		 * @return 減算結果への参照
		 * @throw overflow_error 桁溢れ
		 */
		Mjd& operator-=(const Mjd& r);
		
		/** 減算代入演算子.
		 * @param [in] r 演算子右辺[sec]
		 * @return 減算結果への参照
		 * @throw overflow_error 桁溢れ
		 */
		Mjd& operator-=(double r);
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** 日数の加算.
		 * @param d 加算日数
		 * @return 加算結果への参照
		 */
		Mjd& addDays(int64_t d);
		
		/** 時間数の加算.
		 * @param h 加算時間数
		 * @return 加算結果への参照
		 */
		Mjd& addHours(int64_t h);
		
		/** 分数の加算.
		 * @param m 加算分数
		 * @return 加算結果への参照
		 */
		Mjd& addMinutes(int64_t m);
		
		/** 秒数の加算.
		 * @param s 加算秒数
		 * @return 加算結果への参照
		 */
		Mjd& addSeconds(int64_t s);
		
		/** ミリ秒数の加算.
		 * @param ms 加算ミリ秒数
		 * @return 加算結果への参照
		 */
		Mjd& addmsec(int64_t ms);
		
		/** マイクロ秒数の加算.
		 * @param us 加算マイクロ秒数
		 * @return 加算結果への参照
		 */
		Mjd& addusec(int64_t us);
		
		/** ナノ秒数の加算.
		 * @param ns 加算ナノ秒数
		 * @return 加算結果への参照
		 */
		Mjd& addnsec(int64_t ns);
		
		/** ピコ秒数の加算.
		 * @param ps 加算ピコ秒数
		 * @return 加算結果への参照
		 */
		Mjd& addpsec(int64_t ps);
		
		/** 時刻系の設定.
		 * @param scale 時刻系
		 * @return 設定結果への参照
		 */
		Mjd& setScale(const std::string& scale);
		
	private:
		/** カレンダー日による設定.
		 * @param [in] year 年
		 * @param [in] month 月
		 * @param [in] mday 日
		 * @param [in] hour 時
		 * @param [in] minute 分
		 * @param [in] second 秒
		 * @param [in] decimal 小数秒
		 * @param [in] precision 小数秒の桁数
		 */
		Mjd& setYMDHMS
		(int64_t year, int64_t month, int64_t mday,
		 int64_t hour, int64_t minute, int64_t second, int64_t decimal,
		 uint16_t precision = PRECISION);

		/** 時刻構造体による設定.
		 * @param [in] t 時刻構造体
		 */
		Mjd& setTm(const std::tm& t);

		/** 繰り上げ.
		 * 経過秒数や小数秒を繰り上げ操作する。
		 */
		void carry();
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 時刻系の参照.
		 * @return 時刻系
		 */
		const std::string& scale() const;
		
		/** 修正ユリウス日の取得.
		 * @return 修正ユリウス日
		 */
		int64_t day() const;
		
		/** 0時からの経過時間の取得.
		 * @return 0時からの経過時間[psec]
		 */
		int64_t psec() const;
		
		/** 0時からの経過秒の取得.
		 * @return 0時からの経過秒
		 */
		double sec() const ;
		
		/** 切り捨て日数 */
		int64_t floorDay() const ;
		
		/** 切り上げ日数 */
		int64_t ceilDay() const;
		
		/** トータル秒数の取得
		 * @return トータル秒数
		 */
		double totalSec() const;
		
		/** トータル日数の取得
		 * @return トータル日数
		 */
		double totalDay() const;
		
		/** 経過日数
		 * @param [in] epoch 元期の修正ユリウス日
		 * @return 経過日数(Elapsed Julian Centuries)
		 */
		double elapsedDay(double epoch = J2000) const;
		
		/** 経過ユリウス世紀の取得
		 * @param [in] epoch 元期の修正ユリウス日
		 * @return 経過ユリウス世紀(Elapsed Julian Centuries)
		 */
		double elapsedJC(double epoch = J2000) const;
		
		/** 2実数の修正ユリウス日の取得.
		 * @return 修正ユリウス日整数部,小数部(0時からの経過日数)
		 */
		std::pair<double, double> mjd2d() const;
		
		/** time_t時刻の取得.
		 * @param [in] precision 小数秒桁数
		 * @return time_t時刻
		 * @return 小数秒
		 * @pre digitsは12以下であること
		 */
		std::pair<time_t, int64_t> tt(uint16_t precision = PRECISION) const;
		
		/** 時刻構造体の取得.
		 * @param [in] precision 小数秒桁数
		 * @return 時刻構造体
		 * @return 小数秒
		 * @pre precisionは12以下であること
		 */
		std::pair<std::tm, int64_t> tm(uint16_t precision = PRECISION) const;
		
		/** time_point時刻の取得.
		 * @return time_point時刻
		 * @pre 時系がDURATIONでないこと
		 */
		std::chrono::time_point
		<std::chrono::system_clock, std::chrono::nanoseconds> tp() const;
		
		/** duration時刻の取得.
		 * @return chrono::durationの取得
		 * @pre 時系がDURATIONであること
		 */
		std::chrono::nanoseconds duration() const;
		
		/** GPS週番号の取得.
		 * @param [in] roll ロールオーバーするフラグ
		 * @return GPS週番号
		 */
		int64_t wn(bool roll = false) const;
		
		/** GPS週秒の取得.
		 * @param [in] precision 小数秒桁数
		 * @return GPS週秒[sec]
		 */
		int64_t tow(uint16_t precision = PRECISION) const;
		
		/** カレンダー文字列(フォーマット指定)の取得.
		 * @param [in] precision 小数点以下表示桁数
		 * @param [in] format strftime関数の出力フォーマット
		 * @return カレンダー文字列
		 */
		std::string string(uint16_t precision = PRECISION,
						   const std::string& format = "%Y/%m/%d %T") const;
		
		/** デバッグ文字列の取得
		 * @return デバッグ文字列
		 */
		std::string debug() const;
		
		/** 真夜中の時刻の取得
		 * @return 真夜中の時刻
		 */
		Mjd midnight() const;

		/** 整数時へ切り捨てた結果の取得.
		 * @return 整数時へ切り捨てた結果
		 */
		Mjd intHour() const;

		/** 整数分へ切り捨てた結果の取得.
		 * @return 整数分へ切り捨てた結果
		 */
		Mjd intMinute() const;

		/** 整数秒へ切り捨てた結果の取得.
		 * @return 整数秒へ切り捨てた結果
		 */
		Mjd intSecond() const;

		/** ミリ秒へ切り捨てた結果の取得.
		 * @return ミリ秒へ切り捨てた結果
		 */
		Mjd intmsec() const;

		/** マイクロ秒へ切り捨てた結果の取得.
		 * @return ミリ秒へ切り捨てた結果
		 */
		Mjd intusec() const;

		/** ナノ秒へ切り捨てた結果の取得.
		 * @return ミリ秒へ切り捨てた結果
		 */
		Mjd intnsec() const;

		/** 比較対象と等しい.
		 * @param [in] target 比較対象時刻
		 * @retval true 等しい
		 * @retval false 等しくない
		 */
		bool equals(const Mjd& target) const;
		
		/** 比較対象より大きい.
		 * @param [in] target 比較対象
		 * @retval true  比較対象より大きい
		 * @retval false 比較対象より以下
		 */
		bool isGreaterThan(const Mjd& target) const;
		
		/** 比較対象より小さい.
		 * @param [in] target 比較対象
		 * @retval true  比較対象より小さい
		 * @retval false 比較対象より以上
		 */
		bool isLessThan(const Mjd& target) const;

		/** Local以外の適正な時系
		 * @retval true  Local以外の適正な時系
		 * @retval false Localの時系または不正な時系
		 */
		bool isGrobal() const;

		/** 時系がLocal
		 * @retval true  Local時系
		 * @retval false Localでない時系
		 */
		bool isLocal() const;

		/** 閏秒を含む時系
		 * @retval true  閏秒を含む時系
		 * @retval false 閏秒を含まない時系(DUR,TDB,TDT,TAI,GPS,UT1)
		 */
		bool withLeap() const;

	private:
		/** 一日の長さの取得
		 * @return １日の長さ[sec]
		 */
		double secday() const;
		
		/** 同じ時刻系であることを確認.
		 * @param [in] target 比較対象
		 * @throw domain_error 時刻系が異なる。
		 */
		void checkSameScale(const Mjd& target) const;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 現在時刻の取得
		 * @return 現在時刻
		 */
		static Mjd now();

		/** 時系文字列の確認
		 * @param [in] scale 時系文字列
		 * @param [in] throws 不適切な場合に例外を投げる
		 * @retval true 時系文字列として適切
		 * @retval false 時系文字列として不適切
		 */
		static bool validScale(const std::string& scale, bool throws);
		
		/** 年月日から修正ユリウス日への変換.
		 * @param [in] y 年
		 * @param [in] m 月
		 * @param [in] d 日
		 * @return 修正ユリウス日
		 */
		static int64_t mjd(int64_t y, int64_t m, int64_t d);
		
		/** 修正ユリウス日から年月日への変換.
		 * @param [in] mjd 修正ユリウス日
		 * @return 年,月,日
		 */
		static std::tuple<int16_t, int16_t, int16_t> ymd(int64_t mjd);
		
		/** 時分秒から0時からの経過秒への変換.
		 * @param [in] h 時
		 * @param [in] m 分
		 * @param [in] s 秒
		 * @param [in] decimal 小数秒
		 * @param [in] precision 小数秒の桁数
		 * @return 経過ピコ秒
		 */
		static int64_t psec
		(int64_t h, int64_t m, int64_t s,
		 int64_t decimal = 0, uint16_t precision = PRECISION);
		
		/** 経過時間から時分秒への変換.
		 * @param [in] decimal 経過時間
		 * @param [in] precision 経過時間の小数秒の桁数
		 * @return 日,時,分,秒,小数秒
		 */
		static std::tuple<int64_t, int16_t, int16_t, int16_t, int64_t>
		hms(int64_t decimal, uint16_t precision = PRECISION);

		/** 実数秒をピコ秒に変換
		 * @param [in] s 秒
		 * @param [in] round 丸めフラグ
		 * @param [in] precision 丸め桁数
		 * @return ピコ秒
		 * @throw std::domain_error 範囲外
		 */
		static int64_t psec
		(double s, bool round = true, uint16_t precision = PRECISION);

		/** 実数秒を日とピコ秒に変換
		 * @param [in] s 秒
		 * @param [in] round 丸めフラグ
		 * @param [in] precision 丸め桁数
		 * @return 日数
		 * @return ピコ秒
		 */
		static std::pair<int64_t, int64_t> daypsec
		(double s, bool round = true, uint16_t precision = PRECISION);

		/** time_tを日とピコ秒に変換
		 * @param [in] t 1970年1月1日0時0分0秒UTCからの経過秒(閏秒非考慮)
		 * @return 日数
		 * @return ピコ秒
		 */
		static std::pair<int64_t, int64_t> daypsec(time_t t);

		/** 日とピコ秒をtime_tと小数秒(ピコ秒)に変換
		 * @param [in] mjd 修正ユリウス日
		 * @param [in] psec ピコ秒
		 * @return 1970年1月1日0時0分0秒UTCからの経過秒
		 * @return 小数秒
		 * @remark 閏秒非考慮
		 */
		static std::pair<time_t, int64_t> tt(int64_t mjd, int64_t psec);

		/** 時分秒実数から時分秒小数秒への分割.
		 * @param [in] o 時分秒実数
		 * @param [in] round 丸めフラグ
		 * @param [in] precision 経過時間の小数秒の桁数
		 * @return 時,分,秒,小数秒
		 */
		static std::tuple<int16_t, int16_t, int16_t, int64_t>
		separate(double o, bool round = true, uint16_t precision = PRECISION);
		
		/** 整数表記小数秒を実数秒に変換
		 * @param [in] d 小数秒
		 * @param [in] precision 小数秒最小単位
		 * @return 実数秒
		 */
		static double sec(int64_t d, uint16_t precision = PRECISION);

		/** DUR時系文字列の参照
		 * @return DUR時系文字列
		 */
		static const std::string& DUR();

		/** TDB時系文字列の参照
		 * @return TDB時系文字列
		 */
		static const std::string& TDB();

		/** TDT時系文字列の参照
		 * @return TDT時系文字列
		 */
		static const std::string& TDT();

		/** TAI時系文字列の参照
		 * @return TAI時系文字列
		 */
		static const std::string& TAI();

		/** GPS時系文字列の参照
		 * @return GPS時系文字列
		 */
		static const std::string& GPS();

		/** UT1時系文字列の参照
		 * @return UT1時系文字列
		 */
		static const std::string& UT1();

		/** UTC時系文字列の参照
		 * @return UTC時系文字列
		 */
		static const std::string& UTC();

		/** LOCAL時系文字列の参照
		 * @return LOCAL時系文字列
		 */
		static const std::string& LOCAL();

		/** JST時系文字列の参照
		 * @return JST時系文字列
		 */
		static const std::string& JST();

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 時刻系 */
		std::string scale_;
		
		/** 修正ユリウス日 */
		int64_t day_;
		
		/** 0時からの経過時間[psec] */
		int64_t psec_;
		
	public:
		/** 小数桁数 */
		static const uint16_t PRECISION = 12;

		/** 時間の単位変換係数 hours / day.  */
		static const int64_t HOURS_PER_DAY = 24;

		/** 時間の単位変換係数 minutes / day.  */
		static const int64_t MINUTES_PER_HOUR = 60;

		/** 時間の単位変換係数 seconds / min.  */
		static const int64_t SECONDS_PER_MINUTE = 60;

		/** 時間の単位変換係数 minutes / day.  */
		static const int64_t MINUTES_PER_DAY = 1440;

		/** 時間の単位変換係数 seconds / hour.  */
		static const int64_t SECONDS_PER_HOUR = 3600;

		/** 時間の単位変換係数 seconds / day.  */
		static const int64_t SECONDS_PER_DAY = 86400;

		/** 週の単位変換係数 days / week.  */
		static const int64_t DAYS_PER_WEEK = 7;

		/** time_t元期(1970/1/1)の修正ユリウス日 */
		static const int64_t MJD_TIMET_EPOCH = 40587;

		/** GPS時刻元期(1980/1/6)の修正ユリウス日 */
		static const int64_t MJD_GPS_EPOCH = 44244;

		/** GPS週番号の最大値 */
		static const int64_t MAX_GPS_WN = 1024;

		/** ユリウス日と修正ユリウス日の差(JD-MJD)[day] */
		static constexpr double JD_MJD = 2400000.5;

		/** 2000年1月1日12時の修正ユリウス日 */
		static constexpr double J2000 = 51544.5;

		/** ユリウス世紀の日数 */
		static constexpr double DAYS_PER_JULIAN_CENTURY = 36525.;
	};
}

//------------------------------------------------------------------------------
// グローバル関数

/** 加算演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側
 * @return 加算結果
 * @throw domain_error 桁溢れ
 */
ib2_mss::Mjd operator+(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 加算演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側[sec]
 * @return 加算結果
 * @throw domain_error 桁溢れ
 */
ib2_mss::Mjd operator+(const ib2_mss::Mjd& left, double right);

/** 減算演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側
 * @return 減算結果
 * @throw domain_error 桁溢れ
 */
ib2_mss::Mjd operator-(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 減算演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側[sec]
 * @return 減算結果
 * @throw domain_error 桁溢れ
 */
ib2_mss::Mjd operator-(const ib2_mss::Mjd& left, double right);

/** 等号比較演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側
 * @retval true  等しい
 * @retval false 等しくない
 */
bool operator==(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 不等比較演算子.
 * @param [in] left  演算子左側
 * @param [in] right 演算子右側
 * @retval true  等しくない
 * @retval false 等しい
 */
bool operator!=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 大なり不等号比較演算子.
 * @param [in] left 演算子の左側
 * @param [in] right 演算子の右側
 * @retval true  比較対象より大きい
 * @retval false 比較対象より以下
 */
bool operator>(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 小なり不等号比較演算子.
 * @param [in] left 演算子の左側
 * @param [in] right 演算子の右側
 * @retval true  比較対象より小さい
 * @retval false 比較対象より以上
 */
bool operator<(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 以上不等号比較演算子.
 * @param [in] left 演算子の左側
 * @param [in] right 演算子の右側
 * @retval true  比較対象より以上
 * @retval false 比較対象よりより小さい
 */
bool operator>=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 以下不等号比較演算子.
 * @param [in] left 演算子の左側
 * @param [in] right 演算子の右側
 * @retval true  比較対象より以下
 * @retval false 比較対象よりより大きい
 */
bool operator<=(const ib2_mss::Mjd& left, const ib2_mss::Mjd& right);

/** 出力演算子
 * @param [in] os 演算子の左側
 * @param [in] t 演算子の右側
 * @return 出力ストリーム
 */
std::ostream& operator<<(std::ostream& os, const ib2_mss::Mjd& t);


// End Of File -----------------------------------------------------------------
