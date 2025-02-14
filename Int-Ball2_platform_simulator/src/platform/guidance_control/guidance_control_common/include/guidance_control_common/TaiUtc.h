
#pragma once

#include <string>

namespace ib2_mss
{
	/**
	 * @brief TAI-UTCデータ.
	 *
	 * 国際原子時TAIと協定世界時UTCの差を管理する。
	 * 読み込みファイルは下記URLより取得する。
	 * http://maia.usno.navy.mil/ser7/tai-utc.dat
	 */
	class TaiUtc
	{
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** TAI-UTCデータの構成.
		 * TAI-UTCデータファイルを読み込む。
		 * @param [in] filename TAI-UTCデータファイルパス
		 */
		static void configure(const std::string& filename = "");

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** UTCMJDが入力されたときのTAI-UTCの取得.
		 * @param [in] utcmjd 修正ユリウス日(UTC)
		 * @param [in] secday 0時からの経過秒(UTC)
		 * @return TAI-UTC [sec]
		 */
		static double dAT(long long utcmjd, double secday = 0.);

		/** 閏秒の取得.
		 * @param [in] utcmjd 修正ユリウス日(UTC)
		 * @return 当日の最後に挿入される閏秒 [sec]
		 */
		static double leapsec(long long utcmjd);

		/** 閏秒の取得.
		 * @param [in]  from 開始修正ユリウス日(UTC)
		 * @param [in]  to   終了修正ユリウス日(UTC)
		 * @return 閏秒 [sec]
		 */
		static double leapsec(long long from, long long to);
		
		/** 閏秒の存在確認
		 * @param [in] utcmjd 修正ユリウス日(UTC)
		 * @retval true 閏秒の存在期間内
		 * @retval false 閏秒が存在期間外
		 */
		static bool step(long long utcmjd);

		/** 直前の閏秒挿入日の取得
		 * @param [in] utcmjd 修正ユリウス日(UTC)
		 * @return 直前の閏秒挿入日の修正ユリウス日(UTC)
		 */
		static long long before(long long utcmjd);

		/** 直後の閏秒挿入日の取得
		 * @param [in] utcmjd 修正ユリウス日(UTC)
		 * @return 直後の閏秒挿入日の修正ユリウス日(UTC)
		 */
		static long long after(long long utcmjd);

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 実装クラス */
		class Implement;
	};
}
// End Of File -----------------------------------------------------------------
