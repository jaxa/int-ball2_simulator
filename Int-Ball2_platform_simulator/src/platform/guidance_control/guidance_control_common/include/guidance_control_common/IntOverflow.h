
#pragma once

#include <string>

#define ADDABLE(l, r, throws) \
ib2_mss::IntOverflow::addable(l, r, throws, __FILE__, __FUNCTION__, __LINE__)
#define SUBTRACTABLE(l, r, throws) \
ib2_mss::IntOverflow::subtractable(l, r, throws, __FILE__, __FUNCTION__, __LINE__)
#define MULTIPLIABLE(l, r, throws) \
ib2_mss::IntOverflow::multipliable(l, r, throws, __FILE__, __FUNCTION__, __LINE__)
#define DIVIDABLE(l, r, throws) \
ib2_mss::IntOverflow::dividable(l, r, throws, __FILE__, __FUNCTION__, __LINE__)

namespace ib2_mss
{
	/**
	 * @brief 整数オーバーフローチェッカー.
	 */
	class IntOverflow
	{
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 加算可能判定.
		 * @param [in] l 記号の左側
		 * @param [in] r 記号の右側
		 * @param [in] throws 例外送出フラグ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 * @retval true 加算できる
		 * @retval false 加算できない
		 */
		template <typename T>
		static bool addable
		(T l, T r, bool throws, const std::string& file,
		 const std::string& function, unsigned long lineno);

		/** 減算可能判定.
		 * @param [in] l 記号の左側
		 * @param [in] r 記号の右側
		 * @param [in] throws 例外送出フラグ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 * @retval true 減算できる
		 * @retval false 減算できない
		 */
		template <typename T>
		static bool subtractable
		(T l, T r, bool throws, const std::string& file,
		 const std::string& function, unsigned long lineno);

		/** 乗算可能判定.
		 * @param [in] l 記号の左側
		 * @param [in] r 記号の右側
		 * @param [in] throws 例外送出フラグ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 * @retval true 乗算できる
		 * @retval false 乗算できない
		 */
		template <typename T>
		static bool multipliable
		(T l, T r, bool throws, const std::string& file,
		 const std::string& function, unsigned long lineno);

		/** 除算可能判定.
		 * @param [in] l 記号の左側
		 * @param [in] r 記号の右側
		 * @param [in] throws 例外送出フラグ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 * @retval true 除算できる
		 * @retval false 除算できない
		 */
		template <typename T>
		static bool dividable
		(T l, T r, bool throws, const std::string& file,
		 const std::string& function, unsigned long lineno);
	};
}
// End Of File -----------------------------------------------------------------
