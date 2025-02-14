
#pragma once

#include <string>
#include <vector>
#include <sstream>

#define LOG_DEBUG(str) ib2_mss::Log::debug(str, __FILE__, __FUNCTION__, __LINE__)
#define LOG_INFO(str) ib2_mss::Log::info(str, __FILE__, __FUNCTION__, __LINE__)
#define LOG_WARN(str) ib2_mss::Log::warn(str, __FILE__, __FUNCTION__, __LINE__)
#define LOG_ERROR(str) ib2_mss::Log::error(str, __FILE__, __FUNCTION__, __LINE__)

namespace ib2_mss
{
	/**
	 * @brief ログ操作.
	 *
	 * プログラムのログを出力する。
	 * ただし、メンバー関数instanceの静的変数を参照するため、インスタンスの削除は不要。
	 * スレッドセーフではないので、シングルスレッド状態においてconfigureすること。
	 * - Log::configure(ログファイル名、ログレベル)でログ出力条件を設定する。
	 * - Log::debug → デバッグログの出力
	 * - Log::info  → インフォメーションログの出力
	 * - Log::warn  → 警告ログの出力
	 * - Log::error → エラーログの出力
	 */
	class Log
	{
		//----------------------------------------------------------------------
		// 列挙子/構造体
	public:
		/** ログレベルの列挙子 */
		enum class LEVEL : size_t
		{
			DEBUG_LOG,	///< デバッグログ
			INFO_LOG,	///< 情報ログ
			WARN_LOG,	///< 警告ログ
			ERROR_LOG	///< エラーログ
		};
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** ログの構成.
		 * ログ初期化ファイルを読み込み、ログ出力条件を構築する。
		 * @param [in] logfile ログファイル名
		 * @param [in] loglevel ログレベル
		 * @param [in] maxlines 最大書き込み行数
		 * @retval true ログ初期化成功
		 * @retval false ログ初期化失敗
		 */
		static bool configure
		(const std::string& logfile, const std::string& loglevel,
		 size_t maxlines = DEFAULT_MAX_LINES);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** ログファイル名の参照
		 * @return ログファイル名
		 */
		static const std::string& logfile();
		
		/** デバッグ出力判定.
		 * @retval true  デバッグログを出力する
		 * @retval false デバッグログを出力しない
		 */
		static bool isDebug();
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** デバッグログ出力.
		 * @param [in] message	ログメッセージ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 */
		static void debug
		(const std::string& message, const std::string& file,
		 const std::string& function, unsigned long lineno);
		
		/** 情報ログ出力.
		 * @param [in] message	ログメッセージ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 */
		static void info
		(const std::string& message, const std::string& file,
		 const std::string& function, unsigned long lineno);
		
		/** 警告ログ出力.
		 * @param [in] message	ログメッセージ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 */
		static void warn
		(const std::string& message, const std::string& file,
		 const std::string& function, unsigned long lineno);
		
		/** エラーログ出力.
		 * @param [in] message	ログメッセージ
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 */
		static void error
		(const std::string& message, const std::string& file,
		 const std::string& function, unsigned long lineno);
		
		/** ファイル読み込みエラーメッセージ
		 * @param [in] what 例外のメッセージ
		 * @param [in] comment コメント
		 * @return エラーメッセージ
		 */
		static std::string caughtException
		(const std::string& what, const std::string& comment = "");
		
		/** ファイル読み込みエラーメッセージ
		 * @param [in] what 例外のメッセージ
		 * @param [in] comment コメント
		 * @param [in] filename 読み込みファイル名
		 * @param [in] lineno 読み込み行番号
		 * @return エラーメッセージ
		 */
		static std::string errorFileLine
		(const std::string& what, const std::string& comment,
		 const std::string& filename, size_t lineno);
		
		/** 配列読み込みエラーメッセージ
		 * @param [in] what 例外のメッセージ
		 * @param [in] comment コメント
		 * @param [in] name 配列名
		 * @param [in] index 配列番号
		 * @param [in] size 配列サイズ
		 * @return エラーメッセージ
		 */
		static std::string errorArrayIndex
		(const std::string& what, const std::string& comment,
		 const std::string& name, size_t index, size_t size);
		
		/** デバッグ文字列の作成
		 * @param [in] info 変数名、値
		 * @return デバッグ文字列
		 */
		template <typename T>
		static std::string stringDebug
		(const std::vector<std::pair<std::string, T>>& info)
		{
			std::ostringstream ss;
			ss.precision(16);
			for (auto& x: info)
				ss << x.first << ":" << x.second << ",";
			return ss.str();
		}
		
		/** 配列デバッグ文字列の作成
		 * @param [in] array 配列
		 * @param [in] name 配列名
		 * @return デバッグ文字列
		 */
		template <class Range>
		static std::string stringDebug
		(const Range& array, const std::string& name)
		{
			size_t count(0);
			std::ostringstream ss;
			ss.precision(16);
			for (auto& x: array)
				ss << name << "[" << count++ << "]=" << x << ",";
			return ss.str();
		}
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 実装クラス */
		class Implement;
		
	public:
		/** 最大書き込み行数デフォルト値 */
		static const size_t DEFAULT_MAX_LINES = 100000;
		
		/** 例外キャッチメッセージ */
		static const std::string CAUGHT_EXCEPTION;
	};
}
// End Of File ----------------------------------------------------------------
