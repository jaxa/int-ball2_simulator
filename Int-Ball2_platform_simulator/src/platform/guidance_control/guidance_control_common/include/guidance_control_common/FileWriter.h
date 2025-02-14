
#pragma once

#include <vector>
#include <string>

namespace ib2_mss
{
	/**
	 * @brief テキストファイルの書き込み.
	 *
	 * ファイルへ出力する文字列を保持し、テキストファイルへ書き込む。
	 * また、出力ディレクトリを作成する。
	 */
	class FileWriter final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** コンストラクタ.
		 * @param [in] filename 出力ファイル名(ディレクトリを含む)
		 * @param [in] append 追記フラグ
		 */
		FileWriter(const std::string& filename = "", bool append = false);
		
		/** デストラクタ. */
		~FileWriter();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		FileWriter(const FileWriter&) = delete;
		
		/** コピー代入演算子. */
		FileWriter& operator=(const FileWriter&) = delete;
		
		/** ムーブコンストラクタ. */
		FileWriter(FileWriter&&) = delete;
		
		/** ムーブ代入演算子. */
		FileWriter& operator=(FileWriter&&) = delete;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 一行文字列の出力.
		 * 出力する文字列を保持しない
		 * @param [in] line 一行文字列
		 */
		void write(const std::string& line) const;
		
		/** ディレクトリを作成する.
		 * 最後が区切り文字でない場合は区切り文字を追加する
		 * @param [in,out] dirname ディレクトリ名
		 */
		static void makedir(std::string& dirname);
		
		/** ディレクトリの中のファイルを削除する.
		 * @param [in] dirname ディレクトリ名
		 */
		static void deleteFiles(const std::string& dirname);

		/** 指定ディレクトリに存在するファイル名リストの取得
		 * @param [in] dirname ディレクトリ名
		 * @return ファイル名リスト
		 */
		static std::vector<std::string> listFiles(const std::string& dirname);

		/** ファイル名をディレクトリ名とファイル名と拡張子に分割する.
		 * @param [in] filename ファイル名
		 * @param [in] extention 拡張子分割フラグ
		 * @return 分割結果(ディレクトリ名,ファイル名,拡張子)
		 */
		static std::vector<std::string> separate
		(const std::string& filename, bool extention = false);
		
		/** ファイル名の検査
		 * @param [in] filename 検査対象ファイル名
		 * @param [in] fullpath フルパスフラグ(true:ディレクトリ含む、false:含まない)
		 * @retval true 不正なファイル名
		 * @retval false 正当なファイル名
		 */
		static bool invalidname(const std::string& filename, bool fullpath);
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** ファイル名 */
		std::string filename_;
	};
}
// End Of File -----------------------------------------------------------------
