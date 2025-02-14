
#pragma once

#include <vector>
#include <string>

namespace ib2_mss
{
	/**
	 * @brief テキストファイルの読み込み.
	 *
	 * テキストファイルを読み込み、各行の文字列を保持し、必要な値を出力する。
	 */
	class FileReader final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** コンストラクタ.
		 * @param [in] filename 読み込みファイル名
		 * @param [in] comment コメント開始識別文字
		 * @param [in] blank 空行読み込みフラグ
		 */
		FileReader(const std::string& filename = "",
				   const std::string& comment = "#", bool blank = false);
		
		/** デストラクタ. */
		~FileReader();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		FileReader(const FileReader&) = delete;
		
		/** コピー代入演算子. */
		FileReader& operator=(const FileReader&) = delete;
		
		/** ムーブコンストラクタ. */
		FileReader(FileReader&&) = delete;
		
		/** ムーブ代入演算子. */
		FileReader& operator=(FileReader&&) = delete;
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** ファイル名の取得.
		 * @return ファイル名
		 */
		const std::string& filename() const;
		
		/** ブロック開始行番号の参照.
		 * @return ブロック開始行番号
		 */
		const std::vector<size_t>& block() const;
		
		/** 取得行数の取得.
		 * @return 取得行数
		 */
		size_t lines() const;
		
		/** 指定ブロックの取得行数の取得.
		 * @param [in] blockno ブロック番号
		 * @return 取得行数
		 */
		size_t lines(size_t blockno) const;
		
		/** 全行文字列の参照.
		 * @return 文字列(trimしない)
		 */
		const std::vector<std::string>& line() const;
		
		/** 全行文字列の取得.
		 * @return 文字列(trimする)
		 */
		std::vector<std::string> string() const;

		/** 文字列の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] comment コメント識別文字
		 * @return 文字列
		 */
		std::string string(size_t lineno, const std::string& comment ="") const;
		
		/** 倍精度浮動小数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return 倍精度浮動小数値
		 */
		double value(size_t lineno, const std::string& name = "",
					 bool empty0 = true, bool strict = true) const;
		
		/** 浮動小数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return 浮動小数値
		 */
		float valueFloat(size_t lineno, const std::string& name = "",
						 bool empty0 = true, bool strict = true) const;
		
		/** short型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int型数値
		 */
		short valueShort
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** int型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int型数値
		 */
		int valueInt
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** long型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return long型数値
		 */
		long valueLong
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** long long型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return long long型数値
		 */
		long long valueLL
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** unsigned short型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned short型数値
		 */
		unsigned short valueUS
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** unsigned int型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned int型数値
		 */
		unsigned int valueUI
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** unsigned long型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned long型数値
		 */
		unsigned long valueUL
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** unsigned long long型数値の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned long型数値
		 */
		unsigned long long valueULL
		(size_t lineno, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true) const;
		
		/** 文字列vector配列の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] delimiter 区切り文字
		 * @param [in] detectAll 全区切り文字の検出フラグ(true:検出,false:無視)
		 * @return 文字列のvector配列
		 */
		std::vector<std::string> vectorString
		(size_t lineno, const std::string& delimiter = "\t,",
		 bool detectAll = false) const;
		
		/** 実数vector配列の取得.
		 * @param [in] lineno 行番号(ブロック開始行を0行とする)
		 * @param [in] delimiter 区切り文字
		 * @param [in] detectAll 全区切り文字の検出フラグ(true:検出,false:無視)
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return 文字列のvector配列
		 */
		std::vector<double> vector
		(size_t lineno, const std::string& delimiter = "\t,",
		 bool detectAll = false, bool empty0 = true, bool strict = true) const;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 不要文字列の除去.
		 * 文字列前後の空白・タブを除去する。
		 * @param [in] field 処理前文字列
		 * @return 文字列整形結果
		 */
		static std::string trim(const std::string& field);
		
		/** 不要文字列の除去.
		 * 文字列から指定の文字を除去する。
		 * @param [in] field 処理前文字列
		 * @param [in] object 削除対象文字列
		 * @return 文字列整形結果
		 */
		static std::string omit
		(const std::string& field, const std::string& object = " \t\r\n\ufeff");
		
		/** 特定文字列の抽出.
		 * 文字列から指定の文字だけを抽出する。
		 * @param [in] field 処理前文字列
		 * @param [in] object 抽出対象文字列
		 * @return 文字列整形結果
		 */
		static std::string extract
		(const std::string& field, const std::string& object);
		
		/** 文字列からdouble値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return double値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static double value
		(const std::string& field, const std::string& name = "",
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からfloat値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return float値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static float valueFloat
		(const std::string& field, const std::string& name = "",
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からshort値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static short valueShort
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からint値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static int valueInt
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からlong値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return long値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static long valueLong
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からlong long値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return double値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static long long valueLL
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からunsigned short値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned short値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static unsigned short valueUS
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からunsigned int値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned int値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static unsigned int valueUI
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からunsigned long値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned long値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static unsigned long valueUL
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列からunsigned long値への変換.
		 * @param [in] field 文字列フィールド
		 * @param [in] name 変数名
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return unsigned long値への変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static unsigned long long valueULL
		(const std::string& field, const std::string& name = "", int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列から文字列vector配列の取得.
		 * @param [in] field 文字列フィールド
		 * @param [in] delimiter 区切り文字
		 * @param [in] detectAll 全区切り文字の検出フラグ(true:検出,false:無視)
		 * @return 文字列のvector配列
		 */
		static std::vector<std::string> vectorString
		(const std::string& field, const std::string& delimiter = "\t,",
		 bool detectAll = false) ;
		
		/** 文字列ベクタからdouble値ベクタへの変換.
		 * @param [in] fields 文字列フィールド
		 * @param [in] is 開始インデックス
		 * @param [in] ie 終了インデックス(ie <= isのとき、最後までベクタへ変換)
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return double値ベクタへの変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static std::vector<double> vector
		(const std::vector<std::string>& fields, size_t is = 0, size_t ie = 0,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列ベクタからint値ベクタへの変換.
		 * @param [in] fields 文字列フィールド
		 * @param [in] is 開始インデックス
		 * @param [in] ie 終了インデックス(ie <= isのとき、最後までベクタへ変換)
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int値ベクタへの変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static std::vector<int> vectorInt
		(const std::vector<std::string>& fields,
		 size_t is = 0, size_t ie = 0, int base = 10,
		 bool empty0 = true, bool strict = true);
		
		/** 文字列ベクタからunsigned long値ベクタへの変換.
		 * @param [in] fields 文字列フィールド
		 * @param [in] is 開始インデックス
		 * @param [in] ie 終了インデックス(ie <= isのとき、最後までベクタへ変換)
		 * @param [in] base 基数
		 * @param [in] empty0 空文字列の時に0を返す
		 * @param [in] strict 途中に文字があると例外を投げる
		 * @return int値ベクタへの変換結果
		 * @throw invalid_argument 値の取得失敗
		 */
		static std::vector<unsigned long> vectorUL
		(const std::vector<std::string>& fields,
		 size_t is = 0, size_t ie = 0, int base = 10,
		 bool empty0 = true, bool strict = true);
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** ファイル名 */
		std::string filename_;
		
		/** ブロック開始行番号 */
		std::vector<size_t> block_;
		
		/** テキストファイルの文字列. */
		std::vector<std::string> line_;
	};
}
// End Of File -----------------------------------------------------------------
