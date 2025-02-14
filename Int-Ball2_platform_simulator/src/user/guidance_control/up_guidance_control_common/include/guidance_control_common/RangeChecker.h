
#pragma once

#include <limits>
#include <string>
#include <vector>

namespace ib2_mss
{
	// 前方宣言
	class Mjd;
	
	/**
	 * @brief 数値が範囲内に入っていることを確認する
	 */
	template <typename T>
	class RangeChecker final
	{
		friend class RangeCheckerTest;

		//----------------------------------------------------------------------
		// 列挙子
	public:
		/** 範囲種別 */
		enum class TYPE : unsigned short
		{
			LT,		///< 判定値(min)より小さい
			LE,		///< 判定値(min)以下
			GT,		///< 判定値(min)より大きい
			GE,		///< 判定値(min)以上
			GT_LT,	///< 最小値より大きく、かつ、最大値より小さい
			GT_LE,	///< 最小値より大きく、かつ、最大値以下
			GE_LT,	///< 最小値以上、かつ、最大値より小さい
			GE_LE,	///< 最小値以上、かつ、最大値より小さい
			LT_GT,	///< 最小値より小さい、または、最大値より大きい
			LT_GE,	///< 最小値より小さい、または、最大値以上
			LE_GT,	///< 最小値以上、または、最大値より小さい
			LE_GE,	///< 最小値以下、または、最大値より小さい
		};

		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ. */
		RangeChecker();

		/** 判定値１つのコンストラクタ.
		 * @param [in] type 範囲種別
		 * @param [in] thr 判定値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 */
		RangeChecker(const TYPE& type, T thr, bool throws = false);

		/** コンストラクタ.
		 * @param [in] type 範囲種別
		 * @param [in] min 判定値/最小値
		 * @param [in] max 最大値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 */
		RangeChecker(const TYPE& type, T min, T max, bool throws = false);

		/** デストラクタ. */
		~RangeChecker();

		//----------------------------------------------------------------------
		// コピー
	private:
		/** コピーコンストラクタ. */
		RangeChecker(const RangeChecker&) = delete;

		/** コピー代入演算子. */
		RangeChecker& operator=(const RangeChecker&) = delete;

		/** ムーブコンストラクタ. */
		RangeChecker(RangeChecker&&) = delete;

		/** ムーブ代入演算子. */
		RangeChecker& operator=(RangeChecker&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** 範囲内の確認.
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が範囲内
		 * @retval false 検査対象値が範囲外
		 */
		bool valid(T x, const std::string& name = "x") const;

		/** 最小値より小さい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より小さい
		 * @retval false 検査対象値が最小値以上
		 */
		bool lt(T x, const std::string& name = "x") const;

		/** 最小値以下の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以下
		 * @retval false 検査対象値が最小値より大きい
		 */
		bool le(T x, const std::string& name = "x") const;

		/** 最小値より大きい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より大きい
		 * @retval false 検査対象値が最小値以下
		 */
		bool gt(T x, const std::string& name = "x") const;

		/** 最小値以上の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以上
		 * @retval false 検査対象値が最小値より小さい
		 */
		bool ge(T x, const std::string& name = "x") const;

		/** 最小値より大きく最大値より小さい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より大きく最大値より小さい
		 * @retval false 検査対象値が最小値以下または最大値以上
		 */
		bool gtlt(T x, const std::string& name = "x") const;

		/** 最小値より大きく最大値以下の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より大きく最大値以下
		 * @retval false 検査対象値が最小値以下または最大値より大きい
		 */
		bool gtle(T x, const std::string& name = "x") const;

		/** 最小値以上で最大値より小さい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以上で最大値より小さい
		 * @retval false 検査対象値が最小値より小さいまたは最大値以上
		 */
		bool gelt(T x, const std::string& name = "x") const;

		/** 最小値以上で最大値以下の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以上で最大値以下
		 * @retval false 検査対象値が最小値より小さいまたは最大値より大きい
		 */
		bool gele(T x, const std::string& name = "x") const;

		/** 最小値より小さいまたは最大値より大きい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より小さいまたは最大値より大きい
		 * @retval false 検査対象値が最小値以上で最大値以下
		 */
		bool ltgt(T x, const std::string& name = "x") const;

		/** 最小値より小さいまたは最大値以上の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値より小さいまたは最大値以上
		 * @retval false 検査対象値が最小値以上で最大値より小さい
		 */
		bool ltge(T x, const std::string& name = "x") const;

		/** 最小値以下または最大値より大きい判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以下または最大値より大きい
		 * @retval false 検査対象値が最小値より大きく最大値以下
		 */
		bool legt(T x, const std::string& name = "x") const;

		/** 最小値以下または最大値以上の判定
		 * @param [in] x 検査対象値
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が最小値以下または最大値以上
		 * @retval false 検査対象値が最小値より大きく最大値より小さい
		 */
		bool lege(T x, const std::string& name = "x") const;

		//----------------------------------------------------------------------
		// 実装(クラス関数)
	public:
		/** 正値判定
		 * @param [in] x 検査対象値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が正
		 * @retval false 検査対象値が正でない
		 */
		static bool positive(T x, bool throws = false,
							 const std::string& name = "x");
		
		/** 負値判定
		 * @param [in] x 検査対象値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が負
		 * @retval false 検査対象値が負でない
		 */
		static bool negative(T x, bool throws = false,
							 const std::string& name = "x");
		
		/** 非負判定
		 * @param [in] x 検査対象値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が0以下
		 * @retval false 検査対象値が正
		 */
		static bool notPositive(T x, bool throws = false,
								const std::string& name = "x");
		
		/** 非正判定
		 * @param [in] x 検査対象値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 * @param [in] name 検査対象値の名前
		 * @retval true 検査対象値が0以上
		 * @retval false 検査対象値が負
		 */
		static bool notNegative(T x, bool throws = false,
								const std::string& name = "x");

		/** 同値判定
		 * @param [in] l 左辺値
		 * @param [in] r 右辺値
		 * @param [in] throws 範囲外の時に例外を投げるフラグ
		 * @param [in] namel 左辺値の名前
		 * @param [in] namer 右辺値の名前
		 * @retval true 検査対象値が等しい
		 * @retval false 等しくない
		 */
		static bool equal
		(T l, T r, bool throws = false,
		 const std::string& namel = "left", const std::string& namer = "right");
		
		/** 単調増加/単調減少判定
		 * @param [in] type 範囲種別（配列番号の若い順に並べた時の不等号）
		 * @param [in] x 検査対象値
		 * @param [in] throws 単調増加/単調減少でない時に例外を投げるフラグ
		 * @param [in] name 検査対象値の名前
		 * @retval true 単調増加/単調減少
		 * @retval false 単調増加/単調減少でない
		 */
		static bool monotonic
		(const TYPE& type, const std::vector<T>& x, bool throws = false,
		 const std::string& name = "x");
		
		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** 判定種別 */
		TYPE type_;

		/** 最小値・判定値 */
		T min_;

		/** 最大値 */
		T max_;

		/** 例外を投げるフラグ */
		bool throws_;
	};
	
	//----------------------------------------------------------------------
	// テンプレートの特殊化
	typedef RangeChecker<int8_t>  RangeCheckerI08;
	typedef RangeChecker<int16_t> RangeCheckerI16;
	typedef RangeChecker<int32_t> RangeCheckerI32;
	typedef RangeChecker<int64_t> RangeCheckerI64;

	typedef RangeChecker<uint8_t>  RangeCheckerUI08;
	typedef RangeChecker<uint16_t> RangeCheckerUI16;
	typedef RangeChecker<uint32_t> RangeCheckerUI32;
	typedef RangeChecker<uint64_t> RangeCheckerUI64;

	typedef RangeChecker<float>  RangeCheckerF;
	typedef RangeChecker<double> RangeCheckerD;
	typedef RangeChecker<long double> RangeCheckerLD;
	typedef RangeChecker<ib2_mss::Mjd> RangeCheckerT;
}

// End Of File -----------------------------------------------------------------
