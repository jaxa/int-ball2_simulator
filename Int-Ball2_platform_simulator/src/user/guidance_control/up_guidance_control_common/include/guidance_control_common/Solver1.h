
#pragma once

#include "Function1.h"

#include <limits>
#include <memory>

namespace ib2_mss
{
	/**
	 * @brief 一次元関数の解.
	 *
	 * 一次元関数 f(x) = 0 の解xを計算する。
	 */
	class Solver1 final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** コンストラクタ.
		 * @param [in] xtol 独立変数収束判定許容値
		 * @param [in] ytol 従属変数収束判定許容値
		 * @param [in] itmax 最大反復計算回数
		 */
		Solver1(double xtol = 1.e-15, double ytol = 1.e-15, size_t itmax = 100);
		
		/** デストラクタ. */
		~Solver1();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Solver1(const Solver1&) = delete;
		
		/** コピー代入演算子. */
		Solver1& operator=(const Solver1&) = delete;
		
		/** ムーブコンストラクタ. */
		Solver1(Solver1&&) = delete;
		
		/** ムーブ代入演算子. */
		Solver1& operator=(Solver1&&) = delete;

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 独立変数収束判定許容値の取得.
		 * @return 独立変数収束判定許容値
		 */
		double xtol() const;

		/** 従属変数収束判定許容値の取得.
		 * @return 従属変数収束判定許容値
		 */
		double ytol() const;
		
		/** 最大反復計算回数の取得.
		 * @return 最大反復計算回数
		 */
		size_t itmax() const;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** Van Wijngaarden-Dekker-Brent法.
		 * brent法により一次元方程式の解を計算する。
		 * ref: numerical receipes in C pp.262-263
		 * @param [in] f 一次元関数
		 * @param [in] x1 解を挟む独立変数1
		 * @param [in] x2 解を挟む独立変数2
		 * @return 解
		 */
		double brent(const Function1& f, double x1, double x2) const;
		
		/** Newton-Raphson法.
		 * newton法により一次元方程式の解を計算する。
		 * @param [in] f 一次元関数
		 * @param [in] x0 初期解
		 * @return 解
		 */
		double newton(const Function1& f, double x0) const;
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 独立変数収束判定許容値. */
		double xtol_;
		
		/** 従属変数収束判定許容値. */
		double ytol_;
		
		/** 最大反復計算回数. */
		size_t itmax_;
	};
}
// End Of File -----------------------------------------------------------------
