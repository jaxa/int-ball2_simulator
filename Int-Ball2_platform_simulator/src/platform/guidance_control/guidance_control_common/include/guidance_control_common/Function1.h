
#pragma once

namespace ib2_mss
{
	/**
	 * @brief 一変数関数.
	 *
	 * 独立変数が一つだけの関数。
	 * 関数 y = f (x)の計算式と導関数dy/dxを定義するインタフェース(仮想クラス)。
	 */
	class Function1
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ. */
		Function1() = default;
		
		/** デストラクタ. */
		virtual ~Function1() = default;
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Function1(const Function1&) = delete;
		
		/** コピー代入演算子. */
		Function1& operator=(const Function1&) = delete;
		
		/** ムーブコンストラクタ. */
		Function1(Function1&&) = delete;
		
		/** ムーブ代入演算子. */
		Function1& operator=(Function1&&) = delete;
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 関数の評価 y = f(x).
		 * @param [in] x 入力値
		 * @return 出力値
		 */
		virtual double operator()(double x) const = 0;
		
		/** 導関数の評価 dy/dx = f'(x)
		 * @param [in] x 独立変数(定義値)
		 * @return 導関数評価結果
		 */
		virtual double d(double x) const = 0;
	};
}
// End Of File -----------------------------------------------------------------
