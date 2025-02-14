
#pragma once

#include <cstdlib>
#include <deque>

namespace ib2_mss
{
	/**
	 * @brief 移動平均.
	 */
	class MovingAverage
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** コンストラクタ.
		 * @param [in] nmax データ配列の最大サイズ(0の時size_t maxと同じ)
		 */
		MovingAverage(size_t nmax = 0);

		/** デストラクタ. */
		~MovingAverage();

		//----------------------------------------------------------------------
		// コピー
	public:
		/** コピーコンストラクタ. */
		MovingAverage(const MovingAverage&) = delete;

		/** コピー代入演算子. */
		MovingAverage& operator=(const MovingAverage&) = delete;

		/** ムーブコンストラクタ. */
		MovingAverage(MovingAverage&&) = delete;

		/** ムーブ代入演算子. */
		MovingAverage& operator=(MovingAverage&&) = delete;

		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** データの追加.
		 * @param [in] x 追加データ
		 */
		void push_back(double x);

		/** 先頭データの取り出し. */
		void pop_front();

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** データの最大個数の取得.
		 * @return 平均
		 */
		size_t nmax() const;

		/** データ配列の参照.
		 * @return データ配列
		 */
		const std::deque<double>& x() const;

		/** 平均の取得.
		 * @return 平均
		 */
		double m() const;

		/** 分散の取得.
		 * @param [in] unbiased 不偏分散出力フラグ
		 * @return 分散
		 */
		double v(bool unbiased = false) const;

		/** 標準偏差の取得
		 * @param [in] unbiased 不偏標準偏差出力フラグ
		 * @return 補間データ配列のサイズ
		 */
		double s(bool unbiased = false) const;

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** データの最大個数 */
		size_t nmax_;

		/** データ */
		std::deque<double> x_;

		/** 平均 */
		double m_;

		/** 標本分散 */
		double v_;
	};
}

// End Of File -----------------------------------------------------------------
