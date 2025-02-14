
#pragma once

#include <string>
#include <chrono>

namespace ib2_mss
{
	/**
	 * @brief 実行時間の計測.
	 *
	 * プログラムの実行時間を計測する。
	 * - コンストラクタで計測開始時刻を記録する。
	 * - elapsed()で、計測開始時刻からの経過時間[sec]を出力する。
	 * - information()で、識別名称と経過時間[sec]を文字列で出力する。
	 */
	class StopWatch
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** コンストラクタ. */
		explicit StopWatch(const std::string& name = "");

		/** デストラクタ. */
		~StopWatch();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		StopWatch(const StopWatch&) = delete;
		
		/** コピー代入演算子. */
		StopWatch& operator=(const StopWatch&) = delete;
		
		/** ムーブコンストラクタ. */
		StopWatch(StopWatch&&) = delete;
		
		/** ムーブ代入演算子. */
		StopWatch& operator=(StopWatch&&) = delete;
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 識別名称の取得.
		 * @return 識別名称[sec]
		 */
		const std::string& name() const;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** 計測開始からの経過時間の出力.
		 * 計測開始時刻からの経過時間を計算し、出力する。
		 * @return 計測開始からの経過時間[sec]
		 */
		double elapsedsec() const;

		/** 計測開始からの経過時間コメントの出力.
		 * "[識別名称] took XX s."
		 * @param [in] digits 小数点以下表示桁数
		 * @return 計測開始からの経過時間コメント
		 */
		std::string elapsed(size_t digits = 9) const;
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 識別名称 */
		std::string name_;
		
		/** 開始時刻. */
		std::chrono::steady_clock::time_point start_;
	};
}
// End Of File -----------------------------------------------------------------
