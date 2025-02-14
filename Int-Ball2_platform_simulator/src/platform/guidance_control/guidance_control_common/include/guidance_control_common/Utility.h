
#pragma once

#include "RangeChecker.h"

#include <array>
#include <vector>
#include <deque>
#include <numeric>
#include <algorithm>
#include <Eigen/Core>
#include <unsupported/Eigen/NonLinearOptimization>

namespace ib2_mss
{
	/**
	 * @brief 便利関数.
	 *
	 * 便利関数を収録。
	 */
	class Utility
	{
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 入力値を周期的な範囲内に修正.
		 * 例 : 角度を±180degに修正する
		 * @param [in] x 修正前の値
		 * @param [in] min 範囲の最小値
		 * @param [in] max 範囲の最大値
		 * @return 最小値以上最大値未満に修正した結果
		 * @remarks min >= max のときは変換しない
		 */
		static double cyclicRange(double x, double min, double max);
		
		/** 多項式の計算
		 * @param [in] x 独立変数
		 * @param [in] c 係数(定数項、xの1,2,3...次項)
		 * @return 計算結果
		 */
		static double polynomial(double x, const std::vector<double>& c);
		
		/** dB値計算
		 * @param [in] r 真数(比,ratio)
		 * @return dB値
		 */
		static double dB(double r);
		
		/** 真数計算
		 * @param [in] dB デシベル値
		 * @return 真数
		 */
		static double dBinverse(double dB);
		
		/** １区間の台形積分.
		 * @param [in] x0 小さい側独立変数
		 * @param [in] x1 大きい側独立変数
		 * @param [in] y0 x0に対応する従属変数
		 * @param [in] y1 x1に対応する従属変数
		 * @return 台形積分結果
		 */
		static double trapz(double x0, double x1, double y0, double y1);
		
		/** 台形積分.
		 * @param [in] x 独立変数
		 * @param [in] y 確率密度
		 * @return 台形積分結果,対応する独立変数まで積分結果
		 */
		static std::vector<double> trapz
		(const std::vector<double>& x, const std::vector<double>& y);

		/** 二円の交わる面積
		 * @param [in] ra 円Aの半径
		 * @param [in] rb 円Bの半径
		 * @param [in] ab ２円の中心間距離
		 * @return 二円の交わる面積
		 */
		static double area2c(double ra, double rb, double ab);

		/** 三円の交わる面積
		 * @param [in] ra 円Aの半径
		 * @param [in] rb 円Bの半径
		 * @param [in] rc 円Cの半径
		 * @param [in] ab 円A,Bの中心間距離
		 * @param [in] bc 円B,Cの中心間距離
		 * @param [in] ca 円C,Aの中心間距離
		 * @return 二円の交わる面積
		 */
		static double area3c(double ra, double rb, double rc,
							 double ab, double bc, double ca);
		
		/** 円の交点
		 * @param [in] ra 円Aの半径
		 * @param [in] pa 円Aの中心位置
		 * @param [in] rb 円Bの半径
		 * @param [in] pb 円Bの中心位置
		 * @param [in] ccw 円Aの中心からみて反時計回りの交点を算出する
		 * @return 円Aと円Bの交点
		 */
		static Eigen::Vector2d intersection2c
		(double ra, const Eigen::Vector2d& pa,
		 double rb, const Eigen::Vector2d& pb, bool ccw = true);
		
		/** 三角形の面積
		 * @param [in] pa 点Aの位置
		 * @param [in] pb 点Bの位置
		 * @param [in] pc 点Cの位置
		 * @return 三角形ABCの面積
		 */
		static double areaTriangle
		(const Eigen::Vector2d& pa, const Eigen::Vector2d& pb,
		 const Eigen::Vector2d& pc);

		/** Levenberg Marquardt法の状態の確認
		 * @param [in] status Levenberg Marquardt法の状態
		 * @param [in] file		ファイル名
		 * @param [in] function	関数名
		 * @param [in] lineno	行番号
		 * @throw 異常終了
		 */
		static void
		checkStatusLM(const Eigen::LevenbergMarquardtSpace::Status& status,
					  const std::string& file, const std::string& function,
					  unsigned long lineno);
		
		/** カウンタ文字列の作成
		 * @param [in] count カウンタ
		 * @param [in] digits 桁数
		 * @param [in] filler 埋める文字
		 * @return カウンタ文字列（ゼロ埋め）
		 */
		static std::string counterString
		(size_t count, int digits, char filler = '0');

		/** 回帰平均
		 * @param [in, out] n 現在値のカウンタ
		 * @param [in] m 現在値を追加する前の平均値
		 * @param [in] c 現在値
		 * @return 現在値を考慮した平均値
		 */
		static double recmean(size_t& n, double m, double c);

		/** 整数指数のべき乗
		 * @param [in] base 基数
		 * @param [in] e 指数
		 * @return べき乗計算結果
		 */
		template <typename T>
		static T powint(T base, int32_t e);

		/** 整数への変換
		 * @param [in] o 変換対象
		 * @param [in] round 丸めフラグ
		 * @return 変換結果
		 */
		template <typename T>
		static int64_t int64(T o, bool round = true);

		/** 等差数列(arithmetic progression)ベクタの作成
		 * @param [in] min 最小値
		 * @param [in] max 最大値
		 * @param [in] d 交差
		 * @return 等差数列ベクタ
		 * @throws domain_error 交差が正でない(min >= max)
		 * @throws domain_error 交差が正でない(d <= 0)
		 */
		template <typename T>
		static std::vector<T> vectorAP(T min, T max, T d)
		{
			RangeChecker<T>::positive(max - min, true, "max - min");
			RangeChecker<T>::positive(d, true, "d");
			std::vector<T> a(static_cast<size_t>(std::floor((max-min)/d)) + 1);
			std::iota(a.begin(), a.end(), 0.);
			for (auto& ai: a)
				ai = std::fma(ai, d, min);
			return a;
		}
		
		/** vector配列への変換
		 * @param [in] input 元の配列
		 * @return vector配列
		 */
		template <class Range, typename T = typename Range::value_type>
		static std::vector<T> to_vector(const Range &input)
		{
			std::vector<T> output;
			std::copy(input.begin(), input.end(), std::back_inserter(output));
			return output;
		}
		
		/** deque配列への変換
		 * @param [in] input 元の配列
		 * @return vector配列
		 */
		template <class Range, typename T = typename Range::value_type>
		static std::deque<T> to_deque(const Range &input)
		{
			std::deque<T> output;
			std::copy(input.begin(), input.end(), std::back_inserter(output));
			return output;
		}
	};
}
// End Of File -----------------------------------------------------------------
