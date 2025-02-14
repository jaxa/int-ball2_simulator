
#pragma once

#include <string>
#include <sstream>
#include <deque>
#include <vector>
#include <memory>

namespace ib2_mss
{
	/**
	 * @brief 一次元補間器.
	 *
	 * 一次元のデータ点列に基づき補間する。補間方法は以下の通りである。
	 * - 線型補間
	 * - 多項式補間
	 * - 有理関数補間
	 */
	class Interpolator
	{
		//----------------------------------------------------------------------
		// 列挙子
	public:
		/** 範囲外の計算方法の列挙子 */
		enum class OUT_RANGE : unsigned int
		{
			EXTRA,	///< 補外
			BOUND,	///< 境界値
			ZERO,	///< 0を出力
			EXCEPT,	///< 例外
		};
		
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** 配列を設定しないコンストラクタ.
		 * 最小値と最大値が同じ場合は、範囲がなく、循環しないものとして取り扱う。
		 * @param [in] size データ配列の最大サイズ
		 * @param [in] xmin 独立変数範囲最小値
		 * @param [in] xmax 独立変数範囲最大値
		 * @param [in] ymin 従属変数範囲最小値
		 * @param [in] ymax 従属変数範囲最大値
		 */
		Interpolator(size_t size = 0,
					 double xmin = 0., double xmax = 0.,
					 double ymin = 0., double ymax = 0.);
		
		/** 配列を設定するコンストラクタ.
		 * 最小値と最大値が同じ場合は、範囲がなく、循環しないものとして取り扱う。
		 * @param [in] x 独立変数配列
		 * @param [in] y 従属変数配列
		 * @param [in] size データ配列の最大サイズ
		 * @param [in] xmin 独立変数範囲最小値
		 * @param [in] xmax 独立変数範囲最大値
		 * @param [in] ymin 従属変数範囲最小値
		 * @param [in] ymax 従属変数範囲最大値
		 */
		Interpolator(const std::deque<double>& x, const std::deque<double>& y,
					 size_t size = 0,
					 double xmin = 0., double xmax = 0.,
					 double ymin = 0., double ymax = 0.);
		
		/** 配列を設定するコンストラクタ.
		 * 最小値と最大値が同じ場合は、範囲がなく、循環しないものとして取り扱う。
		 * @param [in] x 独立変数配列
		 * @param [in] y 従属変数配列
		 * @param [in] size データ配列の最大サイズ
		 * @param [in] xmin 独立変数範囲最小値
		 * @param [in] xmax 独立変数範囲最大値
		 * @param [in] ymin 従属変数範囲最小値
		 * @param [in] ymax 従属変数範囲最大値
		 */
		Interpolator(const std::vector<double>& x, const std::vector<double>& y,
					 size_t size = 0,
					 double xmin = 0., double xmax = 0.,
					 double ymin = 0., double ymax = 0.);
		
		/** デストラクタ. */
		~Interpolator();
		
		//----------------------------------------------------------------------
		// コピー
	public:
		/** コピーコンストラクタ. */
		Interpolator(const Interpolator&) = delete;
		
		/** コピー代入演算子. */
		Interpolator& operator=(const Interpolator&) = delete;
		
		/** ムーブコンストラクタ. */
		Interpolator(Interpolator&&) = delete;
		
		/** ムーブ代入演算子. */
		Interpolator& operator=(Interpolator&&) = delete;
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** データ配列への補間点の追加.
		 * @param [in] x 独立変数
		 * @param [in] y 従属変数
		 * @return 挿入インデックス
		 * @throw invalid_argument 補間データ配列に入力された独立変数xが含まれている
		 */
		size_t insert(double x, double y);
		
		/** 指定インデックスの補間点をデータ配列から削除.
		 * @param [in] index 削除対象
		 * @throw out_of_range 削除インデックスが補間データサイズより大きい
		 */
		void erase(size_t index);
		
		/** 補間データ配列をクリア. */
		void clear();

	private:
		/** メンバ変数の設定. */
		void set();

		/** メンバ変数の並べ替え.
		 * 独立変数が昇順になるように補間データ配列を並べ替える。
		 */
		void sort();

		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 独立変数データ配列の参照.
		 * @return 独立変数データ配列
		 */
		const std::deque<double>& x() const;
		
		/** 従属変数データ配列の参照.
		 * @return 従属変数データ配列
		 */
		const std::deque<double>& y() const;
		
		/** 補間データの独立変数配列指定要素の取得.
		 * @param [in] index 参照番号
		 * @return 独立変数配列指定要素
		 */
		double x(size_t index) const;
		
		/** 補間データの従属変数配列指定要素の取得.
		 * @param [in] index 参照番号
		 * @return 従属変数配列指定要素
		 */
		double y(size_t index) const;
		
		/** 補間データ配列のサイズ.
		 * @return 補間データ配列のサイズ
		 */
		size_t size() const;
		
		/** 補間データ配列の最大サイズ.
		 * @return 補間データ配列の最大サイズ
		 */
		size_t maxSize() const;
		
		/** 独立変数範囲最小値の取得.
		 * @return 独立変数範囲最小値
		 */
		double xmin() const;
		
		/** 独立変数範囲最大値の取得.
		 * @return 独立変数範囲最大値
		 */
		double xmax() const;
		
		/** 従属変数範囲最小値の取得.
		 * @return 従属変数範囲最小値
		 */
		double ymin() const;
		
		/** 従属変数範囲最大値の取得.
		 * @return 従属変数範囲最大値
		 */
		double ymax() const;
		
		/** 文字列の取得.
		 * @param [in] precision 精度
		 * @return ベクトルの各成分を記述した文字列
		 */
		std::string string(std::streamsize precision = 16) const;
		
		/** 同じ独立変数の確認
		 * @param [in] x 独立変数
		 * @retval true 同じ独立変数を持つ
		 * @retval true 同じ独立変数を持たない
		 */
		bool hasX(double x) const;
		
	private:
		/** 補間データ開始点のインデックスを取得.
		 * 独立変数の差が最大の区間の大きい側インデックスが補間データ開始点となる。
		 * @return 補間データ開始点のインデックス
		 */
		size_t indexStart() const;
		
		/** 補間のための修正データ配列の取得.
		 * 隣接する要素の変動が循環周期の半分以下になるように修正する。
		 * @param [in] x 補間位置の独立変数（範囲修正済み）
		 * @return 独立変数と従属変数の配列修正結果
		 */
		std::pair<std::deque<double>, std::deque<double>>
		modifiedArray(double x) const;
		
		/** 循環範囲のメンバ変数の確認.
		 * 最小値が最大値以下であることを確認する。
		 * @throw invalid_argument 設定された補間用データが異常
		 */
		void checkRange() const;
		
		/** メンバ変数(補間用データ)並べ替えの確認.
		 * 独立変数が昇順に並んでいることを確認する。
		 * @retval true ソート済み
		 * @retval false ソート済みでないか、同値が存在する。
		 */
		bool isSorted() const;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** 線型補間.
		 * 補間データ点列を線型補間する。定義域外では、近い側の端の区間の２点を用いる。
		 * @param [in] x 補間点の独立変数
		 * @param [out] dydx 補間点の傾き
		 * @param [in] ob 範囲外の計算方法
		 * @return 補間点の従属変数
		 */
		double linear(double x, double *dydx = nullptr,
					  const OUT_RANGE& ob = OUT_RANGE::EXTRA) const;
		
		/** 線型補間.
		 * 補間データ点列を線型補間する。定義域外では、近い側の端の区間の２点を用いる。
		 * @param [in] x 補間点の独立変数
		 * @param [out] dydx 補間点の傾き
		 * @param [in] ob 範囲外の計算方法
		 * @return 補間点の従属変数
		 */
		std::vector<double> linear
		(const std::vector<double>& x, std::vector<double> *dydx = nullptr,
		 const OUT_RANGE& ob = OUT_RANGE::EXTRA)const;
		
		/** 多項式補間(Lagrange補間).
		 * 補間データ点列を多項式で補間する。
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		double polynomial(double x, double *dy = nullptr) const;
		
		/** 有理関数補間.
		 * 補間データ点列を有理関数で補間する。
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		double rational(double x, double *dy = nullptr) const;
		
		/** 線形補間
		 * @param [in] x0 小さい側独立変数
		 * @param [in] x1 大きい側独立変数
		 * @param [in] y0 x0に対応する従属変数
		 * @param [in] y1 x1に対応する従属変数
		 * @param [in] x 補間点の独立変数
		 * @return 線形補間結果,傾き
		 */
		static std::pair<double, double> linear
		(double x0, double x1, double y0, double y1, double x);

		/** 線型補間.
		 * 補間データ点列を線型補間する。定義域外では、近い側の端の区間の２点を用いる。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dydx 補間点の傾き
		 * @param [in] ob 範囲外の計算方法
		 * @return 補間点の従属変数
		 */
		static double linear
		(const std::deque<double>& xa, const std::deque<double>& ya, double x,
		 double *dydx = nullptr, const OUT_RANGE& ob = OUT_RANGE::EXTRA);
		
		/** 線型補間.
		 * 補間データ点列を線型補間する。定義域外では、近い側の端の区間の２点を用いる。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dydx 補間点の傾き
		 * @param [in] ob 範囲外の計算方法
		 * @return 補間点の従属変数
		 */
		static double linear
		(const std::vector<double>& xa, const std::vector<double>& ya, double x,
		 double *dydx = nullptr, const OUT_RANGE& ob = OUT_RANGE::EXTRA);
		
		/** 多項式補間(Lagrange補間).
		 * 補間データ点列を多項式で補間する。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		static double polynomial
		(const std::deque<double>& xa, const std::deque<double>& ya, double x,
		 double *dy = nullptr);
		
		/** 多項式補間(Lagrange補間).
		 * 補間データ点列を多項式で補間する。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		static double polynomial
		(const std::vector<double>& xa, const std::vector<double>& ya, double x,
		 double *dy = nullptr);
		
		/** 有理関数補間.
		 * 補間データ点列を有理関数で補間する。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		static double rational
		(const std::deque<double>& xa, const std::deque<double>& ya, double x,
		 double *dy = nullptr);
		
		/** 有理関数補間.
		 * 補間データ点列を多項式で補間する。
		 * @param [in] xa 独立変数のデータ配列
		 * @param [in] ya 従属変数のデータ配列
		 * @param [in] x 補間点の独立変数
		 * @param [out] dy 出力する従属変数の誤差評価
		 * @return 補間点の従属変数
		 */
		static double rational
		(const std::vector<double>& xa, const std::vector<double>& ya, double x,
		 double *dy = nullptr);
		
		/** 範囲外種別の取得
		 * @param [in] type 範囲外種別文字列(EXTRA/BOUND/EXCEPT)
		 * @return 範囲外種別
		 */
		static OUT_RANGE outrange(const std::string& type);
		
		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** 独立変数 (independent variables) */
		std::deque<double> x_;
		
		/** 従属変数 (dependent variables) */
		std::deque<double> y_;
		
		/** 最大要素数 */
		size_t maxSize_;
		
		/** 独立変数の最小値 */
		double xmin_;
		
		/** 独立変数の最大値 */
		double xmax_;
		
		/** 従属変数の最小値 */
		double ymin_;
		
		/** 従属変数の最大値 */
		double ymax_;
	};
}

// End Of File -----------------------------------------------------------------
