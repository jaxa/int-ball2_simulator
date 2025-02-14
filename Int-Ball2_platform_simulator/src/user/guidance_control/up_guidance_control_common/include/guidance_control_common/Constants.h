
#pragma once

#include <string>
#include <cmath>

namespace ib2_mss
{
	//--------------------------------------------------------------------------
	// 時間に関する定数
	/** 時間の単位変換係数 seconds / day. */
	const double DAY = 86400.;
	
	/** 時間の単位変換係数 seconds / hour. */
	const double HOUR = 3600.;
	
	/** 時間の単位変換係数 seconds / minute. */
	const double MINUTE = 60.;
	
	//--------------------------------------------------------------------------
	// 距離に関する定数
	/** 距離の単位変換係数 m / cm. */
	const double CM = 0.01;
	
	/** 距離の単位変換係数 m / km. */
	const double KM = 1000.;
	
	/** 距離の単位変換係数 m / NM. */
	const double NM = 1852.;
	
	/** 距離の単位変換係数 m / ft. */
	const double FT = 0.3048;
	
	/** 天文単位の単位変換係数 m / au */
	const double AU = 149597870700.;
	
	//--------------------------------------------------------------------------
	// 速度に関する定数
	/** 速度の単位変換係数 (m/s) / knot. */
	const double KT = NM / HOUR;
	
	/** 光速[m/s] */
	const double C0 = 299792458.;
	
	//--------------------------------------------------------------------------
	// 重量に関する定数
	/** 地球重力加速度[m/s2] */
	const double G = 9.80665;
	
	/** 重量の単位変換係数 kg / lb. */
	const double LB = 0.45359237;
	
	/** 質量の単位変換係数 kg / slug */
	const double SLUG = LB * G / FT;
	
	//--------------------------------------------------------------------------
	// 角度に関する定数
	/** 角度の単位変換係数 rad / deg */
	const double DEG = M_PI / 180.;
	
	/** 角度の単位変換係数 rad / arcsec */
	const double ARCSEC = DEG / 3600.;
	
	/** 2π */
	const double TWOPI = M_PI * 2.;
	
	//--------------------------------------------------------------------------
	// 地球形状に関する定数
	/** 地球赤道半径[m](GRSS80,WGS84). */
	const double RE_GRS80 = 6378137.;
	
	/** GRS80楕円体扁平率 */
	const double FE_GRS80 = 1./298.257222101;
	
	/** WGS84楕円体扁平率 */
	const double FE_WGS84 = 1./298.257223563;
	
	/** 地球重力定数[m3/s2] */
	const double GME_JGM3 = 3.98600441500e+14;
	
	/** 地球自転角速度[rad/s] see IERS Annual Report 2000 page.57 */
	const double WE_IERS2000 = 7.2921151467064e-5;
	
	//--------------------------------------------------------------------------
	// 地球環境に関する定数
	/** 標準大気地表面上密度[kg/m3] */
	const double RHO0 = 1.225;
	
	//--------------------------------------------------------------------------
	// 列挙子
	/** 座標軸の列挙子 */
	enum class AXIS : unsigned long
	{
		X,	///< X軸
		Y,	///< Y軸
		Z,	///< Z軸
	};
	
	/**
	 * @brief 定数の管理.
	 *
	 * 単位など普遍的なものはクラス外で定義する。
	 * 単位変換係数は、si単位系に変換するときに掛け算するように定義する。
	 * 実行環境によって変更するものをクラス内で定義する。
	 * ただし、メンバー関数instanceの静的変数を参照するため、インスタンスの削除は不要。
	 * スレッドセーフではないので、シングルスレッド状態においてconfigureすること。
	 * configureされなければデフォルト値が設定される。
	 */
	class Constants final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	private:
		/** デフォルトコンストラクタ. */
		Constants();
		
		/** デストラクタ */
		~Constants();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Constants(const Constants&) = delete;
		
		/** コピー代入演算子. */
		Constants& operator=(const Constants&) = delete;
		
		/** ムーブコンストラクタ. */
		Constants(Constants&&) = delete;
		
		/** ムーブ代入演算子. */
		Constants& operator=(Constants&&) = delete;
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** 定数の構成.
		 * 定数設定ファイルを読み込み、定数を設定する。
		 * @param [in] filename 定数設定ファイル名
		 * @retval true ログ初期化成功
		 * @retval false ログ初期化失敗
		 */
		static void configure(const std::string& filename);
		
		//----------------------------------------------------------------------
		// 属性(Getter)
	public:
		/** 地球赤道半径の取得.
		 * @return 地球赤道半径[m]
		 */
		static double re();
		
		/** 地球扁平率の取得.
		 * @return 地球扁平率
		 */
		static double fe();
		
		/** 地球重力定数の取得.
		 * @return 地球重力定数[m3/s2]
		 */
		static double ue();
		
		/** 地球自転角速度の取得.
		 * @return 地球自転角速度[rad/s]
		 */
		static double we();
		
		/** 標準大気密度の取得.
		 * @return 標準大気密度[kg/m3]
		 */
		static double rho0();
		
	private:
		/** インスタンスの取得(シングルトンパターン).
		 * @return ログクラスのインスタンスへの参照
		 */
		static Constants& instance();
		
		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** 地球赤道半径[m] */
		double re_;
		
		/** 地球扁平率 */
		double fe_;
		
		/** 地球重力定数[m3/s2] */
		double ue_;
		
		/** 地球自転角速度[rad/s] */
		double we_;
		
		/** 標準大気密度[kg/m3] */
		double rho0_;
	};
}
// End Of File ----------------------------------------------------------------
