
#pragma once

#include <ros/ros.h>

#include <deque>

#include <Eigen/Dense>

// Standard messages
#include "ib2_msgs/Navigation.h"
#include "ib2_msgs/CtlStatus.h"

#include "guidance_control_common/MovingAverage.h"

/**
* @brief 検知クラス
*/
class Dtc
{
	//--------------------------------------------------------------------------
	// 列挙子
public:
	/** 検知内容の列挙子 */
	enum class DETECT : unsigned int
	{
		NONE,			///< 検知なし
		DOCKING,		///< ドッキング
		DISTURBED,		///< 外乱発生（衝突/クルーキャプチャ）
		COLLISION,		///< 衝突
		CREW_CAPTURE,	///< クルーキャプチャー
		CREW_RELEASE	///< クルーリリース
	};

	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
private:
	/** デフォルトコンストラクタ */
	Dtc() = delete;

public:
	
	/** コンストラクタ */
	explicit Dtc(const ros::NodeHandle& nh);

	/** デストラクタ */
	~Dtc();
	
	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	Dtc(const Dtc&) = delete;

	/** コピー代入演算子. */
	Dtc& operator=(const Dtc&) = delete;

	/** ムーブコンストラクタ. */
	Dtc(Dtc&&) = delete;

	/** ムーブ代入演算子. */
	Dtc& operator=(Dtc&&) = delete;

	//----------------------------------------------------------------------
	// 操作(Setter)
public:
	/** メンバ設定
	 * @retval true 設定成功
	 * @retval false 設定失敗
	 */
	bool setMember();
	
	//----------------------------------------------------------------------
	// 属性(Getter)
public:
	/** ステータスの取得.
	 * @return ステータス
	 */
	DETECT status() const;

	/** ドッキング目標値の取得
	 * @param [in] t 目標設定時刻
	 * @return ドッキング目標値航法メッセージ
	 */
	ib2_msgs::Navigation dockingTarget(const ros::Time& t) const;

	//----------------------------------------------------------------------
	// 実装
public:
	/** 検知処理
	 * @param [in] nav_stamp 航法値
	 * @param [in] ctl_status 誘導ステータス
	 * @return ステータス
	 */
	Dtc::DETECT detection(const ib2_msgs::Navigation nav_stamp, const int32_t ctl_status);

	/** 検知ステータスのクリア
	 */
	void clearStatus();

private:
	/** 判定
	 * @param [in] ctl_status 誘導ステータス
	 */
	void check(const int32_t ctl_status);

	/** 航法暦
	 */
	 void history(const ib2_msgs::Navigation nav_stamp);

	/** 衝突・クルーリリース判定
	 */
	void colrelCheck();

	/** 衝突判定
	 */
	void collisionCheck();

	/** クルーリリース判定
	 */
	void releaseCheck();

	/** 標準偏差の立ち上がり判定
	 * @param [in] sigma_dacc    標準偏差立ち上がり加速度差分判定値
	 * @param [in] sigma_drate   標準偏差立ち上がり角速度差分判定値
	 * @param [in] sigma_jud_num 標準偏差立ち上がり判定回数
	 * @retval true  立ち上がった
	 * @retval false 立ち上がっていない
	 */
	bool sigmaAscent(const double sigma_dacc,const double sigma_drate,
    			const int sigma_jud_num);

	/** クルーのキャプチャ判定
	 * @retval true クルーキャプチャ
	 * @retval false クルーキャプチャではない
	 */
	bool crewCapCheck();

	/** ドッキング判定
	 */
	void dockingCheck();

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ROSノードハンドラ */
	ros::NodeHandle nh_;

	/** 前回の航法値*/
	ib2_msgs::Navigation last_nav_stamp_;

	/** 最新の位置*/
	Eigen::Vector3d rc_;

	/** 最新の姿勢*/
	Eigen::Quaterniond qc_;

	/** キューサイズ */
	unsigned int que_size_; 

	/** ステータス */
	DETECT status_;

	/** ドッキング位置[m] */
	Eigen::Vector3d docking_pos_;

	/** ドッキング姿勢[-] */
	Eigen::Quaterniond docking_q_;

	/** ドッキング位置到達判定値[m] */
	double tolerance_pos_;

	/** ドッキング姿勢到達判定値[-] */
	double tolerance_att_;

	/** ドッキング状態維持判定回数[-] */
	int keep_state_num_;
//
	/** D/S接触判定用　標準偏差立ち上がり加速度差分判定値[m/s2] */
	double dc_sigma_start_dacc_;

	/** D/S接触判定用　標準偏差立ち上がり角速度差分判定値[rad/s] */
	double dc_sigma_start_drate_;

	/** D/S接触判定用　標準偏差立ち上がり判定回数[-] */
	int dc_sigma_start_jud_num_;
//
	/** ドッキング状態維持カウンタ[-] */
	int keep_state_counter_;

	/** 標準偏差立ち上がり判定フラグ */
	bool dtc_sigmaup_started_;

	/** 標準偏差立ち上がり加速度差分判定値[m/s2] */
	double sigma_start_dacc_;

	/** 標準偏差立ち下がり加速度差分判定値[m/s2] */
	double sigma_end_dacc_;

	/** 標準偏差立ち上がり角速度差分判定値[rad/s] */
	double sigma_start_drate_;

	/** 標準偏差立ち下がり角速度差分判定値[rad/s] */
	double sigma_end_drate_;

	/** 標準偏差立ち上がり判定回数[-] */
	int sigma_start_jud_num_;

	/** 衝突・クルーリリース判定回数[-] */
	int sigma_end_jud_num_;

	/** 衝突とクルーキャプチャの識別用の判定回数[-] */
	int colcap_id_jud_num_;

	/** 衝突・クルーキャプチャ開始判定カウンタ[-] */
	int sigma_start_counter_;

	/** 衝突・クルーリリース判定値カウンタ[-] */
	int sigma_end_counter_;

	/** 衝突判定値カウンタ[-] */
	int sigma_end_counter_colcheck_;

	/** 標準偏差が立ち上がってからの回数[-] */
	int from_sigup_counter_;

	/** 加速度差分X成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_daccx_;

	/** 加速度差分Y成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_daccy_;

	/** 加速度差分Z成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_daccz_;

	/** 角速度差分X成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_dwx_;

	/** 角速度差分Y成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_dwy_;

	/** 角速度差分Z成分 MovingAverage */
	std::unique_ptr<ib2_mss::MovingAverage> moave_dwz_;

	/** 加速度差分各成分の標準偏差のRSS */
	double std_a_;

	/** 角速度差分各成分の標準偏差のRSS */
	double std_w_;
};
// End Of File -----------------------------------------------------------------
