
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "prop/prop_tlmcmd.h"
#include "prop/prop_pca9685.h"
#include "ib2_msgs/srv/update_parameter.hpp"

#define   SERVICE_UPDATE_PARAMS       "/prop/update_params"

class PropManager : public rclcpp::Node
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	PropManager();

	/** デストラクタ */
	~PropManager();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	PropManager(const PropManager&)            = delete;

	/** コピー代入演算子. */
	PropManager& operator=(const PropManager&) = delete;

	/** ムーブコンストラクタ. */
	PropManager(PropManager&&)                 = delete;

	/** ムーブ代入演算子. */
	PropManager& operator=(PropManager&&)      = delete;

	//--------------------------------------------------------------------------
	// 実装
private:
	/** rosparamからのパラメータ取得
	 * @return                     エラー識別子
	 *                                  0 : 取得成功
	 *                                  1 : ファン数取得エラー
	 *                                  2 : デバイスファイル名取得エラー
	 *                                  4 : デバイスアドレス取得エラー
	 *                                  8 : PWM周波数取得エラー
	 *                                  16: ファンステータスパブリッシュ周期取得エラー
	 *                                  32: I2C通信周期取得エラー
	 */
    int getParameter();

	/** PWM制御ボード(PCA9685)にPWM信号を送信 */
	void sendPWM();

	/** 推進機能ノード停止 */
	void shutdown();

	/** パラメータ更新サービス受信時の処理
	 * @param [in] request                                パラメータ更新サービスリクエスト
	 * @param [in] response                               パラメータ更新サービス実行結果
	 */
	void updateParams(
		const std::shared_ptr<ib2_msgs::srv::UpdateParameter::Request> request,
		std::shared_ptr<ib2_msgs::srv::UpdateParameter::Response> response);

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ファン駆動状態パブリッシュ用　ROS Timer */
	rclcpp::TimerBase::SharedPtr     fan_status_timer_;

	/** PWM制御信号送信用　ROS Timer */
	rclcpp::TimerBase::SharedPtr     pwm_control_timer_;

	/** パラメータ更新サービスサーバ */
	rclcpp::Service<ib2_msgs::srv::UpdateParameter>::SharedPtr update_params_server_;

    /* ファン数 */
    int32_t                          fan_num_;

    /** テレメトリ・コマンド　オブジェクト */
    PropTlmCmd                       prop_tlm_cmd_;

    /** PWM制御信号送信　オブジェクト */
	PropPCA9685                      prop_pca9685_;

    /** デバイスファイル名(PCA9685) */
    std::string                      device_file_name_;

    /** デバイスのアドレス */
    int                              device_address_;

    /** PWM周期 */
    unsigned short                   pwm_frequency_;

    /** ファンステータスのパブリッシュ周期[s] */
    double                           pub_fan_status_duration_;

	/** I2C通信周期[s] */
    double                           i2c_comm_duration_;

    /** 初期化エラー識別子 */
    int                              init_error_id_;

	/** ファンデューティ */
	std::vector<float>               duty_;
};
// End Of File -----------------------------------------------------------------
