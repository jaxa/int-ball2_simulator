
#include "prop/prop_manager.h"

namespace
{
	/** ファン数デフォルト値 */
	const int32_t          FAN_NUM(8);

	/** ファンステータスパブリッシュ周期　デフォルト値 */
	const double           PUB_DURATION(1.0);

	/** shutdownフラグモニタ周期 */
	const double           MON_DURATION(0.01);

	/** PWM制御周期　デフォルト値 */
	const unsigned short   FREQ(1000);
}

//------------------------------------------------------------------------------
// コンストラクタ
PropManager::PropManager(const ros::NodeHandle& nh) : 
nh_(nh),
fan_num_(FAN_NUM),
pwm_frequency_(FREQ),
pub_fan_status_duration_(PUB_DURATION),
init_error_id_(0)
{
	int ge = getParameter();

	prop_tlm_cmd_.initialize(nh_, fan_num_);

	const char* device_file_name = device_file_name_.c_str();
	bool        pe               = prop_pca9685_.initialize(device_file_name, device_address_, pwm_frequency_);

	duty_.resize(fan_num_, 0.0);

	// 初期化エラー識別
	//    0 : 成功
	//    1 : PCA9685 デバイスファイルオープンエラー
	//    2 : PCA9685 I2C SLAVEアドレス設定エラー
	//    4 : ROSPARAM ファン数取得エラー
	//    8 : ROSPARAM デバイスファイル名取得エラー
	//   16 : ROSPARAM デバイスアドレス取得エラー
	//   32 : ROSPARAM PWM周波数取得エラー
	//   64 : ROSPARAM ファンステータスパブリッシュ周期取得エラー
	//  128 : ROSPARAM I2C通信周期取得エラー
	init_error_id_ = ((ge << 2) | pe);
}

//------------------------------------------------------------------------------
// デストラクタ
PropManager::~PropManager()
{
	shutdown();
}

//------------------------------------------------------------------------------
// 管理機能実行
void PropManager::start()
{
	// ファン駆動デューティ比サブスクライブ＆PWM制御ボードへの送信
	if(init_error_id_ == 0)
	{
		fan_status_timer_  = nh_.createTimer(ros::Duration(pub_fan_status_duration_), &PropTlmCmd::pubFanStatus,      &prop_tlm_cmd_);
		pwm_control_timer_ = nh_.createTimer(ros::Duration(i2c_comm_duration_),       &PropManager::sendPWM,          this);
	}
	else
	{
		// 初期化エラーの場合は、ファンステータス(異常)のパブリッシュのみ(PWM信号は送信しない)
		ROS_ERROR("prop node initialization error : %d", init_error_id_);
		fan_status_timer_ = nh_.createTimer(ros::Duration(pub_fan_status_duration_),  &PropTlmCmd::pubErrorFanStatus, &prop_tlm_cmd_);
	}

	// パラメータ更新サービスサーバ
	update_params_server_ = nh_.advertiseService(SERVICE_UPDATE_PARAMS, &PropManager::updateParams, this);

	ros::spin();
}

//------------------------------------------------------------------------------
// 推進機能停止
void PropManager::shutdown()
{
	// テレメトリ・コマンド機能停止
	std::cout << "Shutdown Telemetry Command Function in Prop Node" << std::endl;
	prop_tlm_cmd_.shutdown();

	// I2C通信停止
	std::cout << "Shutdown I2C Communication in Prop Node"          << std::endl;
	duty_ = prop_tlm_cmd_.getFanDuty();
	prop_pca9685_.setPWM(duty_);
	prop_pca9685_.shutdown();

	// 管理機能停止
	std::cout << "Shutdown Manager Function in Prop Node"           << std::endl;
	if(fan_status_timer_)
	{
		std::cout << " -> Stop Fan Status Pubilsh Timer"            << std::endl;
		fan_status_timer_.stop();
	}
	
	if(pwm_control_timer_)
	{
		std::cout << " -> Stop PWM Control Signal Send Timer"       << std::endl;
		pwm_control_timer_.stop();
	}

	// 推進機能ノード停止
	if(!ros::isShuttingDown())
	{
		std::cout << " -> Shutdown Prop Node"                       << std::endl;
		ros::shutdown();
	}
}

//------------------------------------------------------------------------------
// PWM制御ボード(PCA9685)にPWM信号を送信
void PropManager::sendPWM(const ros::TimerEvent&)
{
	duty_ = prop_tlm_cmd_.getFanDuty();
	prop_pca9685_.setPWM(duty_);
}

//------------------------------------------------------------------------------
// rosparamからのパラメータ取得
int PropManager::getParameter()
{
	int ret = 0;

	if(!nh_.getParam("/prop/fan_number", fan_num_))
	{
		ROS_ERROR("Cannot Get /prop/fan_number in prop.cpp");
		ret = ret | 0x0001;
	}

	if(!nh_.getParam("/prop/device_file_name", device_file_name_))
	{
		ROS_ERROR("Cannot Get /prop/device_file_name in prop.cpp");
		ret = ret | 0x0002;
	}

	if(!nh_.getParam("/prop/device_address", device_address_))
	{
		ROS_ERROR("Cannot Get /prop/device_address in prop.cpp");
		ret = ret | 0x0004;
	}

	int freq;
	if(!nh_.getParam("/prop/pwm_frequency", freq))
	{
		ROS_ERROR("Cannot Get /prop/pwm_frequency in prop.cpp");
		ret = ret | 0x0008;
	}
	if(freq > 0)
	{
		pwm_frequency_ = static_cast<unsigned short>(freq);
	}

	if(!nh_.getParam("/prop/pub_fan_status_duration", pub_fan_status_duration_))
	{
		ROS_ERROR("Cannot Get /prop/pub_fan_status_duration in prop.cpp");
		ret = ret | 0x000F;
	}

	if(!nh_.getParam("/prop/i2c_comm_duration", i2c_comm_duration_))
	{
		ROS_ERROR("Cannot Get /prop/i2c_comm_duration in prop.cpp");
		ret = ret | 0x0010;
	}

	ROS_INFO("******** Set Parameters in prop_manager.cpp");
	ROS_INFO("/prop/fan_number                    : %d" , fan_num_);
	ROS_INFO("/prop/device_file_name              : %s" , device_file_name_.c_str());
	ROS_INFO("/prop/device_address                : %x" , device_address_);
	ROS_INFO("/prop/pwm_frequency                 : %d" , pwm_frequency_);
	ROS_INFO("/prop/pub_fan_status_duration       : %f" , pub_fan_status_duration_);
	ROS_INFO("/prop/i2c_comm_duration             : %f" , i2c_comm_duration_);

	return ret;
}

//------------------------------------------------------------------------------
// パラメータ更新
bool PropManager::updateParams
(ib2_msgs::UpdateParameter::Request&,
 ib2_msgs::UpdateParameter::Response& res)
{
	ROS_INFO("Update Parameters by /prop/update_params");

	res.stamp = ros::Time::now();
	int err   = getParameter();

	if (err == 0)
	{
		ROS_INFO("%s: Succeeded", SERVICE_UPDATE_PARAMS);
		res.status = ib2_msgs::UpdateParameter::Response::SUCCESS;
	}
	else
	{
		ROS_ERROR("%s: Failed : err = %d", SERVICE_UPDATE_PARAMS, err);
		res.status = ib2_msgs::UpdateParameter::Response::FAILURE_UPDATE;
	}

	return true;
}

// End Of File -----------------------------------------------------------------
