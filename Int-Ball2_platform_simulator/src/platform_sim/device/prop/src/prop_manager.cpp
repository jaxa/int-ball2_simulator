
#include "prop/prop_manager.h"

namespace
{
	/** ファン数デフォルト値 */
	const int32_t          FAN_NUM(8);

	/** ファンステータスパブリッシュ周期　デフォルト値 */
	const double           PUB_DURATION(1.0);

	/** PWM制御周期　デフォルト値 */
	const unsigned short   FREQ(1000);
}

//------------------------------------------------------------------------------
// コンストラクタ
PropManager::PropManager() :
rclcpp::Node("prop"),
fan_num_(FAN_NUM),
pwm_frequency_(FREQ),
pub_fan_status_duration_(PUB_DURATION),
init_error_id_(0)
{
	// パラメータ宣言
	this->declare_parameter("/prop/fan_number", static_cast<int>(FAN_NUM));
	this->declare_parameter("/prop/device_file_name", std::string(""));
	this->declare_parameter("/prop/device_address", 0);
	this->declare_parameter("/prop/pwm_frequency", static_cast<int>(FREQ));
	this->declare_parameter("/prop/pub_fan_status_duration", PUB_DURATION);
	this->declare_parameter("/prop/i2c_comm_duration", 0.01);

	int ge = getParameter();

	prop_tlm_cmd_.initialize(this, fan_num_);

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

	// ファン駆動デューティ比サブスクライブ＆PWM制御ボードへの送信
	if(init_error_id_ == 0)
	{
		fan_status_timer_  = this->create_wall_timer(
			std::chrono::duration<double>(pub_fan_status_duration_),
			[this]() { prop_tlm_cmd_.pubFanStatus(); });
		pwm_control_timer_ = this->create_wall_timer(
			std::chrono::duration<double>(i2c_comm_duration_),
			std::bind(&PropManager::sendPWM, this));
	}
	else
	{
		// 初期化エラーの場合は、ファンステータス(異常)のパブリッシュのみ(PWM信号は送信しない)
		RCLCPP_ERROR(this->get_logger(), "prop node initialization error : %d", init_error_id_);
		fan_status_timer_ = this->create_wall_timer(
			std::chrono::duration<double>(pub_fan_status_duration_),
			[this]() { prop_tlm_cmd_.pubErrorFanStatus(); });
	}

	// パラメータ更新サービスサーバ
	update_params_server_ = this->create_service<ib2_msgs::srv::UpdateParameter>(
		SERVICE_UPDATE_PARAMS,
		std::bind(&PropManager::updateParams, this, std::placeholders::_1, std::placeholders::_2));
}

//------------------------------------------------------------------------------
// デストラクタ
PropManager::~PropManager()
{
	shutdown();
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
		fan_status_timer_->cancel();
	}

	if(pwm_control_timer_)
	{
		std::cout << " -> Stop PWM Control Signal Send Timer"       << std::endl;
		pwm_control_timer_->cancel();
	}
}

//------------------------------------------------------------------------------
// PWM制御ボード(PCA9685)にPWM信号を送信
void PropManager::sendPWM()
{
	duty_ = prop_tlm_cmd_.getFanDuty();
	prop_pca9685_.setPWM(duty_);
}

//------------------------------------------------------------------------------
// rosparamからのパラメータ取得
int PropManager::getParameter()
{
	int ret = 0;

	if(!this->get_parameter("/prop/fan_number", fan_num_))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/fan_number in prop.cpp");
		ret = ret | 0x0001;
	}

	if(!this->get_parameter("/prop/device_file_name", device_file_name_))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/device_file_name in prop.cpp");
		ret = ret | 0x0002;
	}

	if(!this->get_parameter("/prop/device_address", device_address_))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/device_address in prop.cpp");
		ret = ret | 0x0004;
	}

	int freq;
	if(!this->get_parameter("/prop/pwm_frequency", freq))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/pwm_frequency in prop.cpp");
		ret = ret | 0x0008;
	}
	if(freq > 0)
	{
		pwm_frequency_ = static_cast<unsigned short>(freq);
	}

	if(!this->get_parameter("/prop/pub_fan_status_duration", pub_fan_status_duration_))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/pub_fan_status_duration in prop.cpp");
		ret = ret | 0x000F;
	}

	if(!this->get_parameter("/prop/i2c_comm_duration", i2c_comm_duration_))
	{
		RCLCPP_ERROR(this->get_logger(), "Cannot Get /prop/i2c_comm_duration in prop.cpp");
		ret = ret | 0x0010;
	}

	RCLCPP_INFO(this->get_logger(), "******** Set Parameters in prop_manager.cpp");
	RCLCPP_INFO(this->get_logger(), "/prop/fan_number                    : %d" , fan_num_);
	RCLCPP_INFO(this->get_logger(), "/prop/device_file_name              : %s" , device_file_name_.c_str());
	RCLCPP_INFO(this->get_logger(), "/prop/device_address                : %x" , device_address_);
	RCLCPP_INFO(this->get_logger(), "/prop/pwm_frequency                 : %d" , pwm_frequency_);
	RCLCPP_INFO(this->get_logger(), "/prop/pub_fan_status_duration       : %f" , pub_fan_status_duration_);
	RCLCPP_INFO(this->get_logger(), "/prop/i2c_comm_duration             : %f" , i2c_comm_duration_);

	return ret;
}

//------------------------------------------------------------------------------
// パラメータ更新
void PropManager::updateParams(
	const std::shared_ptr<ib2_msgs::srv::UpdateParameter::Request>,
	std::shared_ptr<ib2_msgs::srv::UpdateParameter::Response> response)
{
	RCLCPP_INFO(this->get_logger(), "Update Parameters by /prop/update_params");

	response->stamp = this->now();
	int err   = getParameter();

	if (err == 0)
	{
		RCLCPP_INFO(this->get_logger(), "%s: Succeeded", SERVICE_UPDATE_PARAMS);
		response->status = ib2_msgs::srv::UpdateParameter::Response::SUCCESS;
	}
	else
	{
		RCLCPP_ERROR(this->get_logger(), "%s: Failed : err = %d", SERVICE_UPDATE_PARAMS, err);
		response->status = ib2_msgs::srv::UpdateParameter::Response::FAILURE_UPDATE;
	}
}

// End Of File -----------------------------------------------------------------
