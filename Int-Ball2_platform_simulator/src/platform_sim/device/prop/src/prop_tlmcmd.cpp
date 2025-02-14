
#include "prop/prop_tlmcmd.h"

//------------------------------------------------------------------------------
// コンストラクタ
PropTlmCmd::PropTlmCmd()  = default;

//------------------------------------------------------------------------------
// デストラクタ
PropTlmCmd::~PropTlmCmd() = default;

//------------------------------------------------------------------------------
// テレメトリ・コマンド機能初期化
void PropTlmCmd::initialize(const ros::NodeHandle& nh, const int32_t& fan_num) 
{
	nh_             = nh;
	fan_num_        = fan_num;

	// Duty初期化
	fan_duty_.layout.dim.push_back(std_msgs::MultiArrayDimension());
	fan_duty_.layout.dim[0].size   = fan_num_;
	fan_duty_.layout.dim[0].stride = 1;
	fan_duty_.layout.dim[0].label  = "";
	fan_duty_.layout.data_offset   = 0;
	fan_duty_.data.resize(fan_num_, 0.0);

	// ファン駆動状態メッセージ初期化
	fan_status_.duty.layout.dim.push_back(std_msgs::MultiArrayDimension());
	fan_status_.duty.data.resize(fan_num_, 0.0);
	initFanStatus(ib2_msgs::PowerStatus::ON);
	
	// ファン駆動デューティ比サブスクライバ
	sub_fan_duty_          = nh_.subscribe(TOPIC_CTL_DUTY, 1, &PropTlmCmd::subFanDuty, this);

	// ファン駆動状態パブリッシャ
	pub_fan_status_        = nh_.advertise<ib2_msgs::FanStatus>(TOPIC_PROP_STATUS, 1);

	// ファン駆動モード設定サービスサーバ
	switch_power_server_   = nh_.advertiseService(SERVICE_SWITCH_POWER, &PropTlmCmd::switchPower, this);
}

//------------------------------------------------------------------------------
// ファン駆動デューティ比をサブスクライブ
void PropTlmCmd::subFanDuty(const std_msgs::Float64MultiArray& msg)
{
	// データサイズチェック
	int size = msg.data.size();
	if(size != fan_num_)
	{
		ROS_ERROR("Prop Node Subscribed Invalid Size(%d) of Fan Duty", size);
		initFanStatus(ib2_msgs::PowerStatus::UNKNOWN);
		return;
	}

	fan_duty_ = msg;

	generateFanStatus();
}

//------------------------------------------------------------------------------
// ファン駆動状態メッセージ生成
void PropTlmCmd::generateFanStatus()
{
	initFanStatus(fan_status_.current_power.status);

	// 推進機能が停止の場合は、デューティ比 = 0とする
	if(fan_status_.current_power.status != ib2_msgs::PowerStatus::ON)
	{
		return;
	}

	// 推進機能が起動の場合、サブスクライブしたデューティ比を設定する
	for(int i = 0; i < fan_num_; i++)
	{
		fan_status_.duty.data[i] = limitter(fan_duty_.data[i], DUTY_MIN, DUTY_MAX);
	}
}

//------------------------------------------------------------------------------
// ファン駆動状態をパブリッシュ
void PropTlmCmd::pubFanStatus(const ros::TimerEvent&)
{
	pub_fan_status_.publish(fan_status_);
}

//------------------------------------------------------------------------------
// ファン駆動状態(異常停止中)をパブリッシュ
void PropTlmCmd::pubErrorFanStatus(const ros::TimerEvent& ev)
{
	initFanStatus(ib2_msgs::PowerStatus::UNKNOWN);
	pubFanStatus(ev);
}

//------------------------------------------------------------------------------
// 推進機能起動/停止
bool PropTlmCmd::switchPower
(ib2_msgs::SwitchPower::Request&   req, 
 ib2_msgs::SwitchPower::Response&  res)
{
	fan_status_.current_power.status = req.power.status;
	res.current_power.status         = fan_status_.current_power.status;

	if(req.power.status != ib2_msgs::PowerStatus::ON)
	{
		for(int i = 0; i < fan_num_; i++)
		{
			fan_duty_.data[i] = 0.0;
		}
	}
	
	return true;
}

//------------------------------------------------------------------------------
// ファン駆動デューティ比のgetter
std::vector<float> PropTlmCmd::getFanDuty()
{
	std::vector<float> duty(fan_num_, 0.0);

	for(int i = 0; i < fan_num_; i++)
	{
		duty[i] = fan_status_.duty.data[i];
	}

	// PWM制御ボードに送信するデューティで、ファン駆動状態メッセージを更新
	generateFanStatus();

	return duty;
}

//------------------------------------------------------------------------------
// ファン駆動状態初期化
void PropTlmCmd::initFanStatus(const uint8_t& status)
{
	fan_status_.header.seq                 = (fan_status_.header.seq + 1) % UINT32_MAX;
	fan_status_.header.stamp               = ros::Time::now();
	fan_status_.header.frame_id            = "";

	fan_status_.duty.layout.dim[0].size    = fan_num_;
	fan_status_.duty.layout.dim[0].stride  = 1;
	fan_status_.duty.layout.dim[0].label   = "";
	fan_status_.duty.layout.data_offset    = 0;

	for(int i = 0; i < fan_num_; i++)
	{
		fan_status_.duty.data[i]   = 0.0;
	}

	fan_status_.current_power.status = status;
}

//------------------------------------------------------------------------------
// テレメトリ・コマンド機能停止
void PropTlmCmd::shutdown()
{
	if(sub_fan_duty_){
		std::cout << " -> Shutdown Fan Duty Subscriber"           << std::endl;
		sub_fan_duty_.shutdown();
	}

	if(pub_fan_status_){
		std::cout << " -> Shutdown Fan Status Publisher"          << std::endl;
		pub_fan_status_.shutdown();
	}

	if(switch_power_server_)
	{
		std::cout << " -> Shutdown Switch Power Service Server" << std::endl;
		switch_power_server_.shutdown();
	}

	initFanStatus(ib2_msgs::PowerStatus::UNKNOWN);
}

// End Of File -----------------------------------------------------------------
