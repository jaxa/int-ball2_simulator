
#include "ctl_command_client/ctl_command_client.h"

#include "guidance_control_common/Constants.h"
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Mjd.h"
#include "guidance_control_common/Log.h"

#include "ib2_msgs/msg/ctl_status_type.hpp"
#include "ib2_msgs/msg/navigation.hpp"

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <fstream>
#include <stdexcept>
#include <algorithm>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 制御目標アクションの名前 */
	const std::string ACTION_NAME("/ctl/command");

	const std::string FRAME_ISS("iss_body");
	const std::string FRAME_IB2("body");

	/** コマンド種別文字列 */
	const std::vector<std::string> COMMAND_TYPE
	{
		"STAND_BY",
		"KEEP_POSE",
		"STOPPING_TARGET",
		"RELATIVE_TARGET",
		"ABSOLUTE_TARGET",
		"RELEASE",
		"DOCKING",
		"SCAN",
		"DOCKING_WITHOUT_CORRECTION",
	};

	/** コマンド種別の出力
	 * @param [in] str コマンド種別文字列
	 * @return コマンド種別番号
	 */
	uint8_t commandType(const std::string& str)
	{
		auto ptr(std::find(COMMAND_TYPE.begin(), COMMAND_TYPE.end(), str));
		if (ptr == COMMAND_TYPE.end())
		{
			std::string what("invalid command type string");
			LOG_ERROR(what + " : " + str);
			throw std::domain_error(what);
		}
		auto index(std::distance(COMMAND_TYPE.begin(), ptr));
		return static_cast<uint8_t>(index);
	}

	/** 姿勢回転角がLongerPathであることの判定
	 * @param [in] dyaw ヨー角コマンド[deg]
	 * @param [in] dpitch ピッチ角コマンド[deg]
	 * @param [in] droll ロール角コマンド[deg]
	 * @retval true Longer Path
	 * @retval false Shorter Path
	 */
	bool isLongerPath(double dyaw, double dpitch, double droll)
	{
		static const double THR(180.);
		return dyaw > THR || dpitch > THR || droll > THR;
	}

	/** 移動量の計算
	 * @param [in] dyaw ヨー角コマンド[deg]
	 * @param [in] dpitch ピッチ角コマンド[deg]
	 * @param [in] droll ロール角コマンド[deg]
	 * @return 姿勢移動量[ND]
	 */
	Eigen::Quaterniond att_maneuver(double dyaw, double dpitch, double droll)
	{
		using namespace ib2_mss;
		Eigen::AngleAxisd rotz(dyaw   * DEG, Eigen::Vector3d::UnitZ());
		Eigen::AngleAxisd roty(dpitch * DEG, Eigen::Vector3d::UnitY());
		Eigen::AngleAxisd rotx(droll  * DEG, Eigen::Vector3d::UnitX());
		Eigen::Quaterniond dq(rotz * roty * rotx);

		bool longer(isLongerPath(dyaw, dpitch, droll));
		if (( longer && dq.w() > 0.) ||
			(!longer && dq.w() < 0.))
		{
			dq.coeffs() = -dq .coeffs();
		}
		return dq;
	}
}

/** キーボード操作検知
 * @retval 1 キーボードが押された
 * @retval 0 キーボードが押されていない
 */
int kbhit(void)
{
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

	ch = getchar();

	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);

	if (ch != EOF) {
		ungetc(ch, stdin);
		return 1;
	}

	return 0;
}


//------------------------------------------------------------------------------
// デフォルトコンストラクタ
CtlCommandClient::CtlCommandClient() :
	rclcpp::Node("target_node"), done_(false)
{
	ac_ = rclcpp_action::create_client<CtlCommandAction>(this, ACTION_NAME);

	this->declare_parameter<std::string>("paramfile", "");
	std::string paramfile;
	this->get_parameter("paramfile", paramfile);

	std::ifstream f(paramfile);
	if (f.good())
		setFromFile(paramfile);
	else
	{
		RCLCPP_INFO(this->get_logger(), "set from rosparam, there is no paramfile : %s",
				 paramfile.c_str());
		setFromParam();
	}
	Eigen::Quaterniond dq(att_maneuver(dyaw_, dpitch_, droll_));
	dqx_ = dq.x();
	dqy_ = dq.y();
	dqz_ = dq.z();
	dqw_ = dq.w();

	RCLCPP_INFO(this->get_logger(), "cmdtype : %s", COMMAND_TYPE.at(type_).c_str());
	RCLCPP_INFO(this->get_logger(), "timeout : %lf", timeout_);
	RCLCPP_INFO(this->get_logger(), "position drx,  dry,    drz   : %lf, %lf, %lf",
			 drx_, dry_, drz_);
	RCLCPP_INFO(this->get_logger(), "attitude dyaw, dpitch, droll : %lf, %lf, %lf",
			 dyaw_, dpitch_, droll_);
	RCLCPP_INFO(this->get_logger(), "attitude dqx, dqy, dqz, dqw : %lf, %lf, %lf, %lf",
			 dqx_, dqy_, dqz_, dqw_);
	RCLCPP_INFO(this->get_logger(), "hit key \"x\" to cancel action.");
}

//------------------------------------------------------------------------------
// デストラクタ
CtlCommandClient::~CtlCommandClient() = default;

//------------------------------------------------------------------------------
// 制御目標設定ファイルによる設定
void CtlCommandClient::setFromFile(const std::string& filename)
{
	using namespace ib2_mss;
	size_t lineno(0);
	try
	{
		RCLCPP_INFO(this->get_logger(), "read paramfile : %s", filename.c_str());
		FileReader f(filename);
		type_    = commandType(f.string(lineno++));
		timeout_ = f.value(lineno++);
		drx_     = f.value(lineno++);
		dry_     = f.value(lineno++);
		drz_     = f.value(lineno++);
		dyaw_    = f.value(lineno++);
		dpitch_  = f.value(lineno++);
		droll_   = f.value(lineno++);
		if (lineno != f.lines())
			throw std::domain_error("too many lines");
	}
	catch (const std::exception& e)
	{
		std::string type("CtlCommandClient::setFromFile");
		auto msg(Log::errorFileLine(e.what(), type, filename, lineno));
		RCLCPP_INFO(this->get_logger(), "%s", msg.c_str());
		LOG_ERROR(msg);
		throw;
	}
}

//------------------------------------------------------------------------------
// rosparamによる設定
void CtlCommandClient::setFromParam()
{
	auto get_param = [this](const std::string& name, auto& value) {
		using T = std::decay_t<decltype(value)>;
		if (!this->has_parameter(name)) {
			this->declare_parameter<T>(name, value);
		}
		this->get_parameter(name, value);
	};

	int type(0);
	get_param("ctl_command.type"   , type);
	get_param("ctl_command.timeout", timeout_);
	get_param("ctl_command.drx"    , drx_);
	get_param("ctl_command.dry"    , dry_);
	get_param("ctl_command.drz"    , drz_);
	get_param("ctl_command.dyaw"   , dyaw_);
	get_param("ctl_command.dpitch" , dpitch_);
	get_param("ctl_command.droll"  , droll_);
	type_ = static_cast<uint8_t>(type);
}

//------------------------------------------------------------------------------
// プログラム実行
void CtlCommandClient::execute()
{
	waitServer();
	sendGoal();
	waitForResult();
}

//------------------------------------------------------------------------------
// アクションサーバー起動待ち
void CtlCommandClient::waitServer()
{
	RCLCPP_INFO(this->get_logger(), "Waiting for action server start");
	ac_->wait_for_action_server();
	RCLCPP_INFO(this->get_logger(), "action server started");
}

//------------------------------------------------------------------------------
// 制御目標の送信
void CtlCommandClient::sendGoal()
{
	Eigen::Quaterniond dq(att_maneuver(dyaw_, dpitch_, droll_));

	int32_t type(type_ == 8 ? ib2_msgs::msg::CtlStatusType::DOCK_WITHOUT_CORRECTION :
		static_cast<int32_t>(type_ * 10));
	auto goal = CtlCommandAction::Goal();

	goal.target.header.stamp = this->now();
	goal.target.header.frame_id =
	(type == ib2_msgs::msg::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET ? FRAME_ISS : FRAME_IB2);


	if (type == ib2_msgs::msg::CtlStatusType::MOVE_TO_RELATIVE_TARGET ||
		type == ib2_msgs::msg::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET)
	{
		goal.target.pose.position.x = drx_;
		goal.target.pose.position.y = dry_;
		goal.target.pose.position.z = drz_;

		goal.target.pose.orientation.x = dq.x();
		goal.target.pose.orientation.y = dq.y();
		goal.target.pose.orientation.z = dq.z();
		goal.target.pose.orientation.w = dq.w();
	}
	else
	{
		goal.target.pose.position.x = 0.;
		goal.target.pose.position.y = 0.;
		goal.target.pose.position.z = 0.;

		goal.target.pose.orientation.x = 0.;
		goal.target.pose.orientation.y = 0.;
		goal.target.pose.orientation.z = 0.;
		goal.target.pose.orientation.w = 1.;
	}

	goal.type.type = type;

	auto send_goal_options = rclcpp_action::Client<CtlCommandAction>::SendGoalOptions();
	send_goal_options.goal_response_callback =
		std::bind(&CtlCommandClient::goalResponseCb, this, std::placeholders::_1);
	send_goal_options.feedback_callback =
		std::bind(&CtlCommandClient::feedbackCb, this, std::placeholders::_1, std::placeholders::_2);
	send_goal_options.result_callback =
		std::bind(&CtlCommandClient::resultCb, this, std::placeholders::_1);

	ac_->async_send_goal(goal, send_goal_options);
}

//------------------------------------------------------------------------------
// アクション結果の待機
void CtlCommandClient::waitForResult()
{
	auto timeout = rclcpp::Duration::from_seconds(timeout_);
	auto begin = this->now();
	rclcpp::Rate rate(10);
	while (rclcpp::ok() && !done_)
	{
		rclcpp::spin_some(this->shared_from_this());
		if (kbhit() && getchar() == 'x')
		{
			cancel("user hit key x");
			break;
		}
		if (this->now() - begin > timeout)
		{
			cancel("timeout");
			break;
		}
		rate.sleep();
	}
}

//------------------------------------------------------------------------------
// アクションのキャンセル
void CtlCommandClient::cancel(const std::string& info)
{
	RCLCPP_INFO(this->get_logger(), "Action Canceled by %s.", info.c_str());
	if (goal_handle_) {
		ac_->async_cancel_goal(goal_handle_);
	}
}

//------------------------------------------------------------------------------
// ゴール応答時の処理
void CtlCommandClient::goalResponseCb(GoalHandle::SharedPtr goal_handle)
{
	if (!goal_handle) {
		RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
		done_ = true;
		return;
	}
	RCLCPP_INFO(this->get_logger(), "Goal just went active");
	goal_handle_ = goal_handle;
}

//------------------------------------------------------------------------------
// アクションのフィードバック受信時の処理
void CtlCommandClient::feedbackCb(
	GoalHandle::SharedPtr,
	const std::shared_ptr<const CtlCommandAction::Feedback> feedback)
{
	using namespace ib2_mss;

	double dt_sec = rclcpp::Duration(feedback->time_to_go).seconds();
	auto& dr(feedback->pose_to_go.position);
	auto& dq(feedback->pose_to_go.orientation);

	double rot(2. * acos(dq.w) / DEG);
	if (dqx_ * dq.x + dqy_ * dq.y + dqz_ * dq.z < 0.)
		rot *= -1.;

	RCLCPP_INFO(this->get_logger(), "Got Feedback time to goal %.3f [s]", dt_sec);
	RCLCPP_INFO(this->get_logger(), "Got Feedback distance(x) to goal %.3f [m]", dr.x);
	RCLCPP_INFO(this->get_logger(), "Got Feedback distance(y) to goal %.3f [m]", dr.y);
	RCLCPP_INFO(this->get_logger(), "Got Feedback distance(z) to goal %.3f [m]", dr.z);
	RCLCPP_INFO(this->get_logger(), "Got Feedback rotation to goal %.3f [deg]", rot);
	RCLCPP_INFO(this->get_logger(), "Got Feedback orientation(x) to goal %.6f", dq.x);
	RCLCPP_INFO(this->get_logger(), "Got Feedback orientation(y) to goal %.6f", dq.y);
	RCLCPP_INFO(this->get_logger(), "Got Feedback orientation(z) to goal %.6f", dq.z);
	RCLCPP_INFO(this->get_logger(), "Got Feedback orientation(w) to goal %.6f", dq.w);
}

//------------------------------------------------------------------------------
// アクション結果受信時の処理
void CtlCommandClient::resultCb(const GoalHandle::WrappedResult& result)
{
	std::string state_str;
	switch (result.code) {
		case rclcpp_action::ResultCode::SUCCEEDED:
			state_str = "SUCCEEDED";
			break;
		case rclcpp_action::ResultCode::ABORTED:
			state_str = "ABORTED";
			break;
		case rclcpp_action::ResultCode::CANCELED:
			state_str = "CANCELED";
			break;
		default:
			state_str = "UNKNOWN";
			break;
	}

	RCLCPP_INFO(this->get_logger(), "Finished in state [%s]", state_str.c_str());

	double stamp_sec = rclcpp::Time(result.result->stamp).seconds();
	RCLCPP_INFO(this->get_logger(), "Finished at %.3f (Simulation Time)", stamp_sec);

	static const std::vector<std::string> RESULT_STRINGS = {
		"SUCCESS", "ABORTED", "TIME_OUT", "INVALID_NAV", "INVALID_CMD"
	};
	auto rstr = RESULT_STRINGS[result.result->type];
	RCLCPP_INFO(this->get_logger(), "Finished in type  [%d : %s]", result.result->type, rstr.c_str());

	done_ = true;
}

//------------------------------------------------------------------------------
// メイン関数
int main(int argc, char** argv)
{
	using namespace ib2_mss;

	rclcpp::init(argc, argv);
	try
	{
		auto node = std::make_shared<CtlCommandClient>();
		node->execute();
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
	}
	catch (...)
	{
		LOG_ERROR("caught unknown exception");
	}
	rclcpp::shutdown();
	return 0;
}

// End Of File -----------------------------------------------------------------
