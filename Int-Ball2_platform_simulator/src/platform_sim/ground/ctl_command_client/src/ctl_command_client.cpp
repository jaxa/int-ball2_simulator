
#include "ctl_command_client/ctl_command_client.h"

#include "guidance_control_common/Constants.h"
#include "guidance_control_common/FileReader.h"
#include "guidance_control_common/Mjd.h"
#include "guidance_control_common/Log.h"

#include "ib2_msgs/CtlStatusType.h"
#include "ib2_msgs/Navigation.h"


#include "boost/date_time/posix_time/posix_time.hpp"

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
	nh_("~"), ac_(ACTION_NAME, true)
{
	std::string paramfile;
	nh_.getParam("paramfile", paramfile);
	std::ifstream f(paramfile);
	if (f.good())
		setFromFile(paramfile);
	else
	{
		ROS_INFO("set from rosparam, there is no paramfile : %s",
				 paramfile.c_str());
		setFromParam();
	}
	Eigen::Quaterniond dq(att_maneuver(dyaw_, dpitch_, droll_));
	dqx_ = dq.x();
	dqy_ = dq.y();
	dqz_ = dq.z();
	dqw_ = dq.w();

	ROS_INFO("cmdtype : %s", COMMAND_TYPE.at(type_).c_str());
	ROS_INFO("timeout : %lf", timeout_);
	ROS_INFO("position drx,  dry,    drz   : %lf, %lf, %lf",
			 drx_, dry_, drz_);
	ROS_INFO("attitude dyaw, dpitch, droll : %lf, %lf, %lf",
			 dyaw_, dpitch_, droll_);
	ROS_INFO("attitude dqx, dqy, dqz, dqw : %lf, %lf, %lf, %lf",
			 dqx_, dqy_, dqz_, dqw_);
	ROS_INFO("hit key \"x\" to cancel action.");
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
		ROS_INFO("read paramfile : %s", filename.c_str());
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
		ROS_INFO_STREAM(msg);
		LOG_ERROR(msg);
		throw;
	}
}

//------------------------------------------------------------------------------
// rosparamによる設定
void CtlCommandClient::setFromParam()
{
	// 事前にrosparam set /ctl_command/timeout XX などを設定すること
	int type(0);
	nh_.getParam("/ctl_command/type", type);
	nh_.getParam("/ctl_command/timeout", timeout_);
	nh_.getParam("/ctl_command/drx"    , drx_);
	nh_.getParam("/ctl_command/dry"    , dry_);
	nh_.getParam("/ctl_command/drz"    , drz_);
	nh_.getParam("/ctl_command/dyaw"   , dyaw_);
	nh_.getParam("/ctl_command/dpitch" , dpitch_);
	nh_.getParam("/ctl_command/droll"  , droll_);
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
	ROS_INFO("Waiting for action server start");
	ac_.waitForServer();
	ROS_INFO("action server started");
}

//------------------------------------------------------------------------------
// 制御目標の送信
void CtlCommandClient::sendGoal()
{
	static uint32_t seq(0);
	Eigen::Quaterniond dq(att_maneuver(dyaw_, dpitch_, droll_));

	int32_t type(type_ == 8 ? ib2_msgs::CtlStatusType::DOCK_WITHOUT_CORRECTION : 
		static_cast<int32_t>(type_ * 10));
	ib2_msgs::CtlCommandGoal goal;

	goal.target.header.seq = ++seq;
	goal.target.header.stamp = ros::Time::now();
	goal.target.header.frame_id = 
	(type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET ? FRAME_ISS : FRAME_IB2);


	if (type == ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET || 
		type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET)
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

	ac_.sendGoal(goal,
				 boost::bind(&CtlCommandClient::doneCb, this, _1, _2),
				 boost::bind(&CtlCommandClient::activeCb, this),
				 boost::bind(&CtlCommandClient::feedbackCb, this, _1));
}

//------------------------------------------------------------------------------
// アクション結果の待機
void CtlCommandClient::waitForResult()
{
	ros::Duration timeout(timeout_);
	ros::Time begin(ros::Time::now());
	while (true)
	{
		auto stateAC(ac_.getState());
		if (stateAC == actionlib::SimpleClientGoalState::REJECTED  ||
			stateAC == actionlib::SimpleClientGoalState::PREEMPTED ||
			stateAC == actionlib::SimpleClientGoalState::ABORTED   ||
			stateAC == actionlib::SimpleClientGoalState::SUCCEEDED ||
			stateAC == actionlib::SimpleClientGoalState::LOST)
			break;
		if (kbhit() && getchar() == 'x')
		{
			cancel("user hit key x");
			break;
		}
		if (ros::Time::now() - begin > timeout)
		{
			cancel("timeout");
			break;
		}
	}
}

//------------------------------------------------------------------------------
// アクションのキャンセル
void CtlCommandClient::cancel(const std::string& info)
{
	ROS_INFO("Action Canceled by %s.",info.c_str());
	ac_.cancelGoal();
}

//------------------------------------------------------------------------------
// アクション終了時の処理
void CtlCommandClient::doneCb
(const actionlib::SimpleClientGoalState& state,
 const ib2_msgs::CtlCommandResultConstPtr& result)
{
	ROS_INFO("Finished in state [%s]", state.toString().c_str());
	if (result->stamp.isSystemTime())
	{
		std::string tstr(boost::posix_time::to_iso_extended_string
						 (result->stamp.toBoost()));
		ROS_INFO("Finished at %s", tstr.c_str());
	}
	else
	{
		ROS_INFO("Finished at %.3f (Simulation Time)",
				 result->stamp.toSec());
	}
	static const std::vector<std::string> RESULT_STRINGS = {
		"SUCCESS", "ABORTED", "TIME_OUT", "INVALID_NAV", "INVALID_CMD"
	};
	auto rstr = RESULT_STRINGS[result->type];
	ROS_INFO("Finished in type  [%d : %s]", result->type, rstr.c_str());
}

//------------------------------------------------------------------------------
// アクション有効時の処理
void CtlCommandClient::activeCb()
{
	ROS_INFO("Goal just went active");
}

//------------------------------------------------------------------------------
// アクションのフィードバック受信時の処理
void CtlCommandClient::feedbackCb(const ib2_msgs::CtlCommandFeedbackConstPtr& feedback)
{
	using namespace ib2_mss;

	auto& dt(feedback->time_to_go);
	auto& dr(feedback->pose_to_go.position);
	auto& dq(feedback->pose_to_go.orientation);

	double rot(2. * acos(dq.w) / DEG);
	if (dqx_ * dq.x + dqy_ * dq.y + dqz_ * dq.z < 0.)
		rot *= -1.;

	ROS_INFO("Got Feedback time to goal %.3f [s]", dt.toSec());
	ROS_INFO("Got Feedback distance(x) to goal %.3f [m]", dr.x);
	ROS_INFO("Got Feedback distance(y) to goal %.3f [m]", dr.y);
	ROS_INFO("Got Feedback distance(z) to goal %.3f [m]", dr.z);
	ROS_INFO("Got Feedback rotation to goal %.3f [deg]", rot);
	ROS_INFO("Got Feedback orientation(x) to goal %.6f", dq.x);
	ROS_INFO("Got Feedback orientation(y) to goal %.6f", dq.y);
	ROS_INFO("Got Feedback orientation(z) to goal %.6f", dq.z);
	ROS_INFO("Got Feedback orientation(w) to goal %.6f", dq.w);
}

//------------------------------------------------------------------------------
// メイン関数
int main(int argc, char** argv)
{
	using namespace ib2_mss;
//	Log::configure("log/target_node.log", "INFO");

	ros::init(argc, argv, "target_node");
	try
	{
		CtlCommandClient o;
		o.execute();
		return 0;
	}
	catch (const std::exception& e)
	{
		LOG_ERROR(Log::caughtException(e.what()));
	}
	catch (...)
	{
		LOG_ERROR("caught unknown exception");
	}
	return 1;
}

// End Of File -----------------------------------------------------------------
