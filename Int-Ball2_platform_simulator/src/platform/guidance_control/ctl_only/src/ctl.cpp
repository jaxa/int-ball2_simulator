
#include "ctl/ctl.h"
#include "ctl/ctl_body.h"
#include "ctl/pos_controller.h"
#include "ctl/att_controller.h"
#include "ctl/pos_att_controller.h"
#include "ctl/pos_profiler.h"
#include "ctl/att_profiler.h"
#include "ctl/pos_att_profiler.h"
#include "ib2_msgs/MarkerCorrection.h"

#include "guidance_control_common/Log.h"
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/Utility.h"
#include "guidance_control_common/RangeChecker.h"

#include <Eigen/Core>

#include <sstream>
#include <stdexcept>
#include <cmath>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	/** 制御目標アクションの名前 */
	const std::string COMMAND_ACTION("/ctl/command");
	
	/** パラメータ更新サービスの名前 */
	const std::string UPDATE_SERVICE("/ctl/update_params");
}

//------------------------------------------------------------------------------
// コンストラクタ
Ctl::Ctl(const ros::NodeHandle& nh) :
	nh_(nh),
	command_as_(nh_, COMMAND_ACTION, boost::bind(&Ctl::commandCallback,this,_1), false),
	dtc_(nh),seq_status_(0),valid_navigation_(false)
{
    ROS_INFO("******** Starting Ctl Node");
	
	status_ = ib2_msgs::CtlStatusType::STAND_BY;
	setMember();

	// Action server
    command_as_.start();
	
	// Service server
	update_ss_ = nh_.advertiseService(UPDATE_SERVICE, &Ctl::updateCallback, this);
	
	// Service client
	marker_sc_ = nh_.serviceClient<ib2_msgs::MarkerCorrection>("/sensor_fusion/marker_correction");

    // Subscribers
	navinfo_sub_ = nh_.subscribe(TOPIC_NAV_POSE, 5, &Ctl::navinfoCallback, this);

    // Advertised messages
	wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>(TOPIC_CTL_WRENCH, 5);

	// Advertised messages
	profile_pub_ = nh_.advertise<ib2_msgs::CtlProfile>(TOPIC_CTL_PROFILE, 5);
	status_pub_  = nh_.advertise<ib2_msgs::CtlStatus> (TOPIC_CTL_STATUS,  5);

	timer_ = nh_.createTimer(interval_status_, &Ctl::timerCallback, this);
}

//------------------------------------------------------------------------------
// デストラクタ
Ctl::~Ctl() = default;

//------------------------------------------------------------------------------
// メンバ設定
bool Ctl::setMember()
{
	using namespace ib2_mss;
	// TODO : メンバを保存
	try 
	{
		static const RangeCheckerD INTERVAL_RANGE
		(RangeCheckerD::TYPE::GE, 0.1, true);
		
		ib2::CtlBody body(nh_);
		ib2::PosController ctl_pos(nh_);
		ib2::AttController ctl_att(nh_);
		ib2::PosProfiler prof_pos(nh_);
		ib2::AttProfiler prof_att(nh_);
		ib2::ThrustAllocator thr(nh_);
		
		static const std::string ROSPARAM_INTERVAL_STATUS   ("/ctl/interval_status");
		static const std::string ROSPARAM_INTERVAL_FEEDBACK ("/ctl/interval_feedback");
		static const std::string ROSPARAM_DURATION_GOAL     ("/ctl/duration_goal");
		static const std::string ROSPARAM_TOLERANCE_POS     ("/ctl/tolerance_pos");
		static const std::string ROSPARAM_TOLERANCE_ATT     ("/ctl/tolerance_att");
		static const std::string ROSPARAM_TOLERANCE_POS_STOP("/ctl/tolerance_pos_stop");
		static const std::string ROSPARAM_TOLERANCE_ATT_STOP("/ctl/tolerance_att_stop");
		static const std::string ROSPARAM_WAIT_CANCEL       ("/ctl/wait_cancel");
		static const std::string ROSPARAM_WAIT_RELEASE      ("/ctl/wait_release");
		static const std::string ROSPARAM_WAIT_CALIBRATION  ("/ctl/wait_calibration");
		static const std::string ROSPARAM_WAIT_DOCKING      ("/ctl/wait_docking");
		static const std::string ROSPARAM_NAV_COUNTER("/navigation_check/nc");
		static const std::string ROSPARAM_NAV_DR("/navigation_check/dr");
		static const std::string ROSPARAM_NAV_DV("/navigation_check/dv");
		static const std::string ROSPARAM_NAV_DA("/navigation_check/da");
		static const std::string ROSPARAM_NAV_DQ("/navigation_check/dq");
		static const std::string ROSPARAM_NAV_DW("/navigation_check/dw");

		double interval_status   (-1.);
		double interval_feedback (-1.);
		double duration_goal     (-1.);
		double tolerance_pos     (-1.);
		double tolerance_att     (-1.);
		double tolerance_pos_stop(-1.);
		double tolerance_att_stop(-1.);
		double wait_cancel       (-1.);
		double wait_release      (-1.);
		double wait_calibration  (-1.);
		double wait_docking      (-1.);
		int32_t nav_counter(-1);
		double nav_dr(-1.);
		double nav_dv(-1.);
		double nav_da(-1.);
		double nav_dq(-1.);
		double nav_dw(-1.);
		
		nh_.getParam(ROSPARAM_INTERVAL_STATUS   , interval_status);
		nh_.getParam(ROSPARAM_INTERVAL_FEEDBACK , interval_feedback);
		nh_.getParam(ROSPARAM_DURATION_GOAL     , duration_goal);
		nh_.getParam(ROSPARAM_TOLERANCE_POS     , tolerance_pos);
		nh_.getParam(ROSPARAM_TOLERANCE_ATT     , tolerance_att);
		nh_.getParam(ROSPARAM_TOLERANCE_POS_STOP, tolerance_pos_stop);
		nh_.getParam(ROSPARAM_TOLERANCE_ATT_STOP, tolerance_att_stop);
		nh_.getParam(ROSPARAM_WAIT_CANCEL       , wait_cancel);
		nh_.getParam(ROSPARAM_WAIT_RELEASE      , wait_release);
		nh_.getParam(ROSPARAM_WAIT_CALIBRATION  , wait_calibration);
		nh_.getParam(ROSPARAM_WAIT_DOCKING      , wait_docking);
		nh_.getParam(ROSPARAM_NAV_COUNTER       , nav_counter);
		nh_.getParam(ROSPARAM_NAV_DR            , nav_dr);
		nh_.getParam(ROSPARAM_NAV_DV            , nav_dv);
		nh_.getParam(ROSPARAM_NAV_DA            , nav_da);
		nh_.getParam(ROSPARAM_NAV_DQ            , nav_dq);
		nh_.getParam(ROSPARAM_NAV_DW            , nav_dw);
		
		INTERVAL_RANGE.valid(interval_status  , ROSPARAM_INTERVAL_STATUS);
		INTERVAL_RANGE.valid(interval_feedback, ROSPARAM_INTERVAL_FEEDBACK);
		RangeCheckerD::positive(duration_goal, true, ROSPARAM_DURATION_GOAL);
		RangeCheckerD::positive(tolerance_pos, true, ROSPARAM_TOLERANCE_POS);
		RangeCheckerD::positive(tolerance_att, true, ROSPARAM_TOLERANCE_ATT);
		RangeCheckerD::positive(tolerance_pos_stop, true, ROSPARAM_TOLERANCE_POS_STOP);
		RangeCheckerD::positive(tolerance_att_stop, true, ROSPARAM_TOLERANCE_ATT_STOP);
		RangeCheckerD::notNegative(wait_cancel     , true, ROSPARAM_WAIT_CANCEL);
		RangeCheckerD::notNegative(wait_release    , true, ROSPARAM_WAIT_RELEASE);
		RangeCheckerD::notNegative(wait_calibration, true, ROSPARAM_WAIT_CALIBRATION);
		RangeCheckerD::notNegative(wait_docking    , true, ROSPARAM_WAIT_DOCKING);
		RangeCheckerI32::positive(nav_counter, true, ROSPARAM_NAV_COUNTER);
		RangeCheckerD::positive(nav_dr, true, ROSPARAM_NAV_DR);
		RangeCheckerD::positive(nav_dv, true, ROSPARAM_NAV_DV);
		RangeCheckerD::positive(nav_da, true, ROSPARAM_NAV_DA);
		RangeCheckerD::positive(nav_dq, true, ROSPARAM_NAV_DQ);
		RangeCheckerD::positive(nav_dw, true, ROSPARAM_NAV_DW);

		dtc_.setMember();

		interval_status_    = ros::Duration(interval_status);
		interval_feedback_  = ros::Duration(interval_feedback);
		duration_goal_      = ros::Duration(duration_goal);
		tolerance_pos_      = tolerance_pos;
		tolerance_att_      = tolerance_att;
		tolerance_pos_stop_ = tolerance_pos_stop;
		tolerance_att_stop_ = tolerance_att_stop;
		waitCancel_         = ros::Duration(wait_cancel);
		waitRelease_        = ros::Duration(wait_release);
		waitCalibration_    = ros::Duration(wait_calibration);
		waitDocking_        = ros::Duration(wait_docking);
		nav_counter_ = static_cast<size_t>(nav_counter);
		nav_dr_ = nav_dr;
		nav_dv_ = nav_dv;
		nav_da_ = nav_da;
		nav_dq_ = nav_dq;
		nav_dw_ = nav_dw;

		ROS_INFO("******** Set Parameters in ctl.cpp");
		ROS_INFO("%s   : %u.%u", ROSPARAM_INTERVAL_STATUS.c_str()   , interval_status_.sec, interval_status_.nsec);
		ROS_INFO("%s   : %u.%u", ROSPARAM_INTERVAL_FEEDBACK.c_str() , interval_feedback_.sec, interval_feedback_.nsec);
		ROS_INFO("%s   : %u.%u", ROSPARAM_DURATION_GOAL.c_str()     , duration_goal_.sec, duration_goal_.nsec);
		ROS_INFO("%s   : %f", ROSPARAM_TOLERANCE_POS.c_str()        , tolerance_pos_);
		ROS_INFO("%s   : %f", ROSPARAM_TOLERANCE_ATT.c_str()        , tolerance_att_);
		ROS_INFO("%s   : %f", ROSPARAM_TOLERANCE_POS_STOP.c_str()   , tolerance_pos_stop_);
		ROS_INFO("%s   : %f", ROSPARAM_TOLERANCE_ATT_STOP.c_str()   , tolerance_att_stop_);
		ROS_INFO("%s   : %u.%u", ROSPARAM_WAIT_CANCEL.c_str()       , waitCancel_.sec, waitCancel_.nsec);
		ROS_INFO("%s   : %u.%u", ROSPARAM_WAIT_RELEASE.c_str()      , waitRelease_.sec, waitRelease_.nsec);
		ROS_INFO("%s   : %u.%u", ROSPARAM_WAIT_CALIBRATION.c_str()  , waitCalibration_.sec, waitCalibration_.nsec);
		ROS_INFO("%s   : %u.%u", ROSPARAM_WAIT_DOCKING.c_str()      , waitDocking_.sec, waitDocking_.nsec);
		ROS_INFO("%s   : %zd", ROSPARAM_NAV_COUNTER.c_str()         , nav_counter_);
		ROS_INFO("%s   : %f", ROSPARAM_NAV_DR.c_str()               , nav_dr_);
		ROS_INFO("%s   : %f", ROSPARAM_NAV_DV.c_str()               , nav_dv_);
		ROS_INFO("%s   : %f", ROSPARAM_NAV_DA.c_str()               , nav_da_);
		ROS_INFO("%s   : %f", ROSPARAM_NAV_DQ.c_str()               , nav_dq_);
		ROS_INFO("%s   : %f", ROSPARAM_NAV_DW.c_str()               , nav_dw_);

		if (timer_.isValid())
			timer_.setPeriod(interval_status_);
		
		if (controller_)
		{
			controller_->setConfigPos(ctl_pos);
			controller_->setConfigAtt(ctl_att);
		}
		else
			controller_.reset(new ib2::PosAttController(ctl_pos, ctl_att));
		if (profiler_)
		{
			profiler_->setConfigPos(prof_pos);
			profiler_->setConfigAtt(prof_att);
			profiler_->setConfigThr(thr);
		}
		else
			profiler_.reset(new ib2::PosAttProfiler(prof_pos, prof_att, thr));
		// Modification for platform packages
		//if (fsm_)s
		//	fsm_->setMember(thr);
		//else
		//	fsm_ = std::unique_ptr<Fsm>(new Fsm(nh_));

		body_.reset(new ib2::CtlBody(body));
		return true;
	}
	catch (const std::exception& e) 
	{
		std::string what(Log::caughtException(e.what()));
		ROS_ERROR("%s", what.c_str());
		return false;
	}
	catch (...) 
	{
		return false;
	}
}

//------------------------------------------------------------------------------
// 位置姿勢保持設定
void Ctl::setKeepPose()
{
	controller_->flash();
	auto profmsg(profiler_->setProfile(last_nav_stamp_));
	profile_pub_.publish(profmsg);
}

//------------------------------------------------------------------------------
// 制御目標への誘導
bool Ctl::guidance(int32_t goal_type, double tolp, double tola)
{
	auto tnav (last_nav_stamp_.pose.header.stamp);
	auto tfb = tnav;
	auto tbGoal = tnav;
	bool stayGoal(false);
	while (true)
	{
		if (!command_as_.isActive())
			break;
		tnav = last_nav_stamp_.pose.header.stamp;
		if (ros::Time::now() - tnav >= waitCancel_)
		{
			timeoutNavigation();
			break;
		}
		auto fb = profiler_->statesToGoal(last_nav_stamp_);
		if (tnav - tfb >= interval_feedback_)
		{
			command_as_.publishFeedback(fb);
			tfb = ros::Time::now();
		}
		if (goal_type == ib2_msgs::CtlStatusType::SCAN ? 
			reachGoalScan(stayGoal, tbGoal, tnav, fb, tola):
			reachGoal(stayGoal, tbGoal, tnav, fb, tolp, tola))
			return true;
		if (dtc_.status() == Dtc::DETECT::COLLISION)
		{
			cancelTarget(status_ == ib2_msgs::CtlStatusType::MOVING_TO_RDP ||
						 status_ == ib2_msgs::CtlStatusType::RELEASE);
			abortAction(ib2_msgs::CtlCommandResult::TERMINATE_ABORTED);
			break;
		}
		if (command_as_.isPreemptRequested() || !ros::ok())
		{
			abortAction(ib2_msgs::CtlCommandResult::TERMINATE_ABORTED);
			if (command_as_.isActive())
				status_ = ib2_msgs::CtlStatusType::STAND_BY;
			else if (goal_type == ib2_msgs::CtlStatusType::STOP_MOVING)
				setKeepPose();
			else 
			{
				cancelTarget(status_ == ib2_msgs::CtlStatusType::MOVING_TO_RDP ||
							 status_ == ib2_msgs::CtlStatusType::RELEASE);
			}
			break;
		}
	}
	return false;
}

//------------------------------------------------------------------------------
// ターゲットモードの処理
void Ctl::target(const ib2_msgs::CtlCommandGoalConstPtr& goal)
{
	status_ = goal->type.type;

	auto profmsg(profiler_->setProfile(last_nav_stamp_, goal, *body_));
	profile_pub_.publish(profmsg);
	controller_->flash();
	
	if (guidance(goal->type.type, tolerance_pos_, tolerance_att_))
		goalTarget();
	if (dtc_.status() == Dtc::DETECT::COLLISION)
	{
		dtc_.clearStatus();
		status_ = ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION;
	}
	else if (status_ != ib2_msgs::CtlStatusType::STAND_BY)
		status_ = ib2_msgs::CtlStatusType::KEEP_POSE;
}

//------------------------------------------------------------------------------
// リリースモードの処理
void Ctl::release()
{
	if (status_ != ib2_msgs::CtlStatusType::STAND_BY)
		abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD);
	else
	{
		setKeepPose();
		controller_->flash();
		auto tnav(last_nav_stamp_.pose.header.stamp);
		auto tfb(tnav);
		auto toff(tnav + waitRelease_);
		while (tnav < toff)
		{
			tnav = last_nav_stamp_.pose.header.stamp;
			auto fb = profiler_->statesToGoal(last_nav_stamp_);
			if (tnav - tfb >= interval_feedback_)
			{
				command_as_.publishFeedback(fb);
				tfb = tnav;
			}
			if (command_as_.isPreemptRequested() || !ros::ok())
			{
				abortAction(ib2_msgs::CtlCommandResult::TERMINATE_ABORTED);
				status_ = ib2_msgs::CtlStatusType::STAND_BY;
				return;
			}
		}

		status_ = ib2_msgs::CtlStatusType::RELEASE;
		auto ipos(ib2::PosAttProfiler::DOCKING_POS::AIP);
		auto iatt(ib2::PosAttProfiler::DOCKING_ATT::RDA);
		auto profmsg(profiler_->dockingProfile(last_nav_stamp_, ipos, iatt, *body_));
		profile_pub_.publish(profmsg);
		controller_->flash();
		if (guidance(ib2_msgs::CtlStatusType::RELEASE, tolerance_pos_, tolerance_att_))
			goalTarget();
		if (status_ != ib2_msgs::CtlStatusType::STAND_BY)
			status_  = ib2_msgs::CtlStatusType::KEEP_POSE;
	}
}

//------------------------------------------------------------------------------
// ホーミングモードの処理
void Ctl::docking(bool correction)
{
	bool goaled(false);
	int32_t start_status_ = (correction ? ib2_msgs::CtlStatusType::MOVING_TO_AIA_AIP : 
										  ib2_msgs::CtlStatusType::MOVING_TO_RDA_AIP);
	for (status_  = start_status_; 
		 status_ <= ib2_msgs::CtlStatusType::MOVING_TO_RDP; ++status_)
	{
		auto ipos(status_ < ib2_msgs::CtlStatusType::MOVING_TO_RDP ?
				  ib2::PosAttProfiler::DOCKING_POS::AIP : 
				  ib2::PosAttProfiler::DOCKING_POS::RDP);
		auto iatt(status_ < ib2_msgs::CtlStatusType::MOVING_TO_RDA_AIP ? 
				  ib2::PosAttProfiler::DOCKING_ATT::AIA : 
				  ib2::PosAttProfiler::DOCKING_ATT::RDA);
		auto profmsg(profiler_->dockingProfile(last_nav_stamp_, ipos, iatt, *body_));
		profile_pub_.publish(profmsg);
		controller_->flash();
		
		if (!guidance(ib2_msgs::CtlStatusType::DOCK, 
					  tolerance_pos_stop_, tolerance_att_stop_))
			break;
		else if (status_ == ib2_msgs::CtlStatusType::MOVING_TO_AIA_AIP)
		{
			ib2_msgs::MarkerCorrection srv;
			bool corrected(marker_sc_.call(srv));
			if (corrected)
				corrected = (srv.response.status == 
							 ib2_msgs::MarkerCorrection::Response::SUCCESS);
			if (!corrected)
			{
				abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV);
				break;
			}
		}
		else if (status_ == ib2_msgs::CtlStatusType::MOVING_TO_RDP)
			goaled = true;
		if (reachGoalDock())
			return;
	}
	setKeepPose();
	controller_->flash();
	if (goaled)
	{
		status_ = ib2_msgs::CtlStatusType::DOCKING_STAND_BY;
		dockingStandBy();
	}
	else if (dtc_.status() == Dtc::DETECT::COLLISION)
	{
		dtc_.clearStatus();
		status_ = ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION;
	}
	else if (status_ != ib2_msgs::CtlStatusType::STAND_BY)
		status_ = ib2_msgs::CtlStatusType::KEEP_POSE;
}

//------------------------------------------------------------------------------
// ドッキングモードの処理
void Ctl::dockingStandBy()
{
	auto tnav(last_nav_stamp_.pose.header.stamp);
	auto tfb(tnav);
	auto toff(tnav + waitDocking_);

	controller_->flash();
	auto profmsg(profiler_->setProfile(dtc_.dockingTarget(tnav)));
	profile_pub_.publish(profmsg);

	bool aborted(false);
	while (tnav < toff)
	{
		tnav = last_nav_stamp_.pose.header.stamp;
		auto fb = profiler_->statesToGoal(last_nav_stamp_);
		if (tnav - tfb >= interval_feedback_)
		{
			command_as_.publishFeedback(fb);
			tfb = tnav;
		}
		if (reachGoalDock())
			return;
		if (command_as_.isPreemptRequested() || !ros::ok())
		{
			aborted = true;
			break;
		}
	}
	status_ = ib2_msgs::CtlStatusType::STAND_BY;
	abortAction(aborted ? ib2_msgs::CtlCommandResult::TERMINATE_ABORTED :
				ib2_msgs::CtlCommandResult::TERMINATE_TIME_OUT);
}

//------------------------------------------------------------------------------
// スキャンモードの処理
void Ctl::scan()
{
	status_ = ib2_msgs::CtlStatusType::SCAN;
	size_t nscan(profiler_->nscan());
	for (size_t i = 0; i < nscan; ++i)
	{
		auto profmsg(profiler_->scanProfile(last_nav_stamp_, i, *body_));
		profile_pub_.publish(profmsg);
		controller_->flash();
		
		if (!guidance(ib2_msgs::CtlStatusType::SCAN, 
					  tolerance_pos_stop_, tolerance_att_stop_))
		{
			status_ = ib2_msgs::CtlStatusType::KEEP_POSE;
			break;
		}
		else if (i + 1 == nscan)
		{
			ib2_msgs::CtlCommandResult r;
			r.stamp = ros::Time::now();
			r.type = ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV;
			command_as_.setSucceeded(r);
			status_ = ib2_msgs::CtlStatusType::STAND_BY;
		}
	}
}

//------------------------------------------------------------------------------
// 停止誘導モードの処理
void Ctl::stopping()
{
	status_ = ib2_msgs::CtlStatusType::STOP_MOVING;
	auto profmsg(profiler_->stoppingProfile(last_nav_stamp_, *body_));
	profile_pub_.publish(profmsg);
	controller_->flash();
	if (guidance(ib2_msgs::CtlStatusType::STOP_MOVING, 
				 tolerance_pos_stop_, tolerance_att_stop_))
		goalTarget();
	status_ = ib2_msgs::CtlStatusType::KEEP_POSE;
}

//------------------------------------------------------------------------------
// アクション中止
void Ctl::abortAction(uint8_t reult_type)
{
	ib2_msgs::CtlCommandResult r;
	r.stamp = ros::Time::now();
	r.type = reult_type;
	if (command_as_.isActive())
		command_as_.setPreempted(r);
}

//------------------------------------------------------------------------------
// 制御目標キャンセル時の処理
void Ctl::cancelTarget(bool docking)
{
	ROS_INFO("%s: Preempted", COMMAND_ACTION.c_str());
	status_ = ib2_msgs::CtlStatusType::STOP_MOVING;
	auto profmsg(profiler_->stoppingProfile(last_nav_stamp_, *body_));
	profile_pub_.publish(profmsg);
	controller_->flash();

	auto tnav = last_nav_stamp_.pose.header.stamp;
	auto tbGoal = tnav;
	bool stayGoal(false);
	auto te(profiler_->te() + waitCancel_);
	while (tnav < te)
	{
		tnav = last_nav_stamp_.pose.header.stamp;
		if (ros::Time::now() - tnav >= waitCancel_)
		{
			timeoutNavigation();
			break;
		}
		auto fb = profiler_->statesToGoal(last_nav_stamp_);
		if (reachGoal(stayGoal, tbGoal, tnav, fb,
					  tolerance_pos_stop_, tolerance_att_stop_))
			break;
	}

	if (docking)
	{
		status_ = ib2_msgs::CtlStatusType::MOVING_TO_RDA_AIP;
		auto ipos(ib2::PosAttProfiler::DOCKING_POS::AIP);
		auto iatt(ib2::PosAttProfiler::DOCKING_ATT::RDA);
		auto profmsg(profiler_->dockingProfile
					 (last_nav_stamp_, ipos, iatt, *body_));
		profile_pub_.publish(profmsg);
		controller_->flash();

		auto tnav = last_nav_stamp_.pose.header.stamp;
		auto tbGoal = tnav;
		bool stayGoal(false);
		auto te(profiler_->te() + waitCancel_);
		while (tnav < te)
		{
			tnav = last_nav_stamp_.pose.header.stamp;
			if (ros::Time::now() - tnav >= waitCancel_)
			{
				timeoutNavigation();
				break;
			}
			auto fb = profiler_->statesToGoal(last_nav_stamp_);
			if (reachGoal(stayGoal, tbGoal, tnav, fb,
						  tolerance_pos_stop_, tolerance_att_stop_))
				break;
		}
	}
	controller_->flash();
}

//------------------------------------------------------------------------------
// 制御目標到達時の処理
void Ctl::goalTarget()
{
	ROS_INFO("%s: Succeeded", COMMAND_ACTION.c_str());
	ib2_msgs::CtlCommandResult r;
	r.stamp = ros::Time::now();
	r.type = ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS;
	command_as_.setSucceeded(r);
	controller_->flash();
}

//------------------------------------------------------------------------------
// 航法メッセージのタイムアウト処理
void Ctl::timeoutNavigation()
{
	ROS_WARN("Navigation message timed out");

	abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV);
	setKeepPose();
	status_ = ib2_msgs::CtlStatusType::STAND_BY;

	auto wrench = controller_->wrenchCommandStop(ros::Time::now());
	//fsm_->subscribeCommand(wrench);    // Modification for platform packages
	wrench_pub_.publish(wrench);
}

//------------------------------------------------------------------------------
// 制御目標到達判定
bool Ctl::reachGoal
(bool& stay, ros::Time& tin, const ros::Time& tnav,
 const ib2_msgs::CtlCommandFeedback& fb, double tolp, double tola)
{
	using namespace ib2_mss;
	auto& rgo(fb.pose_to_go.position);
	double rmax(std::max(std::abs(rgo.x), std::abs(rgo.y)));
	rmax = std::max(rmax, std::abs(rgo.z));
	double dx(rgo.x / rmax);
	double dy(rgo.y / rmax);
	double dz(rgo.z / rmax);
	double dgo(rmax * sqrt(dx * dx + dy * dy + dz * dz));
	double qgo(2. * acos(fb.pose_to_go.orientation.w));
	qgo = Utility::cyclicRange(qgo, -M_PI, M_PI);

	if (std::abs(dgo) < tolp && std::abs(qgo) < tola)
	{
		if (!stay)
			tin  = tnav;
		stay = true;
	}
	else if (stay)
		stay = false;

	return stay && tnav - tin >= duration_goal_;
}

//------------------------------------------------------------------------------
// 制御目標到達判定(SCAN)
bool Ctl::reachGoalScan
(bool& stay, ros::Time& tin, const ros::Time& tnav,
 const ib2_msgs::CtlCommandFeedback& fb, double tola)
{
	using namespace ib2_mss;
	double qgo(2. * acos(fb.pose_to_go.orientation.w));
	qgo = Utility::cyclicRange(qgo, -M_PI, M_PI);

	if (std::abs(qgo) < tola)
	{
		if (!stay)
			tin  = tnav;
		stay = true;
	}
	else if (stay)
		stay = false;

	return stay && tnav - tin >= duration_goal_;
}

//------------------------------------------------------------------------------
// 制御目標到達判定(DOCK)
bool Ctl::reachGoalDock()
{
	if (dtc_.status() != Dtc::DETECT::DOCKING)
		return false;

	goalTarget();
	status_ = ib2_msgs::CtlStatusType::STAND_BY;
	dtc_.clearStatus();
	return true;
}

//------------------------------------------------------------------------------
// 制御目標妥当性確認
bool Ctl::validCommand(const ib2_msgs::CtlCommandGoalConstPtr& goal) const
{
	auto& drg(goal->target.pose.position);
	auto& dqg(goal->target.pose.orientation);
	if (std::isfinite(drg.x) && std::isfinite(drg.y) && std::isfinite(drg.z) &&
		std::isfinite(dqg.x) && std::isfinite(dqg.y) && std::isfinite(dqg.z) &&
		std::isfinite(dqg.w))
	{
		Eigen::Quaterniond dq(dqg.w, dqg.x, dqg.y, dqg.z);
		return ib2_mss::RangeCheckerD::positive(dq.norm(), false, "dq norm");
	}
	return false;
}

//------------------------------------------------------------------------------
// 航法メッセージ妥当性確認
bool Ctl::validNavigation(const ib2_msgs::Navigation& nav, bool first) const
{
	auto& tn(nav.pose.header.stamp);
	auto& rn(nav.pose.pose.position);
	auto& qn(nav.pose.pose.orientation);
	auto& vn(nav.twist.linear);
	auto& wn(nav.twist.angular);
	auto& an(nav.a);
	if (std::isfinite(rn.x) && std::isfinite(rn.y) && std::isfinite(rn.z) &&
		std::isfinite(vn.x) && std::isfinite(vn.y) && std::isfinite(vn.z) &&
		std::isfinite(an.x) && std::isfinite(an.y) && std::isfinite(an.z) &&
		std::isfinite(wn.x) && std::isfinite(wn.y) && std::isfinite(wn.z) &&
		std::isfinite(qn.x) && std::isfinite(qn.y) && std::isfinite(qn.z) &&
		std::isfinite(qn.w))
	{
		Eigen::Quaterniond qne(qn.w, qn.x, qn.y, qn.z);
		if (!ib2_mss::RangeCheckerD::positive(qne.norm(), false, "qn norm"))
		{
			ROS_INFO("Invalid Quaternion : %f", qne.norm());
			return false;
		}
		qne.normalize();
		if (!first)
		{
			auto& tc(last_nav_stamp_.pose.header.stamp);
			if (tc >= tn)
			{
				ROS_INFO("Invalid Nav Stamp : current %u.%u, last %u.%u", tn.sec, tn.nsec, tc.sec, tc.nsec);
				return false;
			}
			double dt((tn - tc).toSec());
			double dr(nav_dr_ * dt);
			double dv(nav_dv_ * dt);
			double da(nav_da_ * dt);
			double dq(nav_dq_ * dt);
			double dw(nav_dw_ * dt);
			
			auto& rc(last_nav_stamp_.pose.pose.position);
			auto& qc(last_nav_stamp_.pose.pose.orientation);
			auto& vc(last_nav_stamp_.twist.linear);
			auto& wc(last_nav_stamp_.twist.angular);
			auto& ac(last_nav_stamp_.a);
			Eigen::Quaterniond qce(qc.w, qc.x, qc.y, qc.z);
			qce.normalize();

			Eigen::Quaterniond dqe(qne.conjugate() * qce);
			double dthe = std::acos(dqe.w()) * 2.;
			if(M_PI < dthe)
				dthe = 2. * M_PI - dthe;

			if (std::abs(rn.x - rc.x) > dr ||
				std::abs(rn.y - rc.y) > dr ||
				std::abs(rn.z - rc.z) > dr ||
				std::abs(vn.x - vc.x) > dv ||
				std::abs(vn.y - vc.y) > dv ||
				std::abs(vn.z - vc.z) > dv ||
				std::abs(an.x - ac.x) > da ||
				std::abs(an.y - ac.y) > da ||
				std::abs(an.z - ac.z) > da ||
				std::abs(wn.x - wc.x) > dw ||
				std::abs(wn.y - wc.y) > dw ||
				std::abs(wn.z - wc.z) > dw ||
				dthe                  > dq)
			{
				ROS_INFO("Invalid Navigation(Current - Last)");
				ROS_INFO("current pos : %f, %f, %f",     rn.x, rn.y, rn.z);
				ROS_INFO("current vel : %f, %f, %f",     vn.x, vn.y, vn.z);
				ROS_INFO("current acc : %f, %f, %f",     an.x, an.y, an.z);
				ROS_INFO("current w   : %f, %f, %f",     wn.x, wn.y, wn.z);
				ROS_INFO("current qtn : %f, %f, %f, %f", qn.x, qn.y, qn.z, qn.w);
				ROS_INFO("last    pos : %f, %f, %f",     rc.x, rc.y, rc.z);
				ROS_INFO("last    vel : %f, %f, %f",     vc.x, vc.y, vc.z);
				ROS_INFO("last    acc : %f, %f, %f",     ac.x, ac.y, ac.z);
				ROS_INFO("last    w   : %f, %f, %f",     wc.x, wc.y, wc.z);
				ROS_INFO("last    qtn : %f, %f, %f, %f", qc.x, qc.y, qc.z, qc.w);
				ROS_INFO("dthe        : %f",             dthe);
				return false;
			}
		}
		return true;
	}
	ROS_INFO("Invalid Navigation(NaN/infinite)");
	return false;
}

//------------------------------------------------------------------------------
// 制御目標アクション受信時の処理
void Ctl::commandCallback(const ib2_msgs::CtlCommandGoalConstPtr& goal)
{
	try 
	{
		auto& tcmd(goal->target.header.stamp);
		auto& tnav(last_nav_stamp_.pose.header.stamp);
		if (!valid_navigation_ || tcmd - tnav > interval_feedback_)
			throw std::domain_error("no valid navigation message");
		if (goal->type.type < ib2_msgs::CtlStatusType::STOP_MOVING)
		{
			status_ = (goal->type.type == ib2_msgs::CtlStatusType::STAND_BY ? 
					   ib2_msgs::CtlStatusType::STAND_BY : 
					   ib2_msgs::CtlStatusType::KEEP_POSE);
			setKeepPose();
			controller_->flash();
			goalTarget();
		}
		else if (goal->type.type == ib2_msgs::CtlStatusType::RELEASE)
			release();
		else if (goal->type.type == ib2_msgs::CtlStatusType::DOCK)
			docking(true);
		else if (goal->type.type == ib2_msgs::CtlStatusType::DOCK_WITHOUT_CORRECTION)
			docking(false);
		else if (goal->type.type == ib2_msgs::CtlStatusType::SCAN)
			scan();
		else if (goal->type.type == ib2_msgs::CtlStatusType::STOP_MOVING)
			target(goal);
		else if ((goal->type.type == ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET ||
				  goal->type.type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET) &&
				  validCommand(goal))
			target(goal);
		else
			abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD);
	}
	catch (const std::exception& e) 
	{
		std::string what(ib2_mss::Log::caughtException(e.what()));
		ROS_ERROR("%s", what.c_str());
		abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD);
		status_ = ib2_msgs::CtlStatusType::STAND_BY;
	}
	catch (...) 
	{
		ROS_ERROR("caught exception at Ctl::commandCallback");
		abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD);
		status_ = ib2_msgs::CtlStatusType::STAND_BY;
	}
}

//------------------------------------------------------------------------------
// Callback of update parameter service
bool Ctl::updateCallback
(ib2_msgs::UpdateParameter::Request&,
 ib2_msgs::UpdateParameter::Response& res)
{
	ROS_INFO("Update Parameters by /ctl/update_params");

	res.stamp = ros::Time::now();
	if (setMember())
	{
		ROS_INFO("%s: Succeeded", UPDATE_SERVICE.c_str());
		res.status = ib2_msgs::UpdateParameter::Response::SUCCESS;
	}
	else
	{
		ROS_INFO("%s: Failed", UPDATE_SERVICE.c_str());
		res.status = ib2_msgs::UpdateParameter::Response::FAILURE_UPDATE;
	}
	return true;
}

//------------------------------------------------------------------------------
// Callback of subscribe on the TOPIC_NAV_POSE
void Ctl::navinfoCallback(const ib2_msgs::Navigation& nav_stamp)
{
//	ROS_INFO_STREAM(nav_stamp);
	static size_t invalid_counter(0);
	try 
	{
		// 誘導制御計算実施
		static bool reset = true;
		if (!validNavigation(nav_stamp, reset))
		{
			++invalid_counter;
			if (invalid_counter < nav_counter_)
				return;
			invalid_counter = 0;
			if (command_as_.isActive())
				abortAction(ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV);
			setKeepPose();
			status_ = ib2_msgs::CtlStatusType::STAND_BY;
			valid_navigation_ = false;
		}
		else
		{
			invalid_counter = 0;
			last_nav_stamp_ = nav_stamp;
			valid_navigation_ = true;
		}
		if (reset)
		{
			setKeepPose();
			status_ = ib2_msgs::CtlStatusType::STAND_BY;
			reset = false;
			valid_navigation_ = true;
		}
		
		// 検知処理
		auto dtc_status(dtc_.detection(nav_stamp, status_));
		if (status_ == ib2_msgs::CtlStatusType::STAND_BY)
			dtc_.clearStatus();
		else if (dtc_status == Dtc::DETECT::DISTURBED && 
			status_ != ib2_msgs::CtlStatusType::RELEASE &&
			status_ != ib2_msgs::CtlStatusType::MOVING_TO_RDP &&
			status_ != ib2_msgs::CtlStatusType::DOCKING_STAND_BY)
			status_ = ib2_msgs::CtlStatusType::DISTURBED;
		else if (dtc_status == Dtc::DETECT::COLLISION)
		{
			if (status_ == ib2_msgs::CtlStatusType::RELEASE)
				dtc_.clearStatus();
			else if (!command_as_.isActive())
			{
				dtc_.clearStatus();
				setKeepPose();
				status_ = ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION;
			}
		}
		else if (dtc_status == Dtc::DETECT::CREW_CAPTURE)
		{
			if (command_as_.isActive())
				abortAction(ib2_msgs::CtlCommandResult::TERMINATE_ABORTED);
			status_ = ib2_msgs::CtlStatusType::CAPTURED;
		}
		else if (dtc_status == Dtc::DETECT::CREW_RELEASE)
		{
			dtc_.clearStatus();
			setKeepPose();
			status_ = ib2_msgs::CtlStatusType::KEEP_POSE;
		}

		// 制御停止判定
		static bool publishWrench = true;
		if (status_ < ib2_msgs::CtlStatusType::KEEP_POSE ||
			status_ == ib2_msgs::CtlStatusType::DOCKING_STAND_BY)
		{
			if (publishWrench)
			{
				auto wrench = controller_->wrenchCommandStop(nav_stamp.pose.header.stamp);
				//fsm_->subscribeCommand(wrench);    /// Modification for platform packages
				wrench_pub_.publish(wrench);
				publishWrench = false;
			}
		}
		else
		{
			publishWrench = true;
			
			// 位置姿勢誘導プロファイル
			auto p = profiler_->posAttProfile(nav_stamp.pose.header.stamp);
			
			// 力トルク
			geometry_msgs::WrenchStamped wrench
			(controller_->wrenchCommand(nav_stamp, p, *body_));
			if (status_ == ib2_msgs::CtlStatusType::SCAN)
			{
				wrench.wrench.force.x = 0.;
				wrench.wrench.force.y = 0.;
				wrench.wrench.force.z = 0.;
			}
			
			// publish
			//fsm_->subscribeCommand(wrench);    // Modification for platform packages
			wrench_pub_.publish(wrench);
		}
	}
	catch (const std::exception& e) 
	{
		std::string what(ib2_mss::Log::caughtException(e.what()));
		ROS_ERROR("%s", what.c_str());
	}
	catch (...) 
	{
		ROS_ERROR("caught exception at Ctl::navinfoCallback");
	}
}

//------------------------------------------------------------------------------
// Callback of publish on the TOPIC_NAV_POSE
void Ctl::timerCallback(const ros::TimerEvent& ev)
{
	try 
	{
		auto p = profiler_->posAttProfile(ev.current_real);
		auto msg(p.status(status_));
		msg.pose.header.seq = ++seq_status_;
		status_pub_.publish(msg);
	}
	catch (const std::exception& e) 
	{
		std::string what(ib2_mss::Log::caughtException(e.what()));
		ROS_ERROR("%s", what.c_str());
	}
	catch (...) 
	{
		ROS_ERROR("caught exception at Ctl::timerCallback");
	}
}

//------------------------------------------------------------------------------
// メイン関数
int main(int argc, char **argv)
{
    //ros::init(argc, argv, "ctl");    // Modification for platform packages
	ros::init(argc, argv, "ctl_only");
    ros::NodeHandle nh_;
    Ctl Ctl(nh_);
    ros::spin();
	return 0;
}
// End Of File -----------------------------------------------------------------
