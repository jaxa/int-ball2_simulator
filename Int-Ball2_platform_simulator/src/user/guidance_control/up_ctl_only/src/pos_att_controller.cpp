
#include "ctl/pos_att_controller.h"
#include "ctl/ctl_body.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::PosAttController::PosAttController() = default;

//------------------------------------------------------------------------------
// 値によるコンストラクタ
ib2::PosAttController::PosAttController
(const PosController& pos, const AttController& att) :
seq_(0), pos_(pos), att_(att)
{
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::PosAttController::~PosAttController() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::PosAttController::PosAttController(const PosAttController&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::PosAttController&
ib2::PosAttController::operator=(const PosAttController&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::PosAttController::PosAttController(PosAttController&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::PosAttController&
ib2::PosAttController::operator=(PosAttController&&) = default;

//------------------------------------------------------------------------------
// 位置制御パラメータの設定
bool ib2::PosAttController::setConfigPos(const PosController& pos)
{
	pos_ = pos;
	return true;
}

//------------------------------------------------------------------------------
// 姿勢制御パラメータの設定
bool ib2::PosAttController::setConfigAtt(const AttController& att)
{
	att_ = att;
	return true;
}

//------------------------------------------------------------------------------
// 積分量のクリア
void ib2::PosAttController::flash()
{
	pos_.flash();
}

//------------------------------------------------------------------------------
// 制御停止時の力トルクコマンドの計算
geometry_msgs::WrenchStamped ib2::PosAttController::wrenchCommandStop
(const ros::Time& t)
{
	geometry_msgs::WrenchStamped cmd;
	cmd.header.seq         = ++seq_;
	cmd.header.stamp       = t;
	cmd.header.frame_id    = "body";
	cmd.wrench.force.x     = 0.;
	cmd.wrench.force.y     = 0.;
	cmd.wrench.force.z     = 0.;
	cmd.wrench.torque.x    = 0.;
	cmd.wrench.torque.y    = 0.;
	cmd.wrench.torque.z    = 0.;
	return cmd;
}

//------------------------------------------------------------------------------
// 力トルクコマンドの計算
geometry_msgs::WrenchStamped ib2::PosAttController::wrenchCommand
(const ib2_msgs::Navigation& nav, const CtlElements &p, const ib2::CtlBody& b, const Eigen::Vector3d& torque_gain)
{
	// 航法値
	auto& tn(nav.pose.header.stamp);
	auto& rn(nav.pose.pose.position);
	auto& qn(nav.pose.pose.orientation);
	auto& vn(nav.twist.linear);
	auto& wn(nav.twist.angular);

	ros::Time t(tn);
	Eigen::Vector3d  r(rn.x, rn.y, rn.z);
	Eigen::Quaterniond q(qn.w, qn.x, qn.y, qn.z);
	Eigen::Vector3d  v(vn.x, vn.y, vn.z);
	Eigen::Vector3d  w(wn.x, wn.y, wn.z);

	// 力トルクコマンド計算
	Eigen::Vector3d force  = pos_.forceCommand(t, r, v, q, p, b.m());
	Eigen::Vector3d torque = att_.torqueCommand(q, w, p, b.Is());

	ROS_INFO_STREAM("      torque gain " << torque_gain(0) << ","<< torque_gain(1) << ","<< torque_gain(2));
	ROS_INFO_STREAM("      force " << force(0) << ","<< force(1) << ","<< force(2));
	ROS_INFO_STREAM("      torque_gain.cross(force) " << torque_gain.cross(force)(0) << ","<< torque_gain.cross(force)(1) << ","<< torque_gain.cross(force)(2));
	ROS_INFO_STREAM("  pre torque " << torque(0) << ","<< torque(1) << ","<< torque(2));
	torque = torque + torque_gain.cross(force);
	ROS_INFO_STREAM("      torque " << torque(0) << ","<< torque(1) << ","<< torque(2));
	
	geometry_msgs::WrenchStamped cmd;
	cmd.header.seq         = ++seq_;
	cmd.header.stamp       = tn;
	cmd.header.frame_id    = "body";
	cmd.wrench.force.x     = force.x();
	cmd.wrench.force.y     = force.y();
	cmd.wrench.force.z     = force.z();
	cmd.wrench.torque.x    = torque.x();
	cmd.wrench.torque.y    = torque.y();
	cmd.wrench.torque.z    = torque.z();
	return cmd;
}


// End Of File -----------------------------------------------------------------
