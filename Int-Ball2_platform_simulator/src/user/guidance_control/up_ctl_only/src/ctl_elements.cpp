
#include "ctl/ctl_elements.h"
#include "ib2_msgs/Navigation.h"


namespace
{
	const std::string FRAME_ISS("iss_body");
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
CtlElements::CtlElements() :
	t_(ros::Time(0.0)),
	r_(Eigen::Vector3d::Zero()), v_(Eigen::Vector3d::Zero()),
	a_(Eigen::Vector3d::Zero()), q_(Eigen::Quaterniond::Identity()),
	w_(Eigen::Vector3d::Zero())
{
}

//------------------------------------------------------------------------------
// 値によるコンストラクタ
CtlElements::CtlElements
(const ros::Time& t,
 const Eigen::Vector3d& r, const Eigen::Vector3d& v, const Eigen::Vector3d& a,
 const Eigen::Quaterniond& q, const Eigen::Vector3d& w) :
	t_(t), r_(r), v_(v), a_(a), q_(q), w_(w)
{
}

//------------------------------------------------------------------------------
// デストラクタ
CtlElements::~CtlElements() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
CtlElements::CtlElements(const CtlElements&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
CtlElements& CtlElements::operator=(const CtlElements&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
CtlElements::CtlElements(CtlElements&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
CtlElements& CtlElements::operator=(CtlElements&&) = default;

//------------------------------------------------------------------------------
// ROS時刻の参照
const ros::Time& CtlElements::t() const
{
	return t_;
}

//------------------------------------------------------------------------------
// 位置ベクトルの参照
const Eigen::Vector3d& CtlElements::r() const
{
	return r_;
}

//------------------------------------------------------------------------------
// 速度ベクトルの参照
const Eigen::Vector3d& CtlElements::v() const
{
	return v_;
}

//------------------------------------------------------------------------------
// 加速度ベクトルの参照
const Eigen::Vector3d& CtlElements::a() const
{
	return a_;
}

//------------------------------------------------------------------------------
// 姿勢クォータニオンの参照
const Eigen::Quaterniond& CtlElements::q() const
{
	return q_;
}

//------------------------------------------------------------------------------
// 角速度ベクトルの参照
const Eigen::Vector3d& CtlElements::w() const
{
	return w_;
}

//------------------------------------------------------------------------------
// 航法メッセージ形式での取得
ib2_msgs::CtlStatus CtlElements::status(int32_t s) const
{
	ib2_msgs::CtlStatus o;

	o.pose.header.stamp = t_;
	o.pose.header.frame_id = FRAME_ISS;

	o.pose.pose.position.x = r_.x();
	o.pose.pose.position.y = r_.y();
	o.pose.pose.position.z = r_.z();

	o.twist.linear.x = v_.x();
	o.twist.linear.y = v_.y();
	o.twist.linear.z = v_.z();

	Eigen::Vector3d ab(q_.conjugate() * a_);
	o.a.x = ab.x();
	o.a.y = ab.y();
	o.a.z = ab.z();

	o.pose.pose.orientation.x = q_.x();
	o.pose.pose.orientation.y = q_.y();
	o.pose.pose.orientation.z = q_.z();
	o.pose.pose.orientation.w = q_.w();

	o.twist.angular.x = w_.x();
	o.twist.angular.y = w_.y();
	o.twist.angular.z = w_.z();

	o.type.type = s;

	return o;
}

// End Of File -----------------------------------------------------------------
