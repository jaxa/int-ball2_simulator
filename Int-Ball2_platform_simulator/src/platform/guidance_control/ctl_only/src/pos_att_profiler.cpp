
#include "ctl/pos_att_profiler.h"
#include "ctl/ctl_body.h"
#include "ib2_msgs/Navigation.h"
#include "ib2_msgs/CtlStatusType.h"
#include "guidance_control_common/Constants.h"
#include "guidance_control_common/Utility.h"
#include "guidance_control_common/RangeChecker.h"

#include <sstream>

//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
	const std::string FRAME_ISS("iss_body");

	/** 移動量の計算
	 * @param [in] goal 制御目標メッセージ
	 * @return 位置移動量[m]
	 * @return 姿勢移動量[ND]
	 */
	std::pair<Eigen::Vector3d, Eigen::Quaterniond> maneuver
	(const ib2_msgs::CtlCommandGoalConstPtr& goal)
	{
		using namespace ib2_mss;
		auto& drg(goal->target.pose.position);
		auto& dqg(goal->target.pose.orientation);
		Eigen::Vector3d dr(drg.x, drg.y, drg.z);
		Eigen::Quaterniond dq(dqg.w, dqg.x, dqg.y, dqg.z);
		RangeCheckerD::positive(dq.norm(), true, "dq norm");
		dq.normalize();
		return std::make_pair(dr, dq);
	}

	/** 移動量の計算
	 * @param [in] goal 制御目標メッセージ
	 * @param [in] r0 制御開始位置
	 * @param [in] q0 制御開始姿勢
	 * @param [in] r1 制御目標位置
	 * @param [in] q1 制御目標姿勢
	 * @return 位置移動量[m]
	 * @return 姿勢移動量[ND]
	 */
	std::pair<Eigen::Vector3d, Eigen::Quaterniond> maneuver
	(const Eigen::Vector3d& r0, const Eigen::Quaterniond& q0,
	 const Eigen::Vector3d& r1, const Eigen::Quaterniond& q1)
	{
		Eigen::Quaterniond q0inv(q0.conjugate());
		Eigen::Vector3d dr = q0inv * (r1 - r0);
		Eigen::Quaterniond dq = q0inv * q1;
		if (dq.w() < 0.)
			dq.coeffs() *= -1.;
		return std::make_pair(dr, dq);
	}
	
	/** Bang-Bang制御時刻計算
	 * @param [in] r 移動量
	 * @param [in] v 最大速度
	 * @param [in] a 最大加速度
	 * @return クルージング開始時刻(移動開始からの経過秒)
	 * @return クルージング終了時刻(移動開始からの経過秒)
	 * @return 移動終了時刻(移動開始からの経過秒)
	 */
	std::tuple<double, double, double> maneuverTiming
	(double r, double v, double a)
	{
		double rnc(v * v / a);
		double tcb(r > rnc ? v / a : sqrt(r / a));
		double tce(r > rnc ? r / v : tcb);
		double te (tcb + tce);
		return std::make_tuple(tcb, tce, te);
	}

	/** プロファイル計算
	 * @param [in] t 移動開始からの経過秒
	 * @param [in] tcb クルージング開始時刻(移動開始からの経過秒)
	 * @param [in] tce クルージング終了時刻(移動開始からの経過秒)
	 * @param [in] te  移動終了時刻(移動開始からの経過秒)
	 * @param [in] thr 移動量閾値
	 * @param [in] r 移動量
	 * @param [in] v 最大速度
	 * @param [in] a 最大加速度
	 * @return 移動量
	 * @return 速度
	 * @return 加速度
	 */
	std::tuple<double, double, double> target
	(double t, double tcb, double tce, double te, double thr,
	 double r, double v, double a)
	{
		if (r < thr)
			return std::make_tuple(r, 0., 0.);

		if (t < 0.)
			return std::make_tuple(0., 0., 0.);

		if (t < tcb)
		{
			double vp(a * t);
			return std::make_tuple(vp * t * 0.5, vp, a);
		}
		if (t < tce)
			return std::make_tuple(v * (t - 0.5 * tcb), v, 0.);

		if (t < te)
		{
			double tg(te - t);
			double vp(a * tg);
			return std::make_tuple(r - vp * tg * 0.5, vp, -a);
		}
		return std::make_tuple(r, 0., 0.);
	}
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::PosAttProfiler::PosAttProfiler() = default;

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::PosAttProfiler::PosAttProfiler(const ros::NodeHandle& nh) : msg_seq_(0)
{
	setMember(nh);
}

//------------------------------------------------------------------------------
// 値によるコンストラクタ
ib2::PosAttProfiler::PosAttProfiler
(const ib2::PosProfiler& pos, const ib2::AttProfiler& att,
 const ib2::ThrustAllocator& thr) :
	pos_(pos), att_(att), thr_(thr), msg_seq_(0)
{
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::PosAttProfiler::~PosAttProfiler() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::PosAttProfiler::PosAttProfiler(const PosAttProfiler&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::PosAttProfiler&
ib2::PosAttProfiler::operator=(const PosAttProfiler&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::PosAttProfiler::PosAttProfiler(PosAttProfiler&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::PosAttProfiler& 
ib2::PosAttProfiler::operator=(PosAttProfiler&&) = default;

//------------------------------------------------------------------------------
// メンバ設定
bool ib2::PosAttProfiler::setMember(const ros::NodeHandle& nh)
{
	pos_ = ib2::PosProfiler(nh);
	att_ = ib2::AttProfiler(nh);
	thr_ = ib2::ThrustAllocator    (nh);
	return true;
}

//------------------------------------------------------------------------------
// 位置誘導プロファイルパラメータ設定
bool ib2::PosAttProfiler::setConfigPos(const ib2::PosProfiler& p)
{
	pos_ = p;
	return true;
}

//------------------------------------------------------------------------------
// 姿勢誘導プロファイルパラメータ設定
bool ib2::PosAttProfiler::setConfigAtt(const ib2::AttProfiler& p)
{
	att_ = p;
	return true;
}

//------------------------------------------------------------------------------
// 姿勢誘導プロファイルパラメータ設定
bool ib2::PosAttProfiler::setConfigThr(const ib2::ThrustAllocator& thr)
{
	thr_ = thr;
	return true;
}

//------------------------------------------------------------------------------
// 位置姿勢誘導プロファイル作成
ib2_msgs::CtlProfile ib2::PosAttProfiler::setProfile
(const ib2_msgs::Navigation& nav)
{
	// 初期位置姿勢設定
	setPose(nav);

	// 位置プロファイル
	r1_   = r0_;
	dh_   = Eigen::Vector3d::UnitX();
	xm_   = 0.;
	amax_ = 0.001;
	xcb_  = 0.;
	xce_  = 0.;
	xtt_  = 0.;

	// 姿勢プロファイル
	q1_ = q0_;
	axis_ = Eigen::Vector3d::UnitX();
	wdmax_ = 0.000001;
	qm_   = 0.;
	qcb_  = 0.;
	qce_  = 0.;
	qtt_  = 0.;
	seq_  = SEQUENCE::PARALLEL;
	return message();
}

//------------------------------------------------------------------------------
// 位置姿勢誘導プロファイル作成
ib2_msgs::CtlProfile ib2::PosAttProfiler::setProfile
(const ib2_msgs::Navigation& nav,
 const ib2_msgs::CtlCommandGoalConstPtr& goal, const ib2::CtlBody& b) 
{
	// 初期位置姿勢設定
	setPose(nav);

	// 制御移動量計算
	Eigen::Vector3d dr;
	Eigen::Quaterniond dq;
	std::tie(dr, dq) = maneuver(goal);
	if (goal->type.type == ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET)
		std::tie(dr, dq) = maneuver(r0_, q0_, dr, dq);

	// 目標位置設定
	setProfilePos(dr, b.m());
	
	// 目標姿勢設定
	setProfileAtt(dq, b.Is());
	
	seq_ = SEQUENCE::POS_ATT;
	return message();
}

//------------------------------------------------------------------------------
// 位置姿勢停止誘導プロファイル作成
ib2_msgs::CtlProfile ib2::PosAttProfiler::stoppingProfile
(const ib2_msgs::Navigation& nav, const ib2::CtlBody& b)
{
	// 初期位置姿勢設定
	setPose(nav);
	auto& wn(nav.twist.angular);
	Eigen::Vector3d w0(wn.x, wn.y, wn.z);

	// 回転の停止
	double w(w0.norm());
	axis_ = w > att_.epsQm() ? w0.normalized() : Eigen::Vector3d::UnitX();
	double Im(axis_.transpose() * b.Is() * axis_);
	wdmax_ = thr_.effectiveTmax(att_.Tmax(), axis_, pos_.etamax()) / Im;
	qcb_  = 0.;
	qce_  = 0.;
	qtt_  = w / wdmax_;
	qm_ = 0.5 * wdmax_ * qtt_ * qtt_;
	q1_ = q0_ * Eigen::AngleAxisd(qm_, axis_);
	
	// 並進の停止
	double v(v0_.norm());
	dh_   = v > pos_.epsRm() ? v0_.normalized() : Eigen::Vector3d::UnitX();
	Eigen::Vector3d db(q1_.conjugate() * dh_);
	amax_ = thr_.effectiveFmax(pos_.Fmax(), db, pos_.etamax()) / b.m();
	xcb_  = 0.;
	xce_  = 0.;
	xtt_  = v / amax_;
	xm_ = 0.5 *  amax_ * xtt_ * xtt_;
	r1_ = r0_ + dh_ * xm_;

	seq_ = SEQUENCE::ATT_POS;
	return message();
}

//------------------------------------------------------------------------------
// ホーミングモードプロファイル作成
ib2_msgs::CtlProfile ib2::PosAttProfiler::dockingProfile
(const ib2_msgs::Navigation& nav, const DOCKING_POS& pos, const DOCKING_ATT& att,
 const ib2::CtlBody& b)
{
	// 初期位置姿勢設定
	setPose(nav);

	// 制御移動量計算
	Eigen::Vector3d    dr(pos == DOCKING_POS::AIP ? pos_.aip() : pos_.rdp());
	Eigen::Quaterniond dq(att == DOCKING_ATT::AIA ? att_.aia() : att_.rda());
	std::tie(dr, dq) = maneuver(r0_, q0_, dr, dq);

	// 目標位置設定
	setProfilePos(dr, b.m());
	
	// 目標姿勢設定
	setProfileAtt(dq, b.Is());
	
	seq_ = SEQUENCE::POS_ATT;
	return message();
}

//------------------------------------------------------------------------------
// スキャンモードプロファイル作成
ib2_msgs::CtlProfile ib2::PosAttProfiler::scanProfile
(const ib2_msgs::Navigation& nav, size_t iaxis, const ib2::CtlBody& b)
{
	// 初期位置姿勢設定
	setPose(nav);

	// 目標位置設定
	Eigen::Vector3d dr(Eigen::Vector3d::Zero());
	setProfilePos(dr, b.m());

	// 目標姿勢設定
	axis_ = att_.scanAxes().at(iaxis);
	axis_.normalize();
	Eigen::Quaterniond dq(Eigen::AngleAxisd(ib2_mss::TWOPI, axis_));
	setProfileAtt(dq, b.Is());
	
	seq_ = SEQUENCE::POS_ATT;
	return message();
}

//------------------------------------------------------------------------------
// 初期位置姿勢の設定
void ib2::PosAttProfiler::setPose(const ib2_msgs::Navigation& nav)
{
	auto& tn(nav.pose.header.stamp);
	auto& rn(nav.pose.pose.position);
	auto& vn(nav.twist.linear);
	auto& qn(nav.pose.pose.orientation);
	
	t0_ = tn;
	r0_ = Eigen::Vector3d(rn.x, rn.y, rn.z);
	v0_ = Eigen::Vector3d(vn.x, vn.y, vn.z);
	q0_ = Eigen::Quaterniond(qn.w, qn.x, qn.y, qn.z);
	q0_.normalize();
}

//------------------------------------------------------------------------------
// 並進プロファイルの設定
void ib2::PosAttProfiler::setProfilePos(const Eigen::Vector3d& dr, double m)
{
	xm_ = dr.norm();
	auto db(xm_ > pos_.epsRm() ? dr.normalized() : Eigen::Vector3d::UnitX());
	dh_ = q0_ * db;
	r1_ = r0_ + dh_ * xm_;
	amax_ = thr_.effectiveFmax(pos_.Fmax(), db, pos_.eta()) / m;
	std::tie(xcb_, xce_, xtt_) = maneuverTiming(xm_, pos_.vmax(), amax_);
}

//------------------------------------------------------------------------------
// 回転プロファイルの設定
void ib2::PosAttProfiler::setProfileAtt
(const Eigen::Quaterniond& dq, const Eigen::Matrix3d& Is)
{
	q1_ = q0_ * dq;
	q1_.normalize();
	Eigen::Quaterniond dq1(q0_.conjugate() * q1_);
	if (dq.vec().dot(dq1.vec()) < 0.)
		q1_.coeffs() *= -1.;
	qm_ = 2. * acos(dq.w());
	axis_ = qm_ > att_.epsQm() ? dq.vec().normalized() : Eigen::Vector3d::UnitX();
	double Im(axis_.transpose() * Is * axis_);
	wdmax_ = thr_.effectiveTmax(att_.Tmax(), axis_, pos_.eta()) / Im;
	std::tie(qcb_, qce_, qtt_) = maneuverTiming(qm_, att_.wmax(), wdmax_);
}

//------------------------------------------------------------------------------
// スキャンモードの回転軸の数の取得
size_t ib2::PosAttProfiler::nscan() const
{
	return att_.scanAxes().size();
}

//------------------------------------------------------------------------------
// プロファイル終了時刻の取得
ros::Time ib2::PosAttProfiler::te() const
{
	double dur(seq_ == SEQUENCE::PARALLEL ? std::max(xtt_, qtt_) : xtt_ + qtt_);
	ros::Duration d(dur);
	return t0_ + d;
}

//------------------------------------------------------------------------------
// 誘導制御プロファイルメッセージの取得
ib2_msgs::CtlProfile ib2::PosAttProfiler::message() const
{
	std::vector<double> dtx{xcb_, xce_, xtt_};
	std::vector<double> dtq{qcb_, qce_, qtt_};
	if (seq_ == SEQUENCE::ATT_POS && qtt_ > 0.)
	{
		for (auto& dtxi : dtx)
			dtxi += qtt_;
	}
	else if (seq_ == SEQUENCE::POS_ATT && xtt_ > 0.)
	{
		for (auto& dtqi : dtq)
			dtqi += xtt_;
	}

	std::vector<double> dt{0.};
	dt.reserve(7);
	if (seq_ == SEQUENCE::ATT_POS)
	{
		if (qtt_ > 0.)
			std::copy(dtq.begin(), dtq.end(), std::back_inserter(dt));
		if (xtt_ > 0.)
			std::copy(dtx.begin(), dtx.end(), std::back_inserter(dt));
	}
	else
	{
		if (xtt_ > 0.)
			std::copy(dtx.begin(), dtx.end(), std::back_inserter(dt));
		if (seq_ != SEQUENCE::PARALLEL && qtt_ > 0.)
			std::copy(dtq.begin(), dtq.end(), std::back_inserter(dt));
	}

	auto n(dt.size());
	ib2_msgs::CtlProfile p;
	p.header.seq = ++msg_seq_;
	p.header.stamp = t0_;
	p.header.frame_id = FRAME_ISS;
	p.poses.resize(n);
	for (size_t i = 0; i < n; ++i)
	{
		ros::Duration d(dt.at(i));
		auto s(posAttProfile(t0_ + d).status(0));
		p.poses[i] = s.pose;
		p.poses[i].header.seq = static_cast<uint32_t>(i);
	}
	return p;
}

//------------------------------------------------------------------------------
// 位置姿勢誘導プロファイルに基づき基準値計算
CtlElements ib2::PosAttProfiler::posAttProfile(const ros::Time& t_stamp) const
{
	// プロファイル開始からの経過秒
	ros::Duration d(t_stamp - t0_);
	double t(d.toSec());
//TODO	ROS_INFO("interval = %f\n",t);

	// 位置誘導プロファイル計算
	Eigen::Vector3d r, v, a;
	std::tie(r, v, a) = posProfile(t);

	// 姿勢誘導プロファイル計算
	Eigen::Quaterniond q;
	Eigen::Vector3d w;
	std::tie(q, w) = attProfile(t);

	return CtlElements(t_stamp, r, v, a, q, w);
}

//------------------------------------------------------------------------------
// 制御目標までの状態量計算
ib2_msgs::CtlCommandFeedback ib2::PosAttProfiler::statesToGoal
(const ib2_msgs::Navigation& nav) const
{
	using namespace ib2_mss;
	auto& tn(nav.pose.header.stamp);
	auto& rn(nav.pose.pose.position);
	auto& qn(nav.pose.pose.orientation);

	ib2_msgs::CtlCommandFeedback fb;

	// 制御終了までの時間
	ros::Duration d(tn - t0_);
	double t(d.toSec());
	double dur(seq_ == SEQUENCE::PARALLEL ? std::max(xtt_, qtt_) : xtt_ + qtt_);
	ros::Duration dp(dur);
	fb.time_to_go = dp - d;

	// 目標位置までの距離
	Eigen::Vector3d r(rn.x, rn.y, rn.z);
	Eigen::Vector3d dr(r1_ - r);
	fb.pose_to_go.position.x = dr.x();
	fb.pose_to_go.position.y = dr.y();
	fb.pose_to_go.position.z = dr.z();

	// 目標姿勢までの回転角
	double tq(seq_ == SEQUENCE::POS_ATT ? t - xtt_ : t);
	double qp;
	std::tie(qp, std::ignore, std::ignore) = target
			(tq, qcb_, qce_, qtt_, att_.qthr(), qm_, att_.wmax(), wdmax_);
	Eigen::Quaterniond q(qn.w, qn.x, qn.y, qn.z);
	q.normalize();
	Eigen::Quaterniond dq0(q0_.conjugate() * q);
	if (dq0.vec().norm() > 0.01 && axis_.dot(dq0.vec()) < 0.)
		q.coeffs() *= -1.;
	Eigen::Quaterniond dq1(q.conjugate() * q1_);
	dq1.normalize();
	fb.pose_to_go.orientation.x = dq1.x();
	fb.pose_to_go.orientation.y = dq1.y();
	fb.pose_to_go.orientation.z = dq1.z();
	fb.pose_to_go.orientation.w = dq1.w();

	return fb;
}

//------------------------------------------------------------------------------
// 位置誘導リファレンス値計算
std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
ib2::PosAttProfiler::posProfile(double t) const
{
	auto r0 = r0_;
	if (seq_ == SEQUENCE::ATT_POS)
	{
		static const Eigen::Vector3d ZERO(Eigen::Vector3d::Zero());
		if (t <= qtt_)
			return std::make_tuple(r0_ + v0_ * t, v0_, ZERO);
		r0 += v0_ * qtt_;
		t -= qtt_;
	}
	double xp, vp, ap;
	std::tie(xp, vp, ap) = target
			(t, xcb_, xce_, xtt_, pos_.xthr(), xm_, pos_.vmax(), amax_);
	return std::make_tuple(r0 + dh_ * xp, dh_ * vp, dh_ * ap);
}

//------------------------------------------------------------------------------
// 姿勢誘導リファレンス値計算
std::pair<Eigen::Quaterniond, Eigen::Vector3d>
ib2::PosAttProfiler::attProfile(double t) const
{
	// 位置制御中は姿勢を変更しない
	if (seq_ == SEQUENCE::POS_ATT)
	{
		if (t <= xtt_)
			return std::make_pair(q0_, Eigen::Vector3d::Zero());
		t -= xtt_;
	}
	double qp, wp;
	std::tie(qp, wp, std::ignore) = target
			(t, qcb_, qce_, qtt_, att_.qthr(), qm_, att_.wmax(), wdmax_);
	Eigen::Quaterniond q(q0_ * Eigen::AngleAxisd(qp, axis_));
	Eigen::Vector3d    w(axis_ * wp);
	return std::make_pair(q, w);
}

// End Of File -----------------------------------------------------------------
