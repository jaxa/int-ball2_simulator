
#include "ctl/ctl_body.h"
#include "guidance_control_common/RangeChecker.h"

#include <string>

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::CtlBody::CtlBody() :
m_(1.), Is_(Eigen::Matrix3d::Identity())
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::CtlBody::CtlBody(const ros::NodeHandle& nh) :
m_(1.), Is_(Eigen::Matrix3d::Identity())
{
	using namespace ib2_mss;
	
	static const std::string ROSPARAM_MASS ("/ctl_body/mass");
	static const std::string ROSPARAM_IS_XX("/ctl_body/Is/xx");
	static const std::string ROSPARAM_IS_YY("/ctl_body/Is/yy");
	static const std::string ROSPARAM_IS_ZZ("/ctl_body/Is/zz");
	static const std::string ROSPARAM_IS_XY("/ctl_body/Is/xy");
	static const std::string ROSPARAM_IS_YZ("/ctl_body/Is/yz");
	static const std::string ROSPARAM_IS_ZX("/ctl_body/Is/zx");
	
	double mass(-1.);
	double Is_xx(-1.);
	double Is_yy(-1.);
	double Is_zz(-1.);
	double Is_xy(0.);
	double Is_yz(0.);
	double Is_zx(0.);
	
	nh.getParam(ROSPARAM_MASS, mass);
	nh.getParam(ROSPARAM_IS_XX, Is_xx);
	nh.getParam(ROSPARAM_IS_YY, Is_yy);
	nh.getParam(ROSPARAM_IS_ZZ, Is_zz);
	nh.getParam(ROSPARAM_IS_XY, Is_xy);
	nh.getParam(ROSPARAM_IS_YZ, Is_yz);
	nh.getParam(ROSPARAM_IS_ZX, Is_zx);
	
	RangeCheckerD::positive(mass , true, ROSPARAM_MASS);
	RangeCheckerD::positive(Is_xx, true, ROSPARAM_IS_XX);
	RangeCheckerD::positive(Is_yy, true, ROSPARAM_IS_YY);
	RangeCheckerD::positive(Is_zz, true, ROSPARAM_IS_ZZ);
	
	m_ = mass;
	Is_(0,0) = Is_xx;
	Is_(1,1) = Is_yy;
	Is_(2,2) = Is_zz;
	Is_(0,1) = Is_xy;
	Is_(1,2) = Is_yz;
	Is_(2,0) = Is_zx;
	Is_(1,0) = Is_(0,1);
	Is_(2,1) = Is_(1,2);
	Is_(0,2) = Is_(2,0);

	ROS_INFO("******** Set Parameters in ctl_body.cpp");
	ROS_INFO("%s   : %f", ROSPARAM_MASS.c_str(),  m_);
	ROS_INFO("%s   : %f", ROSPARAM_IS_XX.c_str(), Is_(0, 0));
	ROS_INFO("%s   : %f", ROSPARAM_IS_YY.c_str(), Is_(1, 1));
	ROS_INFO("%s   : %f", ROSPARAM_IS_ZZ.c_str(), Is_(2, 2));
	ROS_INFO("%s   : %f", ROSPARAM_IS_XY.c_str(), Is_(0, 1));
	ROS_INFO("%s   : %f", ROSPARAM_IS_YZ.c_str(), Is_(1, 2));
	ROS_INFO("%s   : %f", ROSPARAM_IS_ZX.c_str(), Is_(2, 0));
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::CtlBody::~CtlBody() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::CtlBody::CtlBody(const CtlBody&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::CtlBody& ib2::CtlBody::operator=(const CtlBody&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::CtlBody::CtlBody(CtlBody&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::CtlBody& ib2::CtlBody::operator=(CtlBody&&) = default;

//------------------------------------------------------------------------------
//  機体質量の取得
double ib2::CtlBody::m() const
{
	return m_;
}

//------------------------------------------------------------------------------
//  質量特性行列の参照
const Eigen::Matrix3d& ib2::CtlBody::Is() const
{
	return Is_;
}

// End Of File -----------------------------------------------------------------
