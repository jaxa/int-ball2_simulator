
#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

TEST(EigenQuaternionTest, rotRoll)
{
	double roll (M_PI / 3.);

	double angle(roll);
	double sh(sin(angle * 0.5));
	double ch(cos(angle * 0.5));
	double sa(2.* sh * ch);
	double ca(2.* ch * ch - 1.);

	Eigen::AngleAxisd a(roll, Eigen::Vector3d::UnitX());
	Eigen::Quaterniond q(a);

	EXPECT_DOUBLE_EQ(sh, q.x());
	EXPECT_DOUBLE_EQ(0., q.y());
	EXPECT_DOUBLE_EQ(0., q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());

	Eigen::Quaterniond u(0., 0., 1., 0.);
	EXPECT_DOUBLE_EQ(0., u.x());
	EXPECT_DOUBLE_EQ(1., u.y());
	EXPECT_DOUBLE_EQ(0., u.z());
	EXPECT_DOUBLE_EQ(0., u.w());

	auto y = q.inverse() * u * q;
	EXPECT_DOUBLE_EQ( 0, y.x());
	EXPECT_DOUBLE_EQ( ca, y.y());
	EXPECT_DOUBLE_EQ(-sa, y.z());
	EXPECT_DOUBLE_EQ( 0, y.w());

	auto yr = q * u * q.inverse();
	EXPECT_DOUBLE_EQ( 0, yr.x());
	EXPECT_DOUBLE_EQ( ca, yr.y());
	EXPECT_DOUBLE_EQ( sa, yr.z());
	EXPECT_DOUBLE_EQ( 0, yr.w());

	auto y1 = q.conjugate() * u.vec();
	EXPECT_DOUBLE_EQ( 0, y1.x());
	EXPECT_DOUBLE_EQ( ca, y1.y());
	EXPECT_DOUBLE_EQ(-sa, y1.z());

	auto y1r = q * u.vec();
	EXPECT_DOUBLE_EQ( 0, y1r.x());
	EXPECT_DOUBLE_EQ( ca, y1r.y());
	EXPECT_DOUBLE_EQ( sa, y1r.z());

}

TEST(EigenQuaternionTest, rotPitch)
{
	double pitch(M_PI / 3.);

	double sh(sin(pitch/2));
	double ch(cos(pitch/2));

	Eigen::AngleAxisd a(pitch, Eigen::Vector3d::UnitY());
	Eigen::Quaterniond q(a);

	EXPECT_DOUBLE_EQ(0., q.x());
	EXPECT_DOUBLE_EQ(sh, q.y());
	EXPECT_DOUBLE_EQ(0., q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}

TEST(EigenQuaternionTest, rotYaw)
{

	double yaw(M_PI / 3.);
	double sh(sin(yaw/2));
	double ch(cos(yaw/2));

	Eigen::AngleAxisd a(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q(a);

	EXPECT_DOUBLE_EQ(0., q.x());
	EXPECT_DOUBLE_EQ(0., q.y());
	EXPECT_DOUBLE_EQ(sh, q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}

TEST(EigenQuaternionTest, rot)
{
	Eigen::Vector3d axis(1., 2., 3.);
	EXPECT_DOUBLE_EQ(1., axis.x());
	EXPECT_DOUBLE_EQ(2., axis.y());
	EXPECT_DOUBLE_EQ(3., axis.z());
	axis.normalize();

	double angle(M_PI / 3.);
	double sh(sin(angle/2));
	double ch(cos(angle/2));

	Eigen::AngleAxisd a(angle, axis);
	Eigen::Quaterniond q(a);

	EXPECT_DOUBLE_EQ(axis.x() * sh, q.x());
	EXPECT_DOUBLE_EQ(axis.y() * sh, q.y());
	EXPECT_DOUBLE_EQ(axis.z() * sh, q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}

TEST(EigenQuaternionTest, testDCM)
{
	double roll (M_PI / 3.);
	double pitch(M_PI / 6.);
	double yaw  (M_PI / 4.);

	Eigen::AngleAxisd rotx(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd roty(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd rotz(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond q(rotz * roty * rotx);
	EXPECT_DOUBLE_EQ(0.36042340565035591, q.x());
	EXPECT_DOUBLE_EQ(0.39190383732911993, q.y());
	EXPECT_DOUBLE_EQ(0.20056212114657512, q.z());
	EXPECT_DOUBLE_EQ(0.82236317190599928, q.w());

	auto xr = q.conjugate() * Eigen::Vector3d::UnitX();
	auto yr = q.conjugate() * Eigen::Vector3d::UnitY();
	auto zr = q.conjugate() * Eigen::Vector3d::UnitZ();

	double sr(sin(roll ));
	double sp(sin(pitch));
	double sy(sin(yaw  ));
	double cr(cos(roll ));
	double cp(cos(pitch));
	double cy(cos(yaw  ));

	const double EPS(1.e-15);
	EXPECT_NEAR(     cp * cy          , xr.x(), EPS);
	EXPECT_NEAR(sr * sp * cy - cr * sy, xr.y(), EPS);
	EXPECT_NEAR(cr * sp * cy + sr * sy, xr.z(), EPS);

	EXPECT_NEAR(     cp * cy          , yr.x(), EPS);
	EXPECT_NEAR(sr * sp * sy + cr * cy, yr.y(), EPS);
	EXPECT_NEAR(cr * sp * sy - sr * cy, yr.z(), EPS);

	EXPECT_NEAR(    -sp, zr.x(), EPS);
	EXPECT_NEAR(sr * cp, zr.y(), EPS);
	EXPECT_NEAR(cr * cp, zr.z(), EPS);
}

TEST(EigenVectorXdTest, initialize)
{
	Eigen::Vector3d x(1,2,3);
	Eigen::VectorXd y(6);
	y <<x, Eigen::Vector3d::Zero();

	EXPECT_EQ(6, y.rows());
	EXPECT_EQ(1, y.cols());

	EXPECT_DOUBLE_EQ(1, y(0));
	EXPECT_DOUBLE_EQ(2, y(1));
	EXPECT_DOUBLE_EQ(3, y(2));
	EXPECT_DOUBLE_EQ(0, y(3));
	EXPECT_DOUBLE_EQ(0, y(4));
	EXPECT_DOUBLE_EQ(0, y(5));
}

// End Of File -----------------------------------------------------------------
