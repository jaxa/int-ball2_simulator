
#include <gtest/gtest.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>

TEST(Tf2QuaternionTest, rotRoll)
{
	double roll (M_PI / 3.);
	double pitch(0.);
	double yaw  (0.);

	double angle(roll);
	double sh(sin(angle * 0.5));
	double ch(cos(angle * 0.5));
	double sa(2.* sh * ch);
	double ca(2.* ch * ch - 1.);

//	tf2::Quaternion q(pitch, roll, yaw);	///< deprecated
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	EXPECT_DOUBLE_EQ(sh, q.x());
	EXPECT_DOUBLE_EQ(0., q.y());
	EXPECT_DOUBLE_EQ(0., q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());

	tf2::Quaternion u(0., 1., 0., 0.);
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
}

TEST(Tf2QuaternionTest, rotPitch)
{
	double roll (0.);
	double pitch(M_PI / 3.);
	double yaw  (0.);

	double sh(sin(pitch/2));
	double ch(cos(pitch/2));

//	tf2::Quaternion q(pitch, roll, yaw);	///< deprecated
	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);

	EXPECT_DOUBLE_EQ(0., q.x());
	EXPECT_DOUBLE_EQ(sh, q.y());
	EXPECT_DOUBLE_EQ(0., q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}

TEST(Tf2QuaternionTest, rotYaw)
{

	double yaw(M_PI / 3.);
	double sh(sin(yaw/2));
	double ch(cos(yaw/2));

	tf2::Quaternion q;
	q.setRPY(0., 0., yaw);

	EXPECT_DOUBLE_EQ(0., q.x());
	EXPECT_DOUBLE_EQ(0., q.y());
	EXPECT_DOUBLE_EQ(sh, q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}

TEST(Tf2QuaternionTest, rot)
{
	tf2::Vector3 axis(1., 2., 3.);
	EXPECT_DOUBLE_EQ(1., axis.x());
	EXPECT_DOUBLE_EQ(2., axis.y());
	EXPECT_DOUBLE_EQ(3., axis.z());
	axis.normalize();

	double angle(M_PI / 3.);
	double sh(sin(angle/2));
	double ch(cos(angle/2));

	tf2::Quaternion q(axis, angle);

	EXPECT_DOUBLE_EQ(axis.x() * sh, q.x());
	EXPECT_DOUBLE_EQ(axis.y() * sh, q.y());
	EXPECT_DOUBLE_EQ(axis.z() * sh, q.z());
	EXPECT_DOUBLE_EQ(ch, q.w());
}
TEST(Tf2QuaternionTest, testDCM)
{
	double roll (M_PI / 3.);
	double pitch(M_PI / 6.);
	double yaw  (M_PI / 4.);

	tf2::Quaternion q;
	q.setRPY(roll, pitch, yaw);
//	q.setEulerZYX(yaw, pitch, roll);		///< deprecated
//	tf2::Quaternion q(pitch, roll, yaw);	///< deprecated 3-2-1ではない
	EXPECT_DOUBLE_EQ(0.36042340565035591, q.x());
	EXPECT_DOUBLE_EQ(0.39190383732911993, q.y());
	EXPECT_DOUBLE_EQ(0.20056212114657512, q.z());
	EXPECT_DOUBLE_EQ(0.82236317190599928, q.w());

	tf2::Quaternion x(1., 0., 0., 0.);
	EXPECT_DOUBLE_EQ(1., x.x());
	EXPECT_DOUBLE_EQ(0., x.y());
	EXPECT_DOUBLE_EQ(0., x.z());
	EXPECT_DOUBLE_EQ(0., x.w());

	tf2::Quaternion y(0., 1., 0., 0.);
	EXPECT_DOUBLE_EQ(0., y.x());
	EXPECT_DOUBLE_EQ(1., y.y());
	EXPECT_DOUBLE_EQ(0., y.z());
	EXPECT_DOUBLE_EQ(0., y.w());

	tf2::Quaternion z(0., 0., 1., 0.);
	EXPECT_DOUBLE_EQ(0., z.x());
	EXPECT_DOUBLE_EQ(0., z.y());
	EXPECT_DOUBLE_EQ(1., z.z());
	EXPECT_DOUBLE_EQ(0., z.w());

	auto xr = q.inverse() * x * q;
	auto yr = q.inverse() * y * q;
	auto zr = q.inverse() * z * q;

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

// End Of File -----------------------------------------------------------------
