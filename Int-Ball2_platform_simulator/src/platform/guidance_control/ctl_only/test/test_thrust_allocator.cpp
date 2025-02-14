
#include "thrust_allocator.h"
#include "thrust_allocator.cpp"

#include <gtest/gtest.h>

#include <Eigen/Dense>

#include <cmath>

namespace
{
	/** ファン推力最大値[N] */
	const double PROP_FMAX(30.e-3);

	std::vector<double> WP_VEC
	{
		0,      0,      0,      0,       0,       6.0423,
		0,      0.6731, 0,      12.0773, 0,       0,
		0,      0.6731, 0.4488, 0,       12.0773, 0,
		0,      0,      0.4488, 12.0773, 12.0773, 6.0423,
		0.3366, 0,      0.4488, 12.0773, 0,       0,
		0.3366, 0,      0,      0,       12.0773, 0,
		0.3366, 0.6731, 0,      12.0773, 12.0773, 6.0423,
		0.3366, 0.6731, 0.4488, 0,       0,       6.0423
	};

	std::vector<double> WM_VEC
	{
		0.3366, 0.6731, 0.4488, 12.0773, 12.0773, 0,
		0.3366, 0,      0.4488, 0,       12.0773, 6.0423,
		0.3366, 0,      0,      12.0773, 0,       6.0423,
		0.3366, 0.6731, 0,      0,       0,       0,
		0,      0.6731, 0,      0,       12.0773, 6.0423,
		0,      0.6731, 0.4488, 12.0773, 0,       6.0423,
		0,      0,      0.4488, 0,       0,       0,
		0,      0,      0,      12.0773, 12.0773, 0
	};


}

TEST(ThrustAllocatorTest, setarray)
{
	using namespace ib2;

	Eigen::MatrixXd Wp = Eigen::Map<Eigen::MatrixXd>(WP_VEC.data(), 8,6);
	Eigen::MatrixXd Wm = Eigen::Map<Eigen::MatrixXd>(WM_VEC.data(), 8,6);

	ThrustAllocator thr(PROP_FMAX, Wp, Wm);

	Eigen::Vector3d F(1, 2, 3);
	Eigen::Vector3d T(4, 5, 6);


	Eigen::VectorXd y(thr.allocate(F,T));

	Eigen::VectorXd yin(6);
	yin << F, T;
	Eigen::VectorXd yy(thr.allocate(yin));

	double Fmax(0.0891338);
	Eigen::Vector3d dir(0.975900073,0.097590007,0.195180015);
	// dir.normalize();
	double p(thr.effectiveFmax(Fmax,dir, 0.99));
	EXPECT_DOUBLE_EQ(0.003569876113200479, p);

	Eigen::Vector3d dir1(0.0975900073,0.975900073,0.195180015);
	// dir.normalize();
	double p1(thr.effectiveFmax(Fmax,dir1, 0.99));
	EXPECT_DOUBLE_EQ(0.0025198880356780912, p1);
}

TEST(ThrustAllocatorTest, allocate)
{
	Eigen::MatrixXd Wp = Eigen::Map<Eigen::MatrixXd>(WP_VEC.data(), 8,6);
	Eigen::MatrixXd Wm = Eigen::Map<Eigen::MatrixXd>(WM_VEC.data(), 8,6);

	double FmaxP(0.0891338);
	double eta(0.99);

	Eigen::Vector3d dir(0.975900073,0.097590007,0.195180015);
	// Eigen::Vector3d dir(1,0,0);

	Eigen::VectorXd y(6);
	y << dir * FmaxP, Eigen::Vector3d::Zero();
	FmaxDif fdif(PROP_FMAX, Wp, Wm, y, eta);

	EXPECT_DOUBLE_EQ(0., fdif(0.));
	EXPECT_DOUBLE_EQ(0., fdif(0.1));
	EXPECT_DOUBLE_EQ(0., fdif(0.2));
	EXPECT_DOUBLE_EQ(0., fdif(0.3));
	EXPECT_DOUBLE_EQ(0., fdif(0.4));
	EXPECT_DOUBLE_EQ(0., fdif(0.5));
	EXPECT_DOUBLE_EQ(0., fdif(0.6));
	EXPECT_DOUBLE_EQ(0., fdif(0.7));
	EXPECT_DOUBLE_EQ(0., fdif(0.8));
	EXPECT_DOUBLE_EQ(0., fdif(0.9));
	EXPECT_DOUBLE_EQ(0., fdif(1.0));





}


// End Of File -----------------------------------------------------------------
