
#include "Log.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <cmath>


TEST(CtlTest, rotRoll)
{

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
	using namespace ib2_mss;
	Log::configure("log/test_ctl.log", "DEBUG");
	testing::InitGoogleTest(&argc, argv);
	// ros::init(argc, argv, "hello-test");
	// ros::NodeHandle nh;
	return RUN_ALL_TESTS();
}

// End Of File -----------------------------------------------------------------
