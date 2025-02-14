
#include "prop/prop.h"

//------------------------------------------------------------------------------
// メイン関数
int main(int argc, char **argv)
{
	// ROS初期化
	ros::init(argc, argv, "prop");

	ros::NodeHandle nh("prop");

	// 管理機能実行
	PropManager     prop_manager(nh);
	prop_manager.start();

	return 0;
}

// End Of File -----------------------------------------------------------------
