
#include "prop/prop.h"

//------------------------------------------------------------------------------
// メイン関数
int main(int argc, char **argv)
{
	// ROS初期化
	rclcpp::init(argc, argv);

	auto node = std::make_shared<PropManager>();

	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}

// End Of File -----------------------------------------------------------------
