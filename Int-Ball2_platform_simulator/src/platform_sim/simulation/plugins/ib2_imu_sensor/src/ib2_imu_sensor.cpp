
#include "ib2_imu_sensor/ib2_imu_sensor.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2_imu_sensor_plugin::Ib2ImuSensor::Ib2ImuSensor() = default;

//------------------------------------------------------------------------------
// デストラクタ.
ib2_imu_sensor_plugin::Ib2ImuSensor::~Ib2ImuSensor() = default;

//------------------------------------------------------------------------------
// プラグインの初期設定
void ib2_imu_sensor_plugin::Ib2ImuSensor::Configure(
	const gz::sim::Entity &_entity,
	const std::shared_ptr<const sdf::Element> &/*_sdf*/,
	gz::sim::EntityComponentManager &_ecm,
	gz::sim::EventManager &/*_eventMgr*/)
{
	if (!rclcpp::ok())
	{
		rclcpp::init(0, nullptr);
	}

	// Store model entity and get first link
	model_entity_ = _entity;
	gz::sim::Model model(model_entity_);
	auto links = model.Links(_ecm);
	if (!links.empty())
	{
		link_entity_ = links[0];
	}

	// Load simulation parameter files
	rclcpp::NodeOptions node_options;
	try {
		std::string ib2_gazebo_share = ament_index_cpp::get_package_share_directory("ib2_gazebo");
		node_options.arguments({
			"--ros-args",
			"--params-file", ib2_gazebo_share + "/sim/sim.yaml",
			"--params-file", ib2_gazebo_share + "/sim/custom.yaml"
		});
	} catch (...) {}

	// Create ROS node
	ros_node_ = std::make_shared<rclcpp::Node>("ib2_imu_sensor", node_options);

	// Publisher
	pub_ = ros_node_->create_publisher<ib2_msgs::msg::IMU>("/imu/imu", 1);

	// Get parameters
	getParameter();
}

//------------------------------------------------------------------------------
// ROS Parameter Serverからパラメータ取得
void ib2_imu_sensor_plugin::Ib2ImuSensor::getParameter()
{
	auto get_param = [this](const std::string &name, auto &value) {
		using T = std::decay_t<decltype(value)>;
		if (!ros_node_->has_parameter(name)) {
			ros_node_->declare_parameter<T>(name, value);
		}
		ros_node_->get_parameter(name, value);
	};

	// 乱数のシード値
	int seed = -1;
	get_param("sim_common.random_seed", seed);
	if (seed >= 0)
	{
		RCLCPP_INFO(ros_node_->get_logger(), "Set the random seed value %d", seed);
		gz::math::Rand::Seed(static_cast<unsigned int>(seed));
	}

	get_param("ib2_imu.add_noise", add_noise_);

	if (add_noise_)
	{
		get_param("ib2_imu.noise.velocity.mean.x", noise_velocity_mean_x_);
		get_param("ib2_imu.noise.velocity.mean.y", noise_velocity_mean_y_);
		get_param("ib2_imu.noise.velocity.mean.z", noise_velocity_mean_z_);
		get_param("ib2_imu.noise.velocity.stddev.x", noise_velocity_stddev_x_);
		get_param("ib2_imu.noise.velocity.stddev.y", noise_velocity_stddev_y_);
		get_param("ib2_imu.noise.velocity.stddev.z", noise_velocity_stddev_z_);
		get_param("ib2_imu.noise.acceleration.mean.x", noise_acceleration_mean_x_);
		get_param("ib2_imu.noise.acceleration.mean.y", noise_acceleration_mean_y_);
		get_param("ib2_imu.noise.acceleration.mean.z", noise_acceleration_mean_z_);
		get_param("ib2_imu.noise.acceleration.stddev.x", noise_acceleration_stddev_x_);
		get_param("ib2_imu.noise.acceleration.stddev.y", noise_acceleration_stddev_y_);
		get_param("ib2_imu.noise.acceleration.stddev.z", noise_acceleration_stddev_z_);
	}
}

//------------------------------------------------------------------------------
// 物理ステップ前の更新
void ib2_imu_sensor_plugin::Ib2ImuSensor::PreUpdate(
	const gz::sim::UpdateInfo &_info,
	gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused)
		return;

	if (link_entity_ == gz::sim::kNullEntity)
		return;

	gz::sim::Link link(link_entity_);

	// Enable velocity and acceleration checks on first call
	if (!checks_enabled_)
	{
		link.EnableVelocityChecks(_ecm);
		link.EnableAccelerationChecks(_ecm);
		checks_enabled_ = true;
		return;
	}

	// Get world pose
	auto worldPose = link.WorldPose(_ecm);
	if (!worldPose)
		return;

	// Get world angular velocity and transform to body frame (gyro)
	auto worldAngVel = link.WorldAngularVelocity(_ecm);
	gz::math::Vector3d bodyAngVel = gz::math::Vector3d::Zero;
	if (worldAngVel)
	{
		bodyAngVel = worldPose->Rot().RotateVectorReverse(*worldAngVel);
	}

	// Get world linear acceleration and transform to body frame (accelerometer)
	auto worldLinAccel = link.WorldLinearAcceleration(_ecm);
	gz::math::Vector3d bodyLinAccel = gz::math::Vector3d::Zero;
	if (worldLinAccel)
	{
		bodyLinAccel = worldPose->Rot().RotateVectorReverse(*worldLinAccel);
	}

	// Add noise
	if (add_noise_)
	{
		bodyAngVel += gz::math::Vector3d(
			gz::math::Rand::DblNormal(noise_velocity_mean_x_, noise_velocity_stddev_x_),
			gz::math::Rand::DblNormal(noise_velocity_mean_y_, noise_velocity_stddev_y_),
			gz::math::Rand::DblNormal(noise_velocity_mean_z_, noise_velocity_stddev_z_));

		bodyLinAccel += gz::math::Vector3d(
			gz::math::Rand::DblNormal(noise_acceleration_mean_x_, noise_acceleration_stddev_x_),
			gz::math::Rand::DblNormal(noise_acceleration_mean_y_, noise_acceleration_stddev_y_),
			gz::math::Rand::DblNormal(noise_acceleration_mean_z_, noise_acceleration_stddev_z_));
	}

	// Publish IMU message
	ib2_msgs::msg::IMU msg;
	auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		_info.simTime).count();
	msg.stamp.sec     = static_cast<int32_t>(sim_time_ns / 1000000000);
	msg.stamp.nanosec = static_cast<uint32_t>(sim_time_ns % 1000000000);
	msg.gyro_x        = static_cast<float>(bodyAngVel.X());
	msg.gyro_y        = static_cast<float>(bodyAngVel.Y());
	msg.gyro_z        = static_cast<float>(bodyAngVel.Z());
	msg.acc_x         = static_cast<float>(bodyLinAccel.X());
	msg.acc_y         = static_cast<float>(bodyLinAccel.Y());
	msg.acc_z         = static_cast<float>(bodyLinAccel.Z());
	msg.temperature   = 0.0f;

	pub_->publish(msg);
}

//------------------------------------------------------------------------------
// gz-simのシステムプラグインとして登録
GZ_ADD_PLUGIN(
	ib2_imu_sensor_plugin::Ib2ImuSensor,
	gz::sim::System,
	gz::sim::ISystemConfigure,
	gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ib2_imu_sensor_plugin::Ib2ImuSensor, "ib2_imu_sensor::Ib2ImuSensor")

// End Of File -----------------------------------------------------------------
