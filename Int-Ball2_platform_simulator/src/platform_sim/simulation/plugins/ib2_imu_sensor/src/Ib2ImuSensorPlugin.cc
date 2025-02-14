#include <functional>
#include "Ib2ImuSensorPlugin.hh"
#include "ib2_msgs/IMU.h"

using namespace gazebo;

Ib2ImuSensorPlugin::Ib2ImuSensorPlugin() : ImuSensorPlugin()
{
}

void Ib2ImuSensorPlugin::Load(sensors::SensorPtr parent,
    sdf::ElementPtr /*_sdf*/)
{
  ImuSensorPlugin::Load(parent, nullptr);

  if(!ros::isInitialized())
  {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "ib2_imu_sensor", ros::init_options::NoSigintHandler);
  }

  nh_ = ros::NodeHandle("ib2_imu");

  pub_ = nh_.advertise<ib2_msgs::IMU>("/imu/imu", 1);

  /* Rosparamの読み込み */

  // 乱数のシード値
  int seed = -1;
  if (!nh_.getParam("/sim_common/random_seed", seed))
  {
    gzerr << "Could not read the parameters of \"/sim_common/random_seed\".\n";
    return;
  }
  if(seed >= 0)
  {
		gazebo::common::Console::msg(__FILE__, __LINE__) << "Set the random seed value " << seed << "\n";
    ignition::math::Rand::Seed(static_cast<unsigned int>(seed));
  }

  if (!nh_.getParam("/ib2_imu/add_noise", add_noise_))
  {
		gzerr << "Cannot Get /ib2_imu/add_noise in ib2_imu_sensor plugin \n";
    return;
  }

  if (add_noise_)
  {
    if (!nh_.getParam("/ib2_imu/noise/velocity/mean/x", noise_velocity_mean_x_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/mean/x in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/velocity/mean/y", noise_velocity_mean_y_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/mean/y in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/velocity/mean/z", noise_velocity_mean_z_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/mean/z in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/velocity/stddev/x", noise_velocity_stddev_x_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/stddev/x in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/velocity/stddev/y", noise_velocity_stddev_y_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/stddev/y in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/velocity/stddev/z", noise_velocity_stddev_z_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/velocity/stddev/z in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/mean/x", noise_acceleration_mean_x_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/mean/x in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/mean/y", noise_acceleration_mean_y_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/mean/y in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/mean/z", noise_acceleration_mean_z_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/mean/z in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/stddev/x", noise_acceleration_stddev_x_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/stddev/x in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/stddev/y", noise_acceleration_stddev_y_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/stddev/y in ib2_imu_sensor plugin \n";
      return;
    }

    if (!nh_.getParam("/ib2_imu/noise/acceleration/stddev/z", noise_acceleration_stddev_z_))
    {
      gzerr << "Cannot Get /ib2_imu/noise/acceleration/stddev/z in ib2_imu_sensor plugin \n";
      return;
    }

  }
}

void Ib2ImuSensorPlugin::OnUpdate(sensors::ImuSensorPtr sensor)
{
      /** IMUセンサのノイズ用パラメタ　加速度：Z軸：標準偏差値. */
  if (!sensor->IsActive())
  {
    return;
  }

  auto timestamp = sensor->LastMeasurementTime();
  /* プラグイン内でノイズを付加するため、センサーオブジェクトからはノイズ無しの値を取得する */
  auto velocity = sensor->AngularVelocity(true);
  auto acceleration = sensor->LinearAcceleration(true);

  if (add_noise_)
  {
    velocity += ignition::math::Vector3<double>(
      ignition::math::Rand::DblNormal(noise_velocity_mean_x_, noise_velocity_stddev_x_),
      ignition::math::Rand::DblNormal(noise_velocity_mean_y_, noise_velocity_stddev_y_),
      ignition::math::Rand::DblNormal(noise_velocity_mean_z_, noise_velocity_stddev_z_)
    );
    acceleration += ignition::math::Vector3<double>(
      ignition::math::Rand::DblNormal(noise_acceleration_mean_x_, noise_acceleration_stddev_x_),
      ignition::math::Rand::DblNormal(noise_acceleration_mean_y_, noise_acceleration_stddev_y_),
      ignition::math::Rand::DblNormal(noise_acceleration_mean_z_, noise_acceleration_stddev_z_)
    );
  }

  msg_.stamp.sec = timestamp.sec;
  msg_.stamp.nsec = timestamp.nsec;
  msg_.gyro_x = velocity.X();
  msg_.gyro_y = velocity.Y();
  msg_.gyro_z = velocity.Z();
  msg_.acc_x = acceleration.X();
  msg_.acc_y = acceleration.Y();
  msg_.acc_z = acceleration.Z();
  msg_.temperature = 0;

  pub_.publish(msg_);
}

