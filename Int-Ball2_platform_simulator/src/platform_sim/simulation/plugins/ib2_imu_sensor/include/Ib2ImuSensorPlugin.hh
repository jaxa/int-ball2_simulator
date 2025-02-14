#ifndef GAZEBO_PLUGINS_IB2IMUSENSORPLUGIN_HH_
#define GAZEBO_PLUGINS_IB2IMUSENSORPLUGIN_HH_

#include <ros/ros.h>
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/plugins/ImuSensorPlugin.hh"
#include "ib2_msgs/IMU.h"

namespace gazebo
{
  /** Int-Ball2用IMUセンサープラグイン. */
  class Ib2ImuSensorPlugin : public ImuSensorPlugin
  {
    public: 
      explicit Ib2ImuSensorPlugin();
      virtual void Load(sensors::SensorPtr parent, sdf::ElementPtr _sdf) override;

    protected:
      virtual void OnUpdate(sensors::ImuSensorPtr sensor) override;

    private:
      /** ROS関連処理用のNodeHandle. */
      ros::NodeHandle nh_;

      /** Int-Ball2向け IMU topicのPublisher. */
      ros::Publisher pub_;

      /** Int-Ball2向け IMU topic. */
      ib2_msgs::IMU msg_;

      /** IMUセンサの出力値に対するノイズ付加の有無. */
      bool add_noise_;

      /** IMUセンサのノイズ用パラメタ　加速度：X軸：平均値. */
      double noise_velocity_mean_x_;

      /** IMUセンサのノイズ用パラメタ　加速度：Y軸：平均値. */
      double noise_velocity_mean_y_;

      /** IMUセンサのノイズ用パラメタ　加速度：Z軸：平均値. */
      double noise_velocity_mean_z_;

      /** IMUセンサのノイズ用パラメタ　加速度：X軸：標準偏差値. */
      double noise_velocity_stddev_x_;
      
      /** IMUセンサのノイズ用パラメタ　加速度：Y軸：標準偏差値. */
      double noise_velocity_stddev_y_;
      
      /** IMUセンサのノイズ用パラメタ　加速度：Z軸：標準偏差値. */
      double noise_velocity_stddev_z_;

      /** IMUセンサのノイズ用パラメタ　角加速度：X軸：平均値. */
      double noise_acceleration_mean_x_;

      /** IMUセンサのノイズ用パラメタ　角加速度：Y軸：平均値. */
      double noise_acceleration_mean_y_;
      
      /** IMUセンサのノイズ用パラメタ　角加速度：Z軸：平均値. */
      double noise_acceleration_mean_z_;
      
      /** IMUセンサのノイズ用パラメタ　角加速度：X軸：標準偏差値. */
      double noise_acceleration_stddev_x_;
      
      /** IMUセンサのノイズ用パラメタ　角加速度：Y軸：標準偏差値. */
      double noise_acceleration_stddev_y_;
      
      /** IMUセンサのノイズ用パラメタ　角加速度：Z軸：標準偏差値. */
      double noise_acceleration_stddev_z_;
  };

  GZ_REGISTER_SENSOR_PLUGIN(Ib2ImuSensorPlugin)
}

#endif
