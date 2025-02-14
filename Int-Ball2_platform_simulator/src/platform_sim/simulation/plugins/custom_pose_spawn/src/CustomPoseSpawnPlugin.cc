#include <ignition/math/Pose3.hh>
#include <ros/ros.h>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class CustomPoseSpawnPlugin : public WorldPlugin
{
  public: void Load(physics::WorldPtr parent, sdf::ElementPtr sdf)
  {
    ros::NodeHandle nh;
    
    transport::NodePtr node(new transport::Node());
    node->Init(parent->Name());

    // 乱数のシード値が設定されている場合は読み込む
    int seed = -1;
    if (!nh.getParam("/sim_common/random_seed", seed))
    {
      gzerr << "Could not read the parameters of \"/sim_common/random_seed\".\n";
      return;
    }
    if(seed >= 0)
    {
			gazebo::common::Console::msg(__FILE__, __LINE__) << "Set the random seed value " << seed << "\n";
      ignition::math::Rand::Seed(static_cast<unsigned int>(seed));
    }

    // トピック名は~factoryとする必要がある
    transport::PublisherPtr factoryPub = node->Advertise<msgs::Factory>("~/factory");

    sdf::ElementPtr modelElement = sdf->GetElement("model");
    while (modelElement)
    {
      double x = 0;
      double y = 0;
      double z = 0;
      double roll = 0;
      double pitch = 0;
      double yaw = 0;

      if(modelElement->HasElement("random") && modelElement->HasElement("specify"))
      {
        gzerr << "\"random\" and \"specify\" cannot be specified in the same model.\n";
        return;
      }

      auto uri = modelElement->Get<std::string>("uri");

      if(modelElement->HasElement("random"))
      {
        auto randomElement = modelElement->GetElement("random");

        auto minXElement = randomElement->GetElement("key_min_x");
        auto minYElement = randomElement->GetElement("key_min_y");
        auto minZElement = randomElement->GetElement("key_min_z");
        auto maxXElement = randomElement->GetElement("key_max_x");
        auto maxYElement = randomElement->GetElement("key_max_y");
        auto maxZElement = randomElement->GetElement("key_max_z");
        auto minRollElement = randomElement->GetElement("key_min_roll");
        auto minPitchElement = randomElement->GetElement("key_min_pitch");
        auto minYawElement = randomElement->GetElement("key_min_yaw");
        auto maxRollElement = randomElement->GetElement("key_max_roll");
        auto maxPitchElement = randomElement->GetElement("key_max_pitch");
        auto maxYawElement = randomElement->GetElement("key_max_yaw");

        double minX, minY, minZ, maxX, maxY, maxZ,
                minRoll, minPitch, minYaw, maxRoll, maxPitch, maxYaw;
        if(!nh.getParam(minXElement->Get<std::string>(), minX) ||
           !nh.getParam(minYElement->Get<std::string>(), minY) ||
           !nh.getParam(minZElement->Get<std::string>(), minZ) ||
           !nh.getParam(maxXElement->Get<std::string>(), maxX) ||
           !nh.getParam(maxYElement->Get<std::string>(), maxY) ||
           !nh.getParam(maxZElement->Get<std::string>(), maxZ) ||
           !nh.getParam(minRollElement->Get<std::string>(), minRoll) ||
           !nh.getParam(minPitchElement->Get<std::string>(), minPitch) ||
           !nh.getParam(minYawElement->Get<std::string>(), minYaw) ||
           !nh.getParam(maxRollElement->Get<std::string>(), maxRoll) ||
           !nh.getParam(maxPitchElement->Get<std::string>(), maxPitch) ||
           !nh.getParam(maxYawElement->Get<std::string>(), maxYaw))
        {
          gzerr << "One of the following parameters could not be read from ROS parameter server: "
                    "key_min_x, key_min_y, key_min_z, "
                    "key_max_x, key_max_y, key_max_z, "
                    "key_min_roll, key_min_pitch, key_min_yaw, "
                    "key_max_roll, key_max_pitch, key_max_yaw"
                    "\n";
          return;
        }

        x = ignition::math::Rand::DblUniform(minX, maxX);
        y = ignition::math::Rand::DblUniform(minY, maxY);
        z = ignition::math::Rand::DblUniform(minZ, maxZ);

        auto r_random =  ignition::math::Rand::DblUniform(minRoll, maxRoll);
        auto r_tmp = ignition::math::Angle(r_random);
        r_tmp.Normalize();
        roll = r_tmp.Radian();

        auto p_random =  ignition::math::Rand::DblUniform(minPitch, maxPitch);
        auto p_tmp = ignition::math::Angle(p_random);
        p_tmp.Normalize();
        pitch = p_tmp.Radian();

        auto y_random =  ignition::math::Rand::DblUniform(minYaw, maxYaw);
        auto y_tmp = ignition::math::Angle(y_random);
        y_tmp.Normalize();
        yaw = y_tmp.Radian();

      }
      else
      {
        auto specifyElement = modelElement->GetElement("specify");

        auto specifyXElement = specifyElement->GetElement("key_x");
        auto specifyYElement = specifyElement->GetElement("key_y");
        auto specifyZElement = specifyElement->GetElement("key_z");
        auto specifyRollElement = specifyElement->GetElement("key_roll");
        auto specifyPitchElement = specifyElement->GetElement("key_pitch");
        auto specifyYawElement = specifyElement->GetElement("key_yaw");

        double r_rad, p_rad, y_rad;
        if(!nh.getParam(specifyXElement->Get<std::string>(),     x    ) ||
           !nh.getParam(specifyYElement->Get<std::string>(),     y    ) ||
           !nh.getParam(specifyZElement->Get<std::string>(),     z    ) ||
           !nh.getParam(specifyRollElement->Get<std::string>(),  r_rad) ||
           !nh.getParam(specifyPitchElement->Get<std::string>(), p_rad) ||
           !nh.getParam(specifyYawElement->Get<std::string>(),   y_rad))
        {
          gzerr << "One of the following parameters could not be read from ROS parameter server: "
                    "key_x, key_y, key_z, "
                    "key_roll, key_pitch, key_yaw"
                    "\n";
          return;
        }

        auto r_tmp = ignition::math::Angle(r_rad);
        r_tmp.Normalize();
        roll = r_tmp.Radian();

        auto p_tmp = ignition::math::Angle(p_rad);
        p_tmp.Normalize();
        pitch = p_tmp.Radian();

        auto y_tmp = ignition::math::Angle(y_rad);
        y_tmp.Normalize();
        yaw = y_tmp.Radian();
      }

      msgs::Factory msg;
      msg.set_sdf_filename(uri);
      msgs::Set(msg.mutable_pose(),
                  ignition::math::Pose3d(
                    ignition::math::Vector3d(x, y, z),
                    ignition::math::Quaterniond(roll, pitch, yaw)
                  )
                );
      factoryPub->Publish(msg);
      gzmsg << "Spwan model: " << uri << 
        " position(x, y, z) = (" << x << ", " << y << ", " << z << 
        ") attitude(roll, pitch, yaw) = (" << roll << ", " << pitch << ", " << yaw << ")\n";

      modelElement = modelElement->GetNextElement("model");
    }
  }
};

GZ_REGISTER_WORLD_PLUGIN(CustomPoseSpawnPlugin)
}
