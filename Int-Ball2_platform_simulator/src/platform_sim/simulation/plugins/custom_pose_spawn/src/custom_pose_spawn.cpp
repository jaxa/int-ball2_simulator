
#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/World.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Rand.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/Utility.hh>
#include <gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>

#include <string>

namespace custom_pose_spawn_plugin
{

class CustomPoseSpawn : public gz::sim::System,
                        public gz::sim::ISystemConfigure
{
public:
	CustomPoseSpawn() = default;
	~CustomPoseSpawn() override = default;

	void Configure(
		const gz::sim::Entity &_entity,
		const std::shared_ptr<const sdf::Element> &_sdf,
		gz::sim::EntityComponentManager &_ecm,
		gz::sim::EventManager &/*_eventMgr*/) override
	{
		if (!rclcpp::ok())
		{
			rclcpp::init(0, nullptr);
		}

		auto ros_node = std::make_shared<rclcpp::Node>("custom_pose_spawn");

		// 乱数のシード値
		int seed = -1;
		if (!ros_node->has_parameter("sim_common.random_seed"))
		{
			ros_node->declare_parameter<int>("sim_common.random_seed", seed);
		}
		ros_node->get_parameter("sim_common.random_seed", seed);
		if (seed >= 0)
		{
			RCLCPP_INFO(ros_node->get_logger(), "Set the random seed value %d", seed);
			gz::math::Rand::Seed(static_cast<unsigned int>(seed));
		}

		// Get world name for the create service
		auto *worldNameComp = _ecm.Component<gz::sim::components::Name>(_entity);
		std::string worldName = worldNameComp ? worldNameComp->Data() : "default";
		std::string createService = "/world/" + worldName + "/create";

		gz::transport::Node gzNode;

		// Iterate model elements in SDF
		auto sdfClone = _sdf->Clone();
		if (!sdfClone->HasElement("model"))
		{
			RCLCPP_WARN(ros_node->get_logger(), "No model elements found in plugin SDF");
			return;
		}

		auto get_param = [&ros_node](const std::string &name, double &value) {
			if (!ros_node->has_parameter(name))
			{
				ros_node->declare_parameter<double>(name, 0.0);
			}
			ros_node->get_parameter(name, value);
		};

		auto modelElement = sdfClone->GetElement("model");
		while (modelElement)
		{
			double x = 0, y = 0, z = 0;
			double roll = 0, pitch = 0, yaw = 0;

			bool hasRandom  = modelElement->HasElement("random");
			bool hasSpecify = modelElement->HasElement("specify");

			if (hasRandom && hasSpecify)
			{
				RCLCPP_ERROR(ros_node->get_logger(),
					"\"random\" and \"specify\" cannot be specified in the same model.");
				return;
			}

			auto uri = modelElement->Get<std::string>("uri");

			if (hasRandom)
			{
				auto randomElement = modelElement->GetElement("random");

				std::string keyMinX = randomElement->GetElement("key_min_x")->Get<std::string>();
				std::string keyMinY = randomElement->GetElement("key_min_y")->Get<std::string>();
				std::string keyMinZ = randomElement->GetElement("key_min_z")->Get<std::string>();
				std::string keyMaxX = randomElement->GetElement("key_max_x")->Get<std::string>();
				std::string keyMaxY = randomElement->GetElement("key_max_y")->Get<std::string>();
				std::string keyMaxZ = randomElement->GetElement("key_max_z")->Get<std::string>();
				std::string keyMinRoll  = randomElement->GetElement("key_min_roll")->Get<std::string>();
				std::string keyMinPitch = randomElement->GetElement("key_min_pitch")->Get<std::string>();
				std::string keyMinYaw   = randomElement->GetElement("key_min_yaw")->Get<std::string>();
				std::string keyMaxRoll  = randomElement->GetElement("key_max_roll")->Get<std::string>();
				std::string keyMaxPitch = randomElement->GetElement("key_max_pitch")->Get<std::string>();
				std::string keyMaxYaw   = randomElement->GetElement("key_max_yaw")->Get<std::string>();

				// Convert '/' to '.' for ROS 2 parameter names
				auto convertParamName = [](std::string name) -> std::string {
					if (!name.empty() && name[0] == '/') name = name.substr(1);
					std::replace(name.begin(), name.end(), '/', '.');
					return name;
				};

				double minX, minY, minZ, maxX, maxY, maxZ;
				double minRoll, minPitch, minYaw, maxRoll, maxPitch, maxYaw;

				get_param(convertParamName(keyMinX), minX);
				get_param(convertParamName(keyMinY), minY);
				get_param(convertParamName(keyMinZ), minZ);
				get_param(convertParamName(keyMaxX), maxX);
				get_param(convertParamName(keyMaxY), maxY);
				get_param(convertParamName(keyMaxZ), maxZ);
				get_param(convertParamName(keyMinRoll),  minRoll);
				get_param(convertParamName(keyMinPitch), minPitch);
				get_param(convertParamName(keyMinYaw),   minYaw);
				get_param(convertParamName(keyMaxRoll),  maxRoll);
				get_param(convertParamName(keyMaxPitch), maxPitch);
				get_param(convertParamName(keyMaxYaw),   maxYaw);

				x = gz::math::Rand::DblUniform(minX, maxX);
				y = gz::math::Rand::DblUniform(minY, maxY);
				z = gz::math::Rand::DblUniform(minZ, maxZ);

				gz::math::Angle r_tmp(gz::math::Rand::DblUniform(minRoll, maxRoll));
				r_tmp.Normalize();
				roll = r_tmp.Radian();

				gz::math::Angle p_tmp(gz::math::Rand::DblUniform(minPitch, maxPitch));
				p_tmp.Normalize();
				pitch = p_tmp.Radian();

				gz::math::Angle y_tmp(gz::math::Rand::DblUniform(minYaw, maxYaw));
				y_tmp.Normalize();
				yaw = y_tmp.Radian();
			}
			else if (hasSpecify)
			{
				auto specifyElement = modelElement->GetElement("specify");

				std::string keyX     = specifyElement->GetElement("key_x")->Get<std::string>();
				std::string keyY     = specifyElement->GetElement("key_y")->Get<std::string>();
				std::string keyZ     = specifyElement->GetElement("key_z")->Get<std::string>();
				std::string keyRoll  = specifyElement->GetElement("key_roll")->Get<std::string>();
				std::string keyPitch = specifyElement->GetElement("key_pitch")->Get<std::string>();
				std::string keyYaw   = specifyElement->GetElement("key_yaw")->Get<std::string>();

				auto convertParamName = [](std::string name) -> std::string {
					if (!name.empty() && name[0] == '/') name = name.substr(1);
					std::replace(name.begin(), name.end(), '/', '.');
					return name;
				};

				double r_rad = 0, p_rad = 0, y_rad = 0;
				get_param(convertParamName(keyX), x);
				get_param(convertParamName(keyY), y);
				get_param(convertParamName(keyZ), z);
				get_param(convertParamName(keyRoll),  r_rad);
				get_param(convertParamName(keyPitch), p_rad);
				get_param(convertParamName(keyYaw),   y_rad);

				gz::math::Angle r_tmp(r_rad); r_tmp.Normalize(); roll  = r_tmp.Radian();
				gz::math::Angle p_tmp(p_rad); p_tmp.Normalize(); pitch = p_tmp.Radian();
				gz::math::Angle y_tmp(y_rad); y_tmp.Normalize(); yaw   = y_tmp.Radian();
			}

			// Spawn model via gz-transport
			gz::msgs::EntityFactory req;
			req.set_sdf_filename(uri);
			gz::msgs::Set(req.mutable_pose(),
				gz::math::Pose3d(
					gz::math::Vector3d(x, y, z),
					gz::math::Quaterniond(roll, pitch, yaw)));

			gz::msgs::Boolean rep;
			bool result;
			bool success = gzNode.Request(createService, req, 5000, rep, result);

			if (success && result)
			{
				RCLCPP_INFO(ros_node->get_logger(),
					"Spawn model: %s position(x, y, z) = (%f, %f, %f) attitude(roll, pitch, yaw) = (%f, %f, %f)",
					uri.c_str(), x, y, z, roll, pitch, yaw);
			}
			else
			{
				RCLCPP_WARN(ros_node->get_logger(),
					"Failed to spawn model: %s", uri.c_str());
			}

			modelElement = modelElement->GetNextElement("model");
		}
	}
};

}  // namespace custom_pose_spawn_plugin

GZ_ADD_PLUGIN(
	custom_pose_spawn_plugin::CustomPoseSpawn,
	gz::sim::System,
	gz::sim::ISystemConfigure)

GZ_ADD_PLUGIN_ALIAS(custom_pose_spawn_plugin::CustomPoseSpawn,
	"custom_pose_spawn::CustomPoseSpawn")

// End Of File -----------------------------------------------------------------
