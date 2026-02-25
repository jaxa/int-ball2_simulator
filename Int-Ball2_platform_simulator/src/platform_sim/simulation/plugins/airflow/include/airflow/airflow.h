
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Rand.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sim_msgs/srv/update_parameter.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>

#include "coordinate_transform/coordinate_transform.h"

#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>

namespace airflow_plugin
{
	/**
	 * @brief Air Flowによる外乱を模擬するプラグイン.
	 */
	class Airflow : public gz::sim::System,
	                public gz::sim::ISystemConfigure,
	                public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		Airflow();
		~Airflow() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Airflow(const Airflow&) = delete;
		Airflow& operator=(const Airflow&) = delete;
		Airflow(Airflow&&) = delete;
		Airflow& operator=(Airflow&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		void Configure(const gz::sim::Entity &_entity,
		               const std::shared_ptr<const sdf::Element> &_sdf,
		               gz::sim::EntityComponentManager &_ecm,
		               gz::sim::EventManager &_eventMgr) override;

		void PreUpdate(const gz::sim::UpdateInfo &_info,
		               gz::sim::EntityComponentManager &_ecm) override;

	private:
		struct Vertex
		{
			bool   is_positive_x{false};
			bool   is_positive_y{false};
			bool   is_positive_z{false};
			double distance{0.0};
			pcl::PointWithViewpoint point;

			Vertex() { point.x = point.y = point.z = point.vp_x = point.vp_y = point.vp_z = 0.0; }
		};

		struct State
		{
			gz::math::Vector3d    pos;
			gz::math::Vector3d    vel;
			gz::math::Quaterniond rot{1.0, 0.0, 0.0, 0.0};
		};

		void getParameter();
		void updateParameter(
			const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> req,
			std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res);
		void logParameter();
		void getModels(gz::sim::EntityComponentManager &_ecm);
		void readFdTable();
		void airflowCallBack(const gz::sim::UpdateInfo &_info,
		                      gz::sim::EntityComponentManager &_ecm);
		void getRobotState(gz::sim::EntityComponentManager &_ecm);
		void loadWindVecField();
		void getPointIndex();
		int  nnSearch(const gz::math::Vector3d& point, std::vector<int>& point_idx);
		bool selectVertex(
			const gz::math::Vector3d& point,
			const std::vector<pcl::PointWithViewpoint>& in_vertices,
			      std::vector<pcl::PointWithViewpoint>& out_vertices);
		gz::math::Vector3d lerp(
			const std::vector<pcl::PointWithViewpoint>& vertex,
			const gz::math::Vector3d& point);
		void getWindVec(const gz::math::Vector3d& point, gz::math::Vector3d& wind_v);
		void getForce(const gz::math::Vector3d& wind_v, gz::math::Vector3d& force);
		void getTorque(const std::vector<gz::math::Vector3d>& wind_v, gz::math::Vector3d& torque);
		void addForceAndTorque(gz::sim::EntityComponentManager &_ecm,
		                       const gz::math::Vector3d& force_body,
		                       const gz::math::Vector3d& torque_body);
		void pubWrenchAndWind(const gz::sim::UpdateInfo &_info,
		                       const gz::math::Vector3d& f,
		                       const gz::math::Vector3d& t,
		                       const gz::math::Vector3d& w);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		std::shared_ptr<rclcpp::Node>  ros_node_;

		std::string                    iss_name_;
		std::string                    ib2_name_;
		gz::sim::Entity                iss_model_{gz::sim::kNullEntity};
		gz::sim::Entity                ib2_model_{gz::sim::kNullEntity};
		gz::sim::Entity                iss_link_{gz::sim::kNullEntity};
		gz::sim::Entity                ib2_link_{gz::sim::kNullEntity};

		gz::math::Vector3d             jpm_pos_;
		gz::math::Vector3d             jpm_att_;
		gz::math::Vector3d             cg_;
		double                         radius_{0.0};

		rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr pub_drag_;
		rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr  pub_wind_;

		rclcpp::Service<sim_msgs::srv::UpdateParameter>::SharedPtr af_param_server_;

		double                         sim_time_{0.0};
		int                            time_step_{0};
		double                         get_cycle_{0.0};
		double                         pub_cycle_{0.0};

		std::string                    pcd_;
		std::string                    table_;
		std::string                    plugin_path_;
		std::vector<std::array<double, 2>> fd_table_;

		gz::math::Vector3d             wind_v_cg_;
		std::vector<gz::math::Vector3d> wind_v_lp_;
		std::vector<gz::math::Vector3d> load_point_;

		double                         force_stddev_{0.0};
		double                         torque_stddev_{0.0};
		double                         kappa_{0.0};
		double                         S_{0.0};

		State                          state_;
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_now_;
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_next1_;
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_next2_;
		std::vector<int>               point_idx_;

		gazebo::CoordinateTransform    coord_transformer_;

		int                            current_postfix_{-1};
		int                            get_cnt_{0};
		int                            pub_cnt_{0};

		bool                           velocity_checks_enabled_{false};
	};
}
// End Of File -----------------------------------------------------------------
