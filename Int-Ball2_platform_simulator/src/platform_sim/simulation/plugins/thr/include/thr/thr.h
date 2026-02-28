
#pragma once

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>

#include <rclcpp/rclcpp.hpp>
#include <ib2_msgs/msg/fan_status.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sim_msgs/srv/update_parameter.hpp>

#include <vector>
#include <array>
#include <cassert>

namespace thr_plugin
{
	/**
	 * @brief スラスタによる推力・トルクをGazeboに入力するプラグイン.
	 */
	class Thr : public gz::sim::System,
	            public gz::sim::ISystemConfigure,
	            public gz::sim::ISystemPreUpdate
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		Thr();
		~Thr() override;

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		Thr(const Thr&) = delete;
		Thr& operator=(const Thr&) = delete;
		Thr(Thr&&) = delete;
		Thr& operator=(Thr&&) = delete;

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
		/** ROS Parameter Serverからパラメータ取得 */
		void getParameter();

		/** 推力プラグインパラメータ更新 */
		void updateParameter(
			const std::shared_ptr<sim_msgs::srv::UpdateParameter::Request> req,
			std::shared_ptr<sim_msgs::srv::UpdateParameter::Response> res);

		/** 推力プラグインパラメータログ作成 */
		void logParameter();

		/** ファン駆動デューティ比のサブスクライバのコールバック関数 */
		void subFanDuty(const ib2_msgs::msg::FanStatus::SharedPtr msg);

		/** 制御コマンド設定 */
		void setCtlCmd(const geometry_msgs::msg::WrenchStamped::SharedPtr ctl);

		/** 力・トルクのGazeboへの設定 */
		void addForceAndTorque(gz::sim::EntityComponentManager &_ecm);

		/** 双二次フィルタ */
		double biQuadFilter(const gz::math::Vector3d& a, const gz::math::Vector3d& b,
		                    const double& in, double* ibuf, double* obuf, bool rst);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノード */
		std::shared_ptr<rclcpp::Node>              ros_node_;

		/** モデルエンティティ */
		gz::sim::Entity                            model_entity_{gz::sim::kNullEntity};

		/** リンクエンティティ */
		gz::sim::Entity                            link_entity_{gz::sim::kNullEntity};

		/** 駆動デューティ比のサブスクライバ */
		rclcpp::Subscription<ib2_msgs::msg::FanStatus>::SharedPtr sub_duty_;

		/** 制御コマンドのサブスクライバ */
		rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_wrench_;

		/** 各ファンの推力のパブリッシャ */
		rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_fan_force_;

		/** 推力プラグインパラメータ更新サービスサーバ */
		rclcpp::Service<sim_msgs::srv::UpdateParameter>::SharedPtr thr_param_server_;

		/** デバッグ用フラグ */
		bool                                       debug_{false};

		/** ファンの数 */
		int                                        fan_num_{0};

		/** 重心位置 */
		gz::math::Vector3d                         cg_;

		/** 各ファンの取り付け位置 */
		std::vector<gz::math::Vector3d>            fan_pos_;

		/** 各ファンの推力方向 */
		std::vector<gz::math::Vector3d>            fan_frc_vec_;

		/** 各ファンのトルクベクトル */
		std::vector<gz::math::Vector3d>            fan_trq_vec_;

		/** 各ファンの推力最大値[N] */
		std::vector<double>                        fan_frc_;

		/** ファン推力誤差の標準偏差 */
		std::vector<double>                        stddev_;

		/** 各ファンのドラッグトルク係数κ[Nm] */
		std::vector<double>                        fan_k_;

		/** PWMデューティ計算用係数 */
		std::vector<double>                        k_prop_;

		/** 各ファンの推力[N] */
		std::vector<double>                        f_;

		/** ファンによる合計推力 */
		gz::math::Vector3d                         force_;

		/** ファンによる合計トルク */
		gz::math::Vector3d                         torque_;

		/** 2次フィルタ係数a */
		gz::math::Vector3d                         coeff_a_{0.0, 0.0, 0.0};

		/** 2次フィルタ係数b */
		gz::math::Vector3d                         coeff_b_{0.0, 0.0, 0.0};

		/** 2次フィルタ保存値(入力) */
		std::vector<std::array<double, 2>>         in_;

		/** 2次フィルタ保存値(出力) */
		std::vector<std::array<double, 2>>         out_;
	};
}
// End Of File -----------------------------------------------------------------
