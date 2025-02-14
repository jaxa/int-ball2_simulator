
#pragma once

#include <ib2_msgs/FanStatus.h>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <geometry_msgs/WrenchStamped.h>
#include <ros/ros.h>
#include "sim_msgs/UpdateParameter.h"
#include <cassert>

namespace gazebo
{
	/**
	 * @brief スラスタによる推力・トルクをGazeboに入力するプラグイン.
	 */
	class Thr : public ModelPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Thr();

		/** デストラクタ. */
		~Thr();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Thr(const Thr&) = delete;

		/** コピー代入演算子. */
		Thr& operator=(const Thr&) = delete;

		/** ムーブコンストラクタ. */
		Thr(Thr&&) = delete;

		/** ムーブ代入演算子. */
		Thr& operator=(Thr&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** プラグインのロード
		 * @param [in, out] _model モデルへのポインタ
		 * @param [in, out] _sdf SDF要素へのポインタ
		 */
		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

		/** 定期的な処理(FanStatusパブリッシュ用)
		 * @param [in] ev タイマーイベント
		 */
		void timerCallback(const ros::TimerEvent& ev);

	private:
		/** ROS Parameter Serverからパラメータ取得 */
		void getParameter();

		/** 推力プラグインパラメータ更新
		* @param [in]                 req      パラメータ更新サービスリクエスト
		* @param [in]                 res      パラメータ更新サービス実行結果
		* @retval                     true     更新成功
		* @retval                     false    更新失敗
		*/
		bool updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res);

		/** 推力プラグインパラメータログ作成 */
		void logParameter();

		/** ファン駆動デューティ比のサブスクライバのコールバック関数
		 * @param [in] msg ファン駆動デューティ比メッセージ
		 */
		void subFanDuty(const ib2_msgs::FanStatus& msg);


		/** 制御コマンド設定
		 * （制御コマンドのサブスクライバのコールバック関数）
		 * @param [in] ctl 制御コマンド
		 */
		void setCtlCmd(const geometry_msgs::WrenchStamped& ctl);

		/** 力・トルクのGazeboへの設定（Gazeboのコールバック関数）
		 * CallBack Function to add force and torque in Gazebo cycle
		 */
		void addForceAndTorque();

		/** 双二次フィルタ
		 * @param [in] a フィルタ係数a
		 * @param [in] b フィルタ係数b
		 * @param [in] in フィルタ入力値
		 * @param [in, out] ibuf 入力値保存値
		 * @param [in, out] obuf 出力値保存値
		 * @param [in] rst フィルタリセットフラグ
		 * @return double フィルタ出力値
		 */
		double biQuadFilter(const ignition::math::Vector3d& a, const ignition::math::Vector3d& b, const double& in, double* ibuf, double* obuf, bool rst);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                            nh_;

		/** モデルへのポインタ */
		physics::ModelPtr                          model_;

		/** モデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>              link_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr                       update_;

		/** 駆動デューティ比のサブスクライバ */
		ros::Subscriber                            sub_duty_;

		/** 制御コマンドのサブスクライバ */
		ros::Subscriber                            sub_wrench_;

		/** 各ファンの推力のパブリッシャ */
		ros::Publisher                             pub_fan_force_;

		/** ステータス出力タイマー */
		ros::Timer                                 timer_;

		/** デバッグ用フラグ */
		bool                                       debug_;

		/** 推力プラグインパラメータ更新サービスサーバ */
		ros::ServiceServer                         thr_param_server_;

		/** ファンの数 */
		int                                        fan_num_;

		/** 重心位置 */
		ignition::math::Vector3d                   cg_;

		/** 各ファンの取り付け位置 */
		std::vector<ignition::math::Vector3d>      fan_pos_;

		/** 各ファンの推力方向 */
		std::vector<ignition::math::Vector3d>      fan_frc_vec_;

		/** 各ファンのトルクベクトル  */
		std::vector<ignition::math::Vector3d>      fan_trq_vec_;

		/** 各ファンの推力最大値[N] */
		std::vector<double>                        fan_frc_;

		/** ファン推力誤差（ランダムノイズ）の標準偏差 */
		std::vector<double>                        stddev_;

		/** 各ファンのドラッグトルク係数κ[Nm] */
		std::vector<double>                        fan_k_;

		/** PWMデューティ計算用係数[-/√N] */
		std::vector<double>                        k_prop_;

		/** 各ファンの推力[N] */
		std::vector<double>                        f_;

		/** ファンによる合計推力 */
		ignition::math::Vector3d                   force_;

		/** ファンによる合計トルク */
		ignition::math::Vector3d                   torque_;

		/** 2次フィルタ係数a */
		ignition::math::Vector3d                   coeff_a_;

		/** 2次フィルタ係数b */
		ignition::math::Vector3d                   coeff_b_;

		/** 2次フィルタ保存値(入力) */
		std::vector<std::array<double, 2>>         in_;

		/** 2次フィルタ保存値(出力) */
		std::vector<std::array<double, 2>>         out_;
	};
}
// End Of File -----------------------------------------------------------------

