
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include "coordinate_transform/coordinate_transform.h"
#include "ib2_msgs/SwitchPower.h"
#include "ib2_msgs/PowerStatus.h"
#include "sim_msgs/UpdateParameter.h"
#include <cassert>

namespace gazebo
{
	/**
	 * @brief ドッキングステーションの磁力による吸引力やトルクを模擬するプラグイン.
	 */
	class Mag : public WorldPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Mag();

		/** デストラクタ. */
		~Mag();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Mag(const Mag&) = delete;

		/** コピー代入演算子. */
		Mag& operator=(const Mag&) = delete;

		/** ムーブコンストラクタ. */
		Mag(Mag&&) = delete;

		/** ムーブ代入演算子. */
		Mag& operator=(Mag&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** プラグインのロード
		 * @param [in, out] _world Worldへのポインタ
		 * @param [in, out] _sdf SDF要素へのポインタ
		 */
		virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

	private:
		/** ROS Parameter Serverからパラメータを取得
		 */
		void getParameter();

		/** 磁力プラグインパラメータ更新
		  * @param [in]                 req      パラメータ更新サービスリクエスト
		  * @param [in]                 res      パラメータ更新サービス実行結果
          * @retval                     true     更新成功
		  * @retval                     false    更新失敗
		  */
		bool updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res);

		/** 磁力プラグインパラメータログ作成
		 */
		void logParameter();

		/** ISS/IB2モデルを取得
		 */
		void getModels();

		/** Gazeboのコールバック関数
		 */
		void magCallBack();
		
		/** ドッキングステーション座標系(ホーム座標系)でのロボット位置・姿勢を取得
		 * @param [in, out] r_ds          ドッキングステーション座標系でのロボット位置[m]
		 * @param [in, out] q_ds          ドッキングステーション座標系に対するロボット姿勢クォータニオン
		 */
		void getDsPose(ignition::math::Vector3d& r_ds, ignition::math::Quaterniond& q_ds);

		/** 磁石による吸引力を取得
		 * @param [in]      r_ds          ドッキングステーション座標系(ホーム座標系)でのロボット位置[m]
		 * @param [in]      q_ds          ドッキングステーション座標系(ホーム座標系)に対するロボット姿勢クォータニオン
		 * @param [in, out] fmag_ds       ドッキングステーション座標系(ホーム座標系)での吸引力[N]
		 * @param [in, out] fmag_bd       機体座標系での吸引力[N]
		 */
		void getForce(
			const ignition::math::Vector3d& r_ds,    const ignition::math::Quaterniond& q_ds,
			      ignition::math::Vector3d& fmag_ds,       ignition::math::Vector3d&    fmag_bd
		);

		/** 磁石によるトルクを取得
		 * @param [in]      fmag_bd       機体座標系での吸引力[N]
		 * @param [in, out] tmag_bd       機体座標系でのトルク[N]
		 */
		void getTorque(const ignition::math::Vector3d& fmag_bd, ignition::math::Vector3d& tmag_bd);
		
		/** 吸引力・トルクを印加
		 * @param [in]      fmag_bd        機体座標系での吸引力[N]
		 * @param [in]      tmag_bd        機体座標系でのトルク[Nm]
		 */
		void addForceAndTorque(const ignition::math::Vector3d& fmag_bd, const ignition::math::Vector3d& tmag_bd);

		/** 吸引力・トルクをパブリッシュ
		 * @param [in]      fmag_ds        ドッキングステーション座標系(ホーム座標系)での吸引力[N]
		 * @param [in]      tmag_bd        機体座標系でのトルク[Nm]
		 */
		void pubForceAndTorque(const ignition::math::Vector3d& fmag_ds, const ignition::math::Vector3d& tmag_bd);

		/** 磁力ON/OFF
		* @param [in]                 req              磁力ON/OFFサービスリクエスト
		* @param [in]                 res              現在の磁力ON/OFFステータス
		* @retval                     true             設定成功
		* @retval                     false            設定失敗
		*/
		bool switchPower(
			ib2_msgs::SwitchPower::Request&  req,
			ib2_msgs::SwitchPower::Response& res
		);

		/** 単位ステップ関数
		 * @param [in]      x      入力値
		 * @return                 出力値    　
		 */
		bool U(double x);

		/** 吸引力関数
		 * @param [in]      dist   IF間距離 [m]
		 * @return                 吸引力   [N]    　
		 */
		double H(double dist);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                nh_;

		/** Worldポインタ */
		physics::WorldPtr              world_;

		/** ISSモデル名 */
		std::string                    iss_name_;

		/** IB2モデル名 */
		std::string                    ib2_name_;

		/** ISSモデルへのポインタ */
		physics::ModelPtr              iss_model_;

		/** ロボットモデルへのポインタ */
		physics::ModelPtr              ib2_model_;

		/** ISSモデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>  iss_link_;

		/** IB2モデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>  ib2_link_;

		/** ISS機体座標系でのJPM基準点の位置 */
		ignition::math::Vector3d                  jpm_pos_;

		/** ISS機体座標系でのJPMの姿勢 */
		ignition::math::Vector3d                  jpm_att_;

		/** JPM基準座標系でのドッキングステーション(ホーム座標系)原点 */
		ignition::math::Vector3d                  ds_pos_;

		/** JPM基準座標系でのドッキングステーション姿勢 */
		ignition::math::Vector3d                  ds_att_;

		/** 磁力プラグインパラメータ更新サービスサーバ */
		ros::ServiceServer             mag_param_server_;

		/** ロボット側の磁力IF点 */
		ignition::math::Vector3d                  rrif_;

		/** ドッキングステーション側の磁力IF点 */
		ignition::math::Vector3d                  rdif_;

		/** ロボット重心位置 */
		ignition::math::Vector3d                  cg_;

		/** 遠方域 - 近傍域係数切替距離 */
		double                         d_threshold_;

		/** 遠方域吸引力計算式係数 a */
		double                         afar_;

		/** 遠方域吸引力計算式係数 b */
		double                         bfar_;

		/** 遠方域吸引力計算式係数 c */
		double                         cfar_;

		/** 近傍域吸引力計算式係数 a */
		double                         aprox_;

		/** 近傍域吸引力計算式係数 b */
		double                         bprox_;

		/** 近傍域吸引力計算式係数 c */
		double                         cprox_;

		/** 吸引力・トルクのパブリッシャ */
		ros::Publisher                 pub_mag_;

		/** 磁力ON/OFFステータスのパブリッシャ */
		ros::Publisher                 pub_power_status_;

		/** 磁力誤差の標準偏差 */
		double                         stddev_;

		/** パブリッシュ周期 */
		double                         cycle_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr           update_;

		/** 座標変換オブジェクト */
		gazebo::CoordinateTransform    coord_transformer_;

		/** 磁力ON/OFFサービスサーバ */
		ros::ServiceServer             switch_power_server_;

		/** 磁力ON/OFFステータス */
		ib2_msgs::PowerStatus          power_status_;
	};
}
// End Of File -----------------------------------------------------------------

