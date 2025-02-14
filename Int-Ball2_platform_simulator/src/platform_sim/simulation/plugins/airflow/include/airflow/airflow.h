
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include "sim_msgs/UpdateParameter.h"
#include "coordinate_transform/coordinate_transform.h"

namespace gazebo
{
	/**
	 * @brief Air Flowによる外乱を模擬するプラグイン.
	 */
	class Airflow : public WorldPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Airflow();

		/** デストラクタ. */
		~Airflow();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Airflow(const Airflow&) = delete;

		/** コピー代入演算子. */
		Airflow& operator=(const Airflow&) = delete;

		/** ムーブコンストラクタ. */
		Airflow(Airflow&&) = delete;

		/** ムーブ代入演算子. */
		Airflow& operator=(Airflow&&) = delete;

		//----------------------------------------------------------------------
		// 実装
	public:
		/** プラグインのロード
		 * @param [in, out] _world Worldへのポインタ
		 * @param [in, out] _sdf SDF要素へのポインタ
		 */
		virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

	private:
		/** 頂点構造体
		 */
		struct Vertex
		{
			bool                    is_positive_x;
			bool                    is_positive_y;
			bool                    is_positive_z;
			double                  distance;
			pcl::PointWithViewpoint point;

			Vertex()
			{
				is_positive_x = false;
				is_positive_y = false;
				is_positive_z = false;
				distance      = 0.0;
				point.x       = 0.0;
				point.y       = 0.0;
				point.z       = 0.0;
				point.vp_x    = 0.0;
				point.vp_y    = 0.0;
				point.vp_z    = 0.0;
			}
		};

		/** ロボット状態量構造体(位置・速度・姿勢)
		 */
		 struct State
		 {
			 ignition::math::Vector3d    pos;
			 ignition::math::Vector3d    vel;
			 ignition::math::Quaterniond rot;

			 State()
			 {
				pos = ignition::math::Vector3d::Zero;
			 	vel = ignition::math::Vector3d::Zero;
			 	rot = ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0);
			 }
		 };

		/** ROS Parameter Serverからパラメータを取得
		 */
		void getParameter();

		/** エアフロープラグインパラメータ更新
		  * @param [in]                 req      パラメータ更新サービスリクエスト
		  * @param [in]                 res      パラメータ更新サービス実行結果
		  * @retval                     true     更新成功
		  * @retval                     false    更新失敗
		  */
		bool updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res);

		/** エアフロープラグインパラメータログ作成
		 */
		void logParameter();

		/** ISS/IB2モデルを取得
		 */
		void getModels();

		/** 抗力テーブル読み込み
		 */
		void readFdTable();

		/** airflowコールバック関数 
		 */
		void airflowCallBack();

		/** JPM基準座標系でのロボット状態量を取得 
		 */
		void getRobotState();

		/** 外部PCDファイルをロード
		 */
		void loadWindVecField();

		/** K近傍点のポイントインデックスを取得
		 */
		void getPointIndex();

		/** 外部PCDファイルからJEM船内風速ベクトル場を取得 
		 * @param [in]      point     K近傍探索する位置[m](JPM基準座標系)
		 * @param [out]     point_idx 最近傍メッシュのインデックスリスト
		 * @return 最近傍点数
		 */
		int nnSearch(const ignition::math::Vector3d& point, std::vector<int>& point_idx);

		/** 頂点探索
		 * @param [in]  point        立体内部の点の位置[m](JPM基準座標系)
		 * @param [in]  in_vertices   pointからのK-近傍点
		 * @param [out] out_vertices  pointを包含する(8つの)頂点
		 * @return      true : 成功 false : 失敗
		 */
		bool selectVertex(
			const ignition::math::Vector3d& point,
			const std::vector<pcl::PointWithViewpoint>& in_vertices, 
			      std::vector<pcl::PointWithViewpoint>& out_vertices
		);

		/** 線形補間
		 * @param [in] vertex pointを包含する8つの頂点
		 * @param [in] point  補間する位置
		 * @return 線形補間後の風速ベクトル
		 */
		ignition::math::Vector3d lerp(const std::vector<pcl::PointWithViewpoint>& vertex, const ignition::math::Vector3d& point);

		/** JPM基準座標系での風速ベクトル取得
		 * @param [in]      point    風速ベクトルを取得する位置[m](JPM基準座標系)
		 * @param [out]     wind_v   JPM基準座標系座標系での風速ベクトル
		 */
		void getWindVec(const ignition::math::Vector3d& point, ignition::math::Vector3d& wind_v);

		/** JPM基準座標系で抗力取得
		 * @param [in]      wind_v   JPM基準座標系座標系での風速ベクトル
		 * @param [out]     force    JPM基準座標系座標系での抗力ベクトル
		 */
		void getForce(const ignition::math::Vector3d& wind_v, ignition::math::Vector3d& force);

		/** 機体座標系で抗力取得
		 * @param [in]      wind_v   JPM基準座標系座標系での風速ベクトル
		 * @param [out]     torque   機体座標系でのトルク
		 */
		void getTorque(const std::vector<ignition::math::Vector3d>& wind_v, ignition::math::Vector3d& torque);

		/** エアフローによる抗力、トルクを印加する
		 * @param [in] force_body       機体座標系での力[N]
		 * @param [in] torque_body      機体座標系でのトルク[Nm]
		 */
		void addForceAndTorque(const ignition::math::Vector3d& force_body, const ignition::math::Vector3d& torque_body);

		/** エアフローによる抗力、トルクおよび風速ベクトルをパブリッシュ
		 * @param [in] f       JPM基準座標系座標系での抗力[N]
		 * @param [in] t       機体座標系でのトルク[Nm]
		 * @param [in] w       JPM基準座標系座標系での風速ベクトル
		 */
		void pubWrenchAndWind(const ignition::math::Vector3d& f, const ignition::math::Vector3d& t, const ignition::math::Vector3d& w);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                          nh_;

		/** Worldポインタ */
		physics::WorldPtr                        world_;

		/** ISSモデル名 */
		std::string                              iss_name_;

		/** IB2モデル名 */
		std::string                              ib2_name_;

		/** ISSモデルへのポインタ */
		physics::ModelPtr                        iss_model_;

		/** ロボットモデルへのポインタ */
		physics::ModelPtr                        ib2_model_;

		/** ISSモデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>            iss_link_;

		/** IB2モデルのリンクへのポインタ */
		std::vector<physics::LinkPtr>            ib2_link_;

		/** ISS ACSでのJPM基準点の位置 */
		ignition::math::Vector3d                            jpm_pos_;

		/** ISS ACSからJPM基準座標系へのオイラー角 */
		ignition::math::Vector3d                            jpm_att_;

		/** ロボットの重心位置 */
		ignition::math::Vector3d                            cg_;

		/** ロボットの半径 */
		double                                   radius_;

		/** 抗力のパブリッシャ */
		ros::Publisher                           pub_drag_;

		/** 風速のパブリッシャ */
		ros::Publisher                           pub_wind_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr                     update_;

		/** シミュレーション時刻 */
		double                                   sim_time_;

		/** PCDファイル読み込み周期 [ms] */
		int                                      time_step_;

		/** K近傍点探索のポイントインデックス更新周期 */
		double                                   get_cycle_;

		/** 抗力のパブリッシュ周期 */
		double                                   pub_cycle_;

		/** PCDファイル名（Prefix) */
		std::string                              pcd_;

		/** Fdテーブルファイル名 */
		std::string                              table_;

		/** Fd テーブル */
		std::vector<std::array<double, 2>>       fd_table_;

		/** ロボット重心に作用する風速ベクトル(JPM基準座標系) */
		ignition::math::Vector3d                            wind_v_cg_;

		/** ロボットの作用点に作用する風速ベクトル(JPM基準座標系) */
		std::vector<ignition::math::Vector3d>               wind_v_lp_;

		/** ロボット機体における抗力作用点 */
		std::vector<ignition::math::Vector3d>               load_point_;

		/** 抗力誤差の標準偏差 */
		double                                   force_stddev_;

		/** トルク誤差の標準偏差 */
		double                                   torque_stddev_;

		/** ドラッグドルク係数 */
		double                                   kappa_;

		/** JPM基準座標系でのロボット状態量 */
		struct State                             state_;

		/** Point Cloud(現在時刻) */
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_now_;

		/** Point Cloud(1 Step 後) */
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_next1_;

		/** Point Cloud(2 Step 後) */
		pcl::PointCloud<pcl::PointWithViewpoint> cloud_next2_;

		/** K-近傍点のポイントインデックス */
		std::vector<int>                         point_idx_;

		/** 座標変換オブジェクト */
		CoordinateTransform                      coord_transformer_;

		/** エアフロープラグインパラメータ更新サービスサーバ */
		ros::ServiceServer                       af_param_server_;
	};
}
// End Of File -----------------------------------------------------------------

