
#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <actionlib/server/simple_action_server.h>
#include "ib2_msgs/CtlCommandAction.h"
#include "ib2_msgs/Navigation.h"
#include "ib2_msgs/NavigationStartUpAction.h"
#include "ib2_msgs/NavigationStatus.h"
#include "ib2_msgs/MarkerCorrection.h"
#include "ib2_msgs/SwitchPower.h"
#include "sim_msgs/Attitude.h"
#include "sim_msgs/UpdateParameter.h"
#include "coordinate_transform/coordinate_transform.h"
#include <cassert>
#include <queue>

namespace gazebo
{
	/**
	 * @brief Robotの状態量を取得し、航法値メッセージをpublishするプラグイン.
	 */
	class Nav : public WorldPlugin
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		Nav();

		/** デストラクタ. */
		~Nav();

		//----------------------------------------------------------------------
		// コピー/ムーブ
	private:
		/** コピーコンストラクタ. */
		Nav(const Nav&) = delete;

		/** コピー代入演算子. */
		Nav& operator=(const Nav&) = delete;

		/** ムーブコンストラクタ. */
		Nav(Nav&&) = delete;

		/** ムーブ代入演算子. */
		Nav& operator=(Nav&&) = delete;

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
		int getParameter();

		/** 航法プラグインパラメータ更新
		* @param [in]                 req      パラメータ更新サービスリクエスト
		* @param [in]                 res      パラメータ更新サービス実行結果
		* @retval                     true     更新成功
		* @retval                     false    更新失敗
		*/
		bool updateParameter(sim_msgs::UpdateParameter::Request& req, sim_msgs::UpdateParameter::Response& res);

		/** マーカー補正
		 * @param [in]                 req      マーカー補正サービスリクエスト
		 * @param [in]                 res      マーカー補正サービス実行結果
		 * @retval                     true     更新成功
		 * @retval                     false    更新失敗
		 */
		bool markerCorrection(ib2_msgs::MarkerCorrection::Request& req, ib2_msgs::MarkerCorrection::Response& res);

		/** Nav ON/OFF
		* @param [in]                 req              Nav ON/OFFサービスリクエスト
		* @param [in]                 res              現在のNav ON/OFFステータス
		* @retval                     true             設定成功
		* @retval                     false            設定失敗
		*/
		bool switchPower(
			ib2_msgs::SwitchPower::Request&  req,
			ib2_msgs::SwitchPower::Response& res
		);
		
		/** 航法誤差CSVファイルオープン */
		void openCSVFile();

		/** ISS/IB2モデルを取得
		 */
		void getModels();

		/** ROS Timerのコールバック関数
		 * @param [in, out] event TimerEvent構造体への参照
		 */
		void navCallBack(const ros::TimerEvent& event);

		/** ROS Timer（航法機能ステータス値（実機固有インタフェース）出力用）のコールバック関数
		 * @param [in, out] event TimerEvent構造体への参照
		 */
		void sensorFusionStatusCallBack(const ros::TimerEvent&);

		/** 航法ステータスのサブスクライバのコールバック関数
		 * @param [in] status 航法ステータス
		 */
		void statusCallback(const std_msgs::Int32& status);

		/** 航法ステータスのサブスクライバのコールバック関数
		 * @param [in] status 航法ステータス
		 */
		void offsetCallback(const std_msgs::Float64& offset);

		/** ロボットの航法値真値を取得する
		 * @return ib2_msgs::Navigation 航法値メッセージ
		 */
		ib2_msgs::Navigation getTrueNavigation();

		/** World(慣性)座標系からホーム座標系への座標変換
		 * @param [in, out] nav_msgs 航法値メッセージへの参照
		 * @return ib2_msgs::Navigation 航法値メッセージ
		 */
		ib2_msgs::Navigation transformWtoH(const ib2_msgs::Navigation& nav_msgs);

		/** 航法値に誤差を付加する
		 * @param [in, out] nav_msgs 航法値メッセージへの参照
		 * @return ib2_msgs::Navigation 誤差込み航法値メッセージ
		 */
		ib2_msgs::Navigation addError(const ib2_msgs::Navigation& nav_msgs);

		/** 制御周期の変動を模擬する
		 * @return double 変動込み制御周期[s]
		 */
		double controlFreqFluctuation();

		/** NavigationメッセージからAttitudeメッセージを作成する
		 * @param [in] nav_msgs       航法値メッセージへの参照
		 * @return sim_msgs::Attitude 姿勢情報メッセージ
		 */
		sim_msgs::Attitude makeAttMsgFromNavMsg(const ib2_msgs::Navigation& nav_msgs);

		/** 加速度、角加速度の加算（Gazeboのコールバック関数） */
		void sumAcclAndAttRate();


		/** 航法誤差
		 */
		struct NavError
		{
			ignition::math::Vector3d  pos;
			ignition::math::Vector3d  vel;
			ignition::math::Vector3d  acc;
			ignition::math::Vector3d  rot;
			ignition::math::Vector3d  w;

			NavError()
			{
				pos = ignition::math::Vector3d::Zero;
				vel = ignition::math::Vector3d::Zero;
				acc = ignition::math::Vector3d::Zero;
				rot = ignition::math::Vector3d::Zero;
				w   = ignition::math::Vector3d::Zero;
			}
		};

		/** 航法誤差を取得する
		 * @return NavError      航法誤差構造体
		 */
		NavError getNavError();

		/** 航法機能アクション（実機固有インタフェース）模擬　受信時の処理
		 * @param [in] request 受信メッセージ
		 */
		void navigationStartUpCallback(const ib2_msgs::NavigationStartUpGoalConstPtr& request);

		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** ROSノードハンドラ */
		ros::NodeHandle                nh_;

		/** Gazeboへの接続のためのポインタ */
		event::ConnectionPtr           update_;

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
		ignition::math::Vector3d          jpm_pos_;

		/** ISS機体座標系でのJPMの姿勢 */
		ignition::math::Vector3d          jpm_att_;

		/** JPM基準座標系でのドッキングステーション(ホーム座標系)原点 */
		ignition::math::Vector3d          ds_pos_;

		/** JPM基準座標系でのドッキングステーション姿勢 */
		ignition::math::Vector3d          ds_att_;

		/** 航法値(誤差込)のパブリッシャ */
		ros::Publisher                 pub_nav_;

		/** 航法機能ステータスのパブリッシャ（実機固有インタフェースの模擬） */
		ros::Publisher                 pub_sensor_fusion_status_;

		/** 姿勢状態量(誤差込)のパブリッシャ */
		ros::Publisher                 pub_att_;

		/** 航法値真値のパブリッシャ */
		ros::Publisher                 pub_true_nav_;

		/** 姿勢状態量真値のパブリッシャ */
		ros::Publisher                 pub_true_att_;

		/** 航法ステータスのサブスクライバ */
		ros::Subscriber                sub_status_;

		/** 航法時刻オフセットのサブスクライバ */
		ros::Subscriber                sub_time_offset_;

		/** 航法プラグインパラメータ更新サービスサーバ */
		ros::ServiceServer             nav_param_server_;

		/** マーカー補正サービスサーバ */
		ros::ServiceServer             marker_correction_server_;

		/** Nav ON/OFFサービスサーバ */
		ros::ServiceServer             switch_power_server_;

		/** ROSタイマ */
		ros::Timer                     timer_;
\
		/** ROSタイマ 航法機能ステータス値（実機固有インタフェース）出力用 */
		ros::Timer                     timer_sensor_fusion_status_;

		/** 航法機能アクション（実機固有インタフェース）スタブ */
		std::unique_ptr<actionlib::SimpleActionServer<ib2_msgs::NavigationStartUpAction>> navigation_start_up_;

		/** 航法メッセージバッファ */
		std::queue<ib2_msgs::Navigation> nav_buffer_;

		/** 航法遅れ時間 */
		double                         delay_;

		/* 航法誤差源 */
		bool                           error_source_csv_;

		/* 加速度・角速度累積カウンタ */
		int                            accum_counter_;

		/* 速度増分(累積加速度) */
		ignition::math::Vector3d       delta_v_;

		/* 角度増分(累積角速度) */
		ignition::math::Vector3d       delta_angle_;

		/**  位置バイアス誤差[m]*/
		ignition::math::Vector3d       bias_p_;

		/**  位置ランダム誤差[m]*/
		ignition::math::Vector3d       rand_p_;

		/**  速度バイアス誤差[m/s]*/
		ignition::math::Vector3d       bias_v_;

		/**  速度ランダム誤差[m/s]*/
		ignition::math::Vector3d       rand_v_;

		/**  加速度バイアス誤差[m/s^2]*/
		ignition::math::Vector3d       bias_a_;

		/**  加速度ランダム誤差[m/s^2]*/
		ignition::math::Vector3d       rand_a_;

		/**  姿勢バイアス誤差[deg]*/
		ignition::math::Vector3d       bias_r_;

		/**  姿勢ランダム誤差[deg]*/
		ignition::math::Vector3d       rand_r_;

		/**  角速度バイアス誤差[deg/s]*/
		ignition::math::Vector3d       bias_w_;

		/**  角速度ランダム誤差[deg/s]*/
		ignition::math::Vector3d       rand_w_;

		/**  ノミナル制御周期[Hz]*/
		double                         bias_cnt_;

		/**  制御周期ランダム誤差[Hz]*/
		double                         rand_cnt_;

		/**  制御周期変動ゲイン[-]*/
		double                         gain_cnt_;

		/**  制御周期変動周期[Hz]*/
		double                         freq_cnt_;

		/** 座標変換オブジェクト */
		gazebo::CoordinateTransform    coord_transformer_;

		/** 航法誤差CSVファイル名 */
		std::string                    csv_file_name_;
		
		/** 航法ステータス */
		uint8_t                        status_;
		
		/** 航法時刻オフセット */
		ros::Duration tnav_offset_;
		
		/** 航法データ異常値 */
		bool invalid_nav_;

		/** 航法出力の初期状態 **/
		bool initial_nav_on_;
	};
}
// End Of File -----------------------------------------------------------------

