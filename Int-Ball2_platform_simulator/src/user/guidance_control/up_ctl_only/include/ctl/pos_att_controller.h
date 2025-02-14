
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/WrenchStamped.h>
#include "ib2_msgs/Navigation.h"
#include "ctl/pos_controller.h"
#include "ctl/att_controller.h"
#include "ctl/ctl_elements.h"

namespace ib2
{
	// classの前方宣言
	class CtlBody;
	
	/**
	 * @brief 位置制御則クラス
	 */
	class PosAttController
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		PosAttController();
		
		/** 値によるコンストラクタ
		 *@param [in] pos 位置制御パラメータ
		 *@param [in] att 姿勢制御パラメータ
		 */
		PosAttController(const PosController& pos, const AttController& att);
		
		/** デストラクタ */
		~PosAttController();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** コピーコンストラクタ. */
		PosAttController(const PosAttController&);
		
		/** コピー代入演算子. */
		PosAttController& operator=(const PosAttController&);
		
		/** ムーブコンストラクタ. */
		PosAttController(PosAttController&&);
		
		/** ムーブ代入演算子. */
		PosAttController& operator=(PosAttController&&);
		
		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** 位置制御パラメータの設定
		 * @param [in] p 位置制御パラメータ
		 */
		bool setConfigPos(const PosController& pos);
		
		/** 姿勢制御パラメータの設定
		 * @param [in] p 姿勢制御パラメータ
		 */
		bool setConfigAtt(const AttController& att);
		
		/** 位置制御積分量のクリア */
		void flash();
		
		//----------------------------------------------------------------------
		// 実装
	public:
		/** 制御停止時の力トルクコマンドの計算
		 * @param [in] t コマンド時刻
		 * @return 力トルクコマンドメッセージ
		 */
		geometry_msgs::WrenchStamped wrenchCommandStop(const ros::Time& t);
		
		/** 力トルクコマンドの計算
		 * @param [in] nav 航法値
		 * @param [in] p 制御目標値
		 * @param [in] b 機体質量特性
		 * @param [in] torque_gain Torque gain forward to body.(Option)
		 * @return 力トルクコマンドメッセージ
		 */
		geometry_msgs::WrenchStamped wrenchCommand
		(const ib2_msgs::Navigation& nav, 
		 const CtlElements& p, const CtlBody& b, const Eigen::Vector3d& torque_gain = Eigen::Vector3d::Zero(3));
		
		//----------------------------------------------------------------------
		// メンバ変数
	private:
		/** 力トルクコマンドシーケンスID */
		uint32_t seq_;
		
		/** 位置制御則パラメータ */
		PosController pos_;
		
		/** 姿勢制御則パラメータ */
		AttController att_;
	};
}

// End Of File -----------------------------------------------------------------

