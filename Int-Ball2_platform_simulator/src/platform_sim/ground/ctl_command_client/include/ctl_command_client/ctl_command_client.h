
#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "ib2_msgs/CtlCommandAction.h"

#include <string>

/**
 * @brief プログラムの実行を管理する.
 */
class CtlCommandClient
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** デフォルトコンストラクタ. */
	CtlCommandClient();

	/** デストラクタ. */
	~CtlCommandClient();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	CtlCommandClient(const CtlCommandClient&) = delete;

	/** コピー代入演算子. */
	CtlCommandClient& operator=(const CtlCommandClient&) = delete;

	/** ムーブコンストラクタ. */
	CtlCommandClient(CtlCommandClient&&) = delete;

	/** ムーブ代入演算子. */
	CtlCommandClient& operator=(CtlCommandClient&&) = delete;

	//----------------------------------------------------------------------
	// 操作(Setter)
private:
	/** 制御目標設定ファイルによる設定
	 * @param [in] filename 制御目標設定ファイル名
	 */
	void setFromFile(const std::string& filename);

	/** rosparamによる設定 */
	void setFromParam();

	//----------------------------------------------------------------------
	// 実装
public:
	/** プログラム実行 */
	void execute();

private:
	/** アクションサーバー起動待ち */
	void waitServer();

	/** 制御目標の送信 */
	void sendGoal();

	/** アクション結果の待機 */
	void waitForResult();

	/** アクションのキャンセル
	 * @param [in] info キャンセルの原因情報
	 */
	void cancel(const std::string& info);

	//----------------------------------------------------------------------
	// 実装（コールバック関数）
public:
	/** アクション終了時の処理
	 * @param [in] state 終了時の状態
	 * @param [in] result アクション実行結果
	 */
	void doneCb(const actionlib::SimpleClientGoalState& state,
				const ib2_msgs::CtlCommandResultConstPtr& result);

	/** アクション有効時の処理 */
	void activeCb();

	/** アクションのフィードバック受信時の処理
	 * @param [in] feedback アクションのフィードバック
	 */
	void feedbackCb(const ib2_msgs::CtlCommandFeedbackConstPtr& feedback);

	//----------------------------------------------------------------------
	// メンバー変数
protected:
	/** ノードハンドラ */
	ros::NodeHandle nh_;

	/** アクションクライアント */
	actionlib::SimpleActionClient<ib2_msgs::CtlCommandAction> ac_;

	/** コマンド種別 */
	uint8_t type_;

	/** タイムアウト[sec] */
	double timeout_;

	/** 制御目標位置X成分 */
	double drx_;

	/** 制御目標位置Y成分 */
	double dry_;

	/** 制御目標位置Z成分 */
	double drz_;

	/** 制御目標姿勢ヨー角[deg] */
	double dyaw_;

	/** 制御目標姿勢ピッチ角[deg] */
	double dpitch_;

	/** 制御目標姿勢ロール角[deg] */
	double droll_;

	/** 姿勢制御クォータニオンx成分 */
	double dqx_;
	
	/** 姿勢制御クォータニオンy成分 */
	double dqy_;

	/** 姿勢制御クォータニオンz成分 */
	double dqz_;

	/** 姿勢制御クォータニオンスカラー成分 */
	double dqw_;
};
// End Of File -----------------------------------------------------------------
