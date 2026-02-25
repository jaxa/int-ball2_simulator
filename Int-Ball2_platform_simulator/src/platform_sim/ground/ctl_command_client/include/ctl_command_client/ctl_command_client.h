
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "ib2_msgs/action/ctl_command.hpp"

#include <string>

/**
 * @brief プログラムの実行を管理する.
 */
class CtlCommandClient : public rclcpp::Node
{
	using CtlCommandAction = ib2_msgs::action::CtlCommand;
	using GoalHandle = rclcpp_action::ClientGoalHandle<CtlCommandAction>;

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
private:
	/** ゴール応答時の処理
	 * @param [in] goal_handle ゴールハンドル
	 */
	void goalResponseCb(GoalHandle::SharedPtr goal_handle);

	/** アクションのフィードバック受信時の処理
	 * @param [in] goal_handle ゴールハンドル
	 * @param [in] feedback アクションのフィードバック
	 */
	void feedbackCb(GoalHandle::SharedPtr goal_handle,
					const std::shared_ptr<const CtlCommandAction::Feedback> feedback);

	/** アクション結果受信時の処理
	 * @param [in] result アクション実行結果
	 */
	void resultCb(const GoalHandle::WrappedResult& result);

	//----------------------------------------------------------------------
	// メンバー変数
private:
	/** アクションクライアント */
	rclcpp_action::Client<CtlCommandAction>::SharedPtr ac_;

	/** ゴールハンドル */
	GoalHandle::SharedPtr goal_handle_;

	/** 完了フラグ */
	bool done_;

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
