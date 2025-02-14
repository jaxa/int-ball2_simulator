#include <algorithm>
#include <iterator>
#include <ros/ros.h>
#include "communication_software/Telemetry.h"
#include "ib2_msgs.h"
#include "platform_msgs.h"
#include "qdebug_custom.h"
#include "telemetry_telecommand_config.h"
#include "telemetry_subscriber.h"
#include "utils.h"

using namespace intball;

const std::string TelemetrySubscriber::TOPIC_NAME_INTBALL2 = "telemetry_intball2";
const std::string TelemetrySubscriber::TOPIC_NAME_DOCK = "telemetry_dock";

namespace
{
template<typename T>
inline void deserializeIntBall2TelemetryMessage(const communication_software::Message& message, T& output)
{
    uint32_t serializeSize = static_cast<uint32_t>(message.data.size());
    LOG_DEBUG() << "serializationLength: " << serializeSize;
    boost::shared_array<uint8_t> buffer(new uint8_t[serializeSize]);
    std::copy(message.data.begin(), message.data.end(), buffer.get());
    ros::serialization::IStream stream(buffer.get(), serializeSize);
    ros::serialization::deserialize(stream, output);
}
}

TelemetrySubscriber::TelemetrySubscriber(QObject* parent)
    : QObject(parent), telemetryIntBall2_(nullptr), telemetryDock_(nullptr)
{
    qRegisterMetaType<communication_software::Telemetry::ConstPtr>("communication_software::Telemetry::ConstPtr");

    connect(this, &TelemetrySubscriber::subscribedIntBall2,
            this, &TelemetrySubscriber::parseIntball2Telemetry, Qt::QueuedConnection);
    connect(this, &TelemetrySubscriber::subscribedDock,
            this, &TelemetrySubscriber::parseDockTelemetry, Qt::QueuedConnection);
}

TelemetrySubscriber::~TelemetrySubscriber()
{
    subscriberIntBall2_.shutdown();
}

void TelemetrySubscriber::start(ros::NodeHandle& nodeHandle, IntBallTelemetry* telemetryIntBall2, DockTelemetry* telemetryDock)
{
    Q_ASSERT(telemetryIntBall2_ == nullptr);
    Q_ASSERT(subscriberIntBall2_.getTopic().empty());
    Q_ASSERT(telemetryDock_ == nullptr);
    Q_ASSERT(subscriberDock_.getTopic().empty());

    telemetryIntBall2_ = telemetryIntBall2;
    subscriberIntBall2_ = nodeHandle.subscribe(TOPIC_NAME_INTBALL2, 1000,
                          &TelemetrySubscriber::intball2TelemetryCallback, this);

    telemetryDock_ = telemetryDock;
    subscriberDock_ = nodeHandle.subscribe(TOPIC_NAME_DOCK, 1000,
                          &TelemetrySubscriber::dockTelemetryCallback, this);
}

void TelemetrySubscriber::intball2TelemetryCallback(const communication_software::Telemetry::ConstPtr& msg)
{
    emit subscribedIntBall2(msg);
}

void TelemetrySubscriber::parseIntball2Telemetry(const communication_software::Telemetry::ConstPtr msg)
{
    telemetryIntBall2_->setReceivedTimestamp(msg->received_time);

    QMap<telemetry::Index, QVariant> subscribedTelemetry;
    std::string contentName;
    bool isCtlCommandResult = false;
    for(auto i = msg->data.begin(); i != msg->data.end(); ++i)
    {
        contentName = (*i).name;
        LOG_DEBUG() << "content_name: " << contentName;

        /*
         * テレメトリのヘッダ情報.
         */
        if(contentName == telemetry::rosname::TIMESTAMP)
        {
            // タイムスタンプ.
            std_msgs::Time timestamp;
            deserializeIntBall2TelemetryMessage((*i), timestamp);
            subscribedTelemetry.insert(telemetry::Index::TIMESTAMP, QVariant::fromValue(timestamp.data));
        }
        else if(contentName == telemetry::rosname::LAST_EXECUTED_COMMAND)
        {
            // 最後に実行したテレコマンドのID.
            std_msgs::UInt16 id;
            deserializeIntBall2TelemetryMessage((*i), id);
            subscribedTelemetry.insert(telemetry::Index::LAST_EXECUTED_COMMAND, QVariant::fromValue(id.data));
        }
        else if(contentName == telemetry::rosname::SPLIT_NUMBER)
        {
            // テレメトリの分割数.
            std_msgs::UInt8 data;
            deserializeIntBall2TelemetryMessage((*i), data);
            subscribedTelemetry.insert(telemetry::Index::SPLIT_NUMBER, QVariant::fromValue(data.data));
        }
        else if(contentName == telemetry::rosname::CURRENT_SPILIT_INDEX)
        {
            // 受信したテレメトリの分割数インデックス.
            std_msgs::UInt8 data;
            deserializeIntBall2TelemetryMessage((*i), data);
            subscribedTelemetry.insert(telemetry::Index::CURRENT_SPILIT_INDEX, QVariant::fromValue(data.data));
        }
        else if(contentName == telemetry::rosname::SENDING_PORT_INDEX)
        {
            // 送信ポートのインデックス.
            std_msgs::UInt8 data;
            deserializeIntBall2TelemetryMessage((*i), data);
            subscribedTelemetry.insert(telemetry::Index::SENDING_PORT_INDEX, QVariant::fromValue(data.data));
        }

        /*
         * テレメトリデータ本体.
         */
        else if(contentName == telemetry::rosname::TASK_MANAGER_MODE)
        {
            // 統合ソフトウェアモード.
            ib2_msgs::Mode mode;
            deserializeIntBall2TelemetryMessage((*i), mode);

            subscribedTelemetry.insert(telemetry::Index::MODE, QVariant::fromValue(mode.mode));
        }
        else if(contentName == telemetry::rosname::TASK_MANAGER_EXIT_DOCKING_MODE)
        {
            // ドッキングモード終了のレスポンス.
            ib2_msgs::ExitDockingModeResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::EXIT_DOCKING_MODE_SUCCESS, QVariant::fromValue(static_cast<bool>(response.success)));
            subscribedTelemetry.insert(telemetry::Index::EXIT_DOCKING_MODE_RESULT_MODE, QVariant::fromValue(response.mode.mode));
        }
        else if(contentName == telemetry::rosname::TASK_MANAGER_SET_MAINTENANCE_MODE)
        {
            // メンテナンスモード遷移のレスポンス.
            ib2_msgs::SetMaintenanceModeResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::SET_MAINTENANCE_MODE_RESULT_MODE, QVariant::fromValue(response.mode.mode));
        }
        else if(contentName == telemetry::rosname::NAVIGATION_STARTUP_FEEDBACK)
        {
            // 航法機能ON/OFFのフィードバック.
            ib2_msgs::NavigationStartUpActionFeedback feedback;
            deserializeIntBall2TelemetryMessage((*i), feedback);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_STARTUP_FEEDBACK_DURATION, QVariant::fromValue(feedback.feedback.time));
        }
        else if(contentName == telemetry::rosname::NAVIGATION_STARTUP_RESULT)
        {
            // 航法機能ON/OFFの結果.
            ib2_msgs::NavigationStartUpActionResult result;
            deserializeIntBall2TelemetryMessage((*i), result);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_STARTUP_RESULT_TIMESTAMP, QVariant::fromValue(result.result.stamp));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_STARTUP_RESULT_TYPE, QVariant::fromValue(result.result.type));
        }
        else if(contentName == telemetry::rosname::NAVIGATION)
        {
            // 航法値.
            ib2_msgs::Navigation navigation;
            deserializeIntBall2TelemetryMessage((*i), navigation);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_HEADER_SEQ, QVariant::fromValue(navigation.pose.header.seq));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_HEADER_STAMP, QVariant::fromValue(navigation.pose.header.stamp));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_HEADER_FRAME_ID, QVariant::fromValue(navigation.pose.header.frame_id));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_POSE_POSITION, QVariant::fromValue(navigation.pose.pose.position));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_POSE_ORIENTATION, QVariant::fromValue(navigation.pose.pose.orientation));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_TWIST_LINEAR, QVariant::fromValue(navigation.twist.linear));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_TWIST_ANGULAR, QVariant::fromValue(navigation.twist.angular));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_A, QVariant::fromValue(navigation.a));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_STATUS, QVariant::fromValue(navigation.status));
        }
        else if(contentName == telemetry::rosname::NAVIGATION_DEBUG)
        {
            // 航法値デバッグ情報.
            ib2_msgs::NavigationDebug navigationDebug;
            deserializeIntBall2TelemetryMessage((*i), navigationDebug);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_DEBUG_POINT, QVariant::fromValue(navigationDebug.point));
            QList<float> posOffsets;
            for(auto i = navigationDebug.mkr_pos_ofs.begin(); i != navigationDebug.mkr_pos_ofs.end(); ++i)
            {
                posOffsets.push_back(*i);
            }
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_DEBUG_POS_OFFSETS, QVariant::fromValue(posOffsets));
            QList<float> attOffsets;
            for(auto i = navigationDebug.mkr_att_ofs.begin(); i != navigationDebug.mkr_att_ofs.end(); ++i)
            {
                attOffsets.push_back(*i);
            }
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_DEBUG_ATT_OFFSETS, QVariant::fromValue(attOffsets));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_DEBUG_IMU_TEMPERATURE, QVariant::fromValue(navigationDebug.temperature));
        }
        else if(contentName == telemetry::rosname::NAVIGATION_STATUS_TOPIC)
        {
            // 航法機能の状態トピック.
            ib2_msgs::NavigationStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_STATUS_TOPIC_STASUS, QVariant::fromValue(status));
        }
        else if(contentName == telemetry::rosname::NAVIGATION_UPDATE_PARAMETER)
        {
            // 航法機能のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::PROP_UPDATE_PARAMETER)
        {
            // propのUpdateParameterのレスポンス
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::IMU_IMU)
        {
            // EPSONG370(IMUセンサ)の値
            ib2_msgs::IMU response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::IMU_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::IMU_ACC_X, QVariant::fromValue(response.acc_x));
            subscribedTelemetry.insert(telemetry::Index::IMU_ACC_Y, QVariant::fromValue(response.acc_y));
            subscribedTelemetry.insert(telemetry::Index::IMU_ACC_Z, QVariant::fromValue(response.acc_z));
            subscribedTelemetry.insert(telemetry::Index::IMU_GYRO_X, QVariant::fromValue(response.gyro_x));
            subscribedTelemetry.insert(telemetry::Index::IMU_GYRO_Y, QVariant::fromValue(response.gyro_y));
            subscribedTelemetry.insert(telemetry::Index::IMU_GYRO_Z, QVariant::fromValue(response.gyro_z));
            subscribedTelemetry.insert(telemetry::Index::IMU_TEMPERATURE, QVariant::fromValue(response.temperature));
        }
        else if(contentName == telemetry::rosname::IMU_UPDATE_PARAMETER)
        {
            // EPSONG370のUpdateParameterのレスポンス
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::IMU_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::SLAM_WRAPPER_UPDATE_PARAMETER)
        {
            // Slam wrapperのUpdateParameterのレスポンス
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::CTL_STATUS)
        {
            // 誘導制御ステータス.
            ib2_msgs::CtlStatus ctlStatus;
            deserializeIntBall2TelemetryMessage((*i), ctlStatus);

            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_HEADER_SEQ, QVariant::fromValue(ctlStatus.pose.header.seq));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_HEADER_STAMP, QVariant::fromValue(ctlStatus.pose.header.stamp));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_HEADER_FRAME_ID, QVariant::fromValue(ctlStatus.pose.header.frame_id));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_POSE_POSITION, QVariant::fromValue(ctlStatus.pose.pose.position));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_POSE_ORIENTATION, QVariant::fromValue(ctlStatus.pose.pose.orientation));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_TWIST_LINEAR, QVariant::fromValue(ctlStatus.twist.linear));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_TWIST_ANGULAR, QVariant::fromValue(ctlStatus.twist.angular));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_A, QVariant::fromValue(ctlStatus.a));
            subscribedTelemetry.insert(telemetry::Index::CTL_STATUS_TYPE, QVariant::fromValue(ctlStatus.type.type));
        }
        else if(contentName == telemetry::rosname::CTL_WRENCH)
        {
            // 誘導制御の力・トルク.
            geometry_msgs::WrenchStamped ctlWrench;
            deserializeIntBall2TelemetryMessage((*i), ctlWrench);

            subscribedTelemetry.insert(telemetry::Index::CTL_WRENCH_HEADER_SEQ, QVariant::fromValue(ctlWrench.header.seq));
            subscribedTelemetry.insert(telemetry::Index::CTL_WRENCH_HEADER_STAMP, QVariant::fromValue(ctlWrench.header.stamp));
            subscribedTelemetry.insert(telemetry::Index::CTL_WRENCH_HEADER_FRAME_ID, QVariant::fromValue(ctlWrench.header.frame_id));
            subscribedTelemetry.insert(telemetry::Index::CTL_WRENCH_FORCE, QVariant::fromValue(ctlWrench.wrench.force));
            subscribedTelemetry.insert(telemetry::Index::CTL_WRENCH_TORQUE, QVariant::fromValue(ctlWrench.wrench.torque));
        }
        else if(contentName == telemetry::rosname::CTL_ACTION_FEEDBACK)
        {
            // 誘導制御アクションのフィードバック.
            ib2_msgs::CtlCommandActionFeedback ctlActionFeedback;
            deserializeIntBall2TelemetryMessage((*i), ctlActionFeedback);

            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_ID,
                                       QVariant::fromValue(ctlActionFeedback.status.goal_id.id));
            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP,
                                       QVariant::fromValue(ctlActionFeedback.status.goal_id.stamp));
            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_FEEDBACK_TIME_TO_GO,
                                       QVariant::fromValue(ctlActionFeedback.feedback.time_to_go));
            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_FEEDBACK_POSE_POSITION,
                                       QVariant::fromValue(ctlActionFeedback.feedback.pose_to_go.position));
            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_FEEDBACK_POSE_ORIENTATION,
                                       QVariant::fromValue(ctlActionFeedback.feedback.pose_to_go.orientation));
        }
        else if(contentName == telemetry::rosname::CTL_ACTION_RESULT)
        {
            // 誘導制御アクションの結果.
            isCtlCommandResult = true;
            ib2_msgs::CtlCommandActionResult ctlActionResult;
            deserializeIntBall2TelemetryMessage((*i), ctlActionResult);

            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_RESULT_TIMESTAMP, QVariant::fromValue(ctlActionResult.result.stamp));
            subscribedTelemetry.insert(telemetry::Index::CTL_ACTION_RESULT_TYPE, QVariant::fromValue(ctlActionResult.result.type));
        }
        else if(contentName == telemetry::rosname::CTL_UPDATE_PARAMETER)
        {
            // 誘導制御のUpdateParameterのレスポンス
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::CTL_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::PROP_STATUS)
        {
            // プロペラ状態.
            ib2_msgs::FanStatus prop;
            deserializeIntBall2TelemetryMessage((*i), prop);

            subscribedTelemetry.insert(telemetry::Index::PROP_STATUS_HEADER_SEQ, QVariant::fromValue(prop.header.seq));
            subscribedTelemetry.insert(telemetry::Index::PROP_STATUS_HEADER_STAMP, QVariant::fromValue(prop.header.stamp));
            subscribedTelemetry.insert(telemetry::Index::PROP_STATUS_HEADER_FRAME_ID, QVariant::fromValue(prop.header.frame_id));
            subscribedTelemetry.insert(telemetry::Index::PROP_STATUS_DUTY, QVariant::fromValue(prop.duty.data));
            subscribedTelemetry.insert(telemetry::Index::PROP_STATUS_POWER, QVariant::fromValue(prop.current_power));
        }
        else if(contentName == telemetry::rosname::PROP_SWITCH_POWER)
        {
            // プロペラON/OFFのレスポンス.
            ib2_msgs::SwitchPowerResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::PROP_SWITCH_POWER_RESPONSE, QVariant::fromValue(response.current_power));
        }
        else if(contentName == telemetry::rosname::PROP_UPDATE_PARAMETER)
        {
            // プロペラのUpdateParameterのレスポンス
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::MARKER_CORRECTION)
        {
            // マーカー補正のレスポンス
            ib2_msgs::MarkerCorrectionResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::MARKER_CORRECTION_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::MARKER_CORRECTION_STATUS, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::CAMERA_MIC_STATUS)
        {
            // カメラ・マイクの状態.
            ib2_msgs::MainCameraStatus mainCameraStatus;
            deserializeIntBall2TelemetryMessage((*i), mainCameraStatus);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_STREAMING_STATUS, QVariant::fromValue(mainCameraStatus.streaming_status));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_RECORDING_STATUS, QVariant::fromValue(mainCameraStatus.recording_status));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_CAMERA_POWER, QVariant::fromValue(mainCameraStatus.camera_power));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_MICROPHONE_POWER, QVariant::fromValue(mainCameraStatus.microphone_power));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_ZOOM, QVariant::fromValue(mainCameraStatus.zoom));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_RESOLUTION_TYPE, QVariant::fromValue(mainCameraStatus.resolution_type));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_EV, QVariant::fromValue(mainCameraStatus.EV));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_CAMERA_GAIN, QVariant::fromValue(mainCameraStatus.camera_gain));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_WHITE_BALANCE_MODE, QVariant::fromValue(mainCameraStatus.white_balance_mode));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_FRAME_RATE, QVariant::fromValue(mainCameraStatus.frame_rate));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_SENDING_BIT_RATE, QVariant::fromValue(mainCameraStatus.sending_bit_rate));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_MICROPHONE_GAIN, QVariant::fromValue(mainCameraStatus.microphone_gain));
        }
        else if(contentName == telemetry::rosname::CAMERA_MIC_UPDATE_PARAMETER)
        {
            // カメラ・マイクのUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::LED_LEFT_UPDATE_PARAMETER)
        {
            // LED左のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::LED_RIGHT_UPDATE_PARAMETER)
        {
            // LED右のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(std::equal(std::begin(telemetry::rosname::LED_LEFT_LED_COLORS),
                           std::end(telemetry::rosname::LED_LEFT_LED_COLORS),
                           std::begin(contentName))) // split対象のテレメトリについては前方一致確認
        {
            // LED左の色.
            // ColorRGBAの配列が1要素ずつに分割されている.
            std_msgs::ColorRGBA color;
            deserializeIntBall2TelemetryMessage((*i), color);

            // LEDの色は、ColorRGBAの配列（LED0からLED7までの値を格納した配列）のインデックス0から順に
            // 下一桁0 ~ 7のID値が割り振られている想定。
            // ID値の下一桁から配列のインデックスを確定する。
            auto index = i->id % 10;

            if(index < 8)
            {
                 QVector<std_msgs::ColorRGBA> values(8);
                if(subscribedTelemetry.contains(telemetry::Index::LED_LEFT_LED_COLORS))
                {
                    values = subscribedTelemetry.value(telemetry::Index::LED_LEFT_LED_COLORS).value<QVector<std_msgs::ColorRGBA>>();
                }
                else
                {
                    values = telemetryIntBall2_->data(telemetry::Index::LED_LEFT_LED_COLORS).value<QVector<std_msgs::ColorRGBA>>();
                }
                values.replace(index, color);
                subscribedTelemetry.insert(telemetry::Index::LED_LEFT_LED_COLORS, QVariant::fromValue(values));
            }
            else
            {
                LOG_WARNING() << "Invalid index: name=" << contentName << " index=" << index;
            }
        }
        else if(std::equal(std::begin(telemetry::rosname::LED_RIGHT_LED_COLORS),
                           std::end(telemetry::rosname::LED_RIGHT_LED_COLORS),
                           std::begin(contentName))) // split対象のテレメトリについては前方一致確認
        {
            // LED右の色.
            // ColorRGBAの配列が1要素ずつに分割されている.
            std_msgs::ColorRGBA color;
            deserializeIntBall2TelemetryMessage((*i), color);

            // LEDの色は、ColorRGBAの配列（LED0からLED7までの値を格納した配列）のインデックス0から順に
            // 下一桁0 ~ 7のID値が割り振られている想定。
            // ID値の下一桁から配列のインデックスを確定する。
            auto index = i->id % 10;

            if(index < 8)
            {
                 QVector<std_msgs::ColorRGBA> values(8);
                if(subscribedTelemetry.contains(telemetry::Index::LED_RIGHT_LED_COLORS))
                {
                    values = subscribedTelemetry.value(telemetry::Index::LED_RIGHT_LED_COLORS).value<QVector<std_msgs::ColorRGBA>>();
                }
                else
                {
                    values = telemetryIntBall2_->data(telemetry::Index::LED_RIGHT_LED_COLORS).value<QVector<std_msgs::ColorRGBA>>();
                }
                values.replace(index, color);
                subscribedTelemetry.insert(telemetry::Index::LED_RIGHT_LED_COLORS, QVariant::fromValue(values));
            }
            else
            {
                LOG_WARNING() << "Invalid index: name=" << contentName << " Message.id=" << (i->id);
            }
        }
        else if(contentName == telemetry::rosname::DISPLAY_MANAGER_STATUS)
        {
            // 表示管理の状態.
            ib2_msgs::DisplayManagerStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);

            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_STATUS_MODE, QVariant::fromValue(status.operation_mode.mode));
            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_STATUS_COLOR, QVariant::fromValue(status.color));
            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_STATUS_CTL_STATUS_TYPE, QVariant::fromValue(status.ctl_status_type.type));
            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_STATUS_POWER, QVariant::fromValue(status.power));
            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_STATUS_FLASH, QVariant::fromValue(status.flash_mode));
        }
        else if(contentName == telemetry::rosname::DISPLAY_MANAGER_SWITCH_POWER)
        {
            // 表示管理ON/OFFのレスポンス
            ib2_msgs::SwitchPowerResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_SWITCH_POWER_RESPONSE, QVariant::fromValue(response.current_power));

        }
        else if(contentName == telemetry::rosname::DISPLAY_MANAGER_SWITCH_FLASH)
        {
            // 撮影用フラッシュON/OFFのレスポンス
            ib2_msgs::SwitchPowerResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_SWITCH_FLASH_RESPONSE, QVariant::fromValue(response.current_power));
        }
        else if(contentName == telemetry::rosname::DISPLAY_MANAGER_UPDATE_PARAMETER)
        {
            // 表示管理のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::PARAMETER_MANAGER_GET_ROS_PARAM)
        {
            // 単一ROSパラメータの取得.
            ib2_msgs::GetRosParamResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);
            // 追加のタイムスタンプ.
            ros::Time additionalTimestamp = (*i).additional_timestamp;

            // 処理結果フラグ.
            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::GET_ROS_PARAM_SUCCESS, QVariant::fromValue(static_cast<bool>(response.success)));

            // 取得データ.
            RosParam setParams;
            setParams.id = response.param.id;
            setParams.value = response.param.value;
            setParams.stamp = additionalTimestamp;
            subscribedTelemetry.insert(telemetry::Index::GET_ROS_PARAM_VALUES, QVariant::fromValue(setParams));
        }
        else if(std::equal(std::begin(telemetry::rosname::PARAMETER_MANAGER_GET_ROS_PARAMS),
                           std::end(telemetry::rosname::PARAMETER_MANAGER_GET_ROS_PARAMS),
                           std::begin(contentName))) // split対象のテレメトリについては前方一致確認
        {
            // 複数ROSパラメータの取得.
            // メッセージが分割されている.

            if(contentName.find(telemetry::rosname::SUFFIX_PARAMETER_MANAGER_GET_ROS_PARAMS_PARAMS) != std::string::npos)
            {
                // 取得したrosparamのリスト.
                ib2_msgs::RosParam param;
                deserializeIntBall2TelemetryMessage((*i), param);

                // 追加のタイムスタンプ.
                ros::Time additionalTimestamp = (*i).additional_timestamp;

                // 既存のリスト内容を追加・修正する.
                QMap<QString, RosParam> beforeValues;
                if(subscribedTelemetry.contains(telemetry::Index::GET_ROS_PARAMS_PARAMS_LIST))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::GET_ROS_PARAMS_PARAMS_LIST).value<QMap<QString, RosParam>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::GET_ROS_PARAMS_PARAMS_LIST).value<QMap<QString, RosParam>>();
                }
                RosParam setParams;
                setParams.id = param.id;
                setParams.value = param.value;
                setParams.stamp = additionalTimestamp;
                beforeValues.insert(QString::fromStdString(param.id), setParams);
                subscribedTelemetry.insert(telemetry::Index::GET_ROS_PARAMS_PARAMS_LIST, QVariant::fromValue(beforeValues));
            }
            else
            {
                LOG_WARNING() << "Invalid contentName (Name with invalid suffix): " << contentName;
            }
        }
        else if(contentName == telemetry::rosname::PARAMETER_MANAGER_SET_ROS_PARAM)
        {
            // 単一ROSパラメータの設定のレスポンス.
            ib2_msgs::SetRosParamResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);
            // 追加のタイムスタンプ.
            ros::Time additionalTimestamp = (*i).additional_timestamp;

            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::SET_ROS_PARAM_SUCCESS, QVariant::fromValue(static_cast<bool>(response.success)));
            subscribedTelemetry.insert(telemetry::Index::SET_ROS_PARAM_SUCCESS_TIMESTAMP, QVariant::fromValue(additionalTimestamp));
        }
        else if(std::equal(std::begin(telemetry::rosname::PARAMETER_MANAGER_SET_ROS_PARAMS),
                           std::end(telemetry::rosname::PARAMETER_MANAGER_SET_ROS_PARAMS),
                           std::begin(contentName)))
        {
            // 複数ROSパラメータの設定のレスポンス.
            // メッセージが分割されている.
            if(contentName.find(telemetry::rosname::SUFFIX_PARAMETER_MANAGER_SET_ROS_PARAMS_SUCCESS_ALL) != std::string::npos)
            {
                // 全てのrosparam設定に成功したかどうか.
                std_msgs::Bool flag;
                deserializeIntBall2TelemetryMessage((*i), flag);
                // 追加のタイムスタンプ.
                ros::Time additionalTimestamp = (*i).additional_timestamp;

                // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
                subscribedTelemetry.insert(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL, QVariant::fromValue(static_cast<bool>(flag.data)));
                subscribedTelemetry.insert(telemetry::Index::SET_ROS_PARAMS_SUCCESS_ALL_TIMESTAMP, QVariant::fromValue(additionalTimestamp));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_PARAMETER_MANAGER_SET_ROS_PARAMS_SUCCESS_PARAMS) != std::string::npos)
            {
                // 設定に成功したrosparamのリスト.
                ib2_msgs::RosParam param;
                deserializeIntBall2TelemetryMessage((*i), param);

                // 追加のタイムスタンプ.
                ros::Time additionalTimestamp = (*i).additional_timestamp;

                // 既存のリスト内容を追加・修正する.
                QMap<QString, RosParam> beforeValues;
                if(subscribedTelemetry.contains(telemetry::Index::SET_ROS_PARAMS_PARAMS_LIST))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::SET_ROS_PARAMS_PARAMS_LIST).value<QMap<QString, RosParam>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::SET_ROS_PARAMS_PARAMS_LIST).value<QMap<QString, RosParam>>();
                }
                RosParam setParams;
                setParams.id = param.id;
                setParams.value = param.value;
                setParams.stamp = additionalTimestamp;
                beforeValues.insert(QString::fromStdString(param.id), setParams);
                subscribedTelemetry.insert(telemetry::Index::SET_ROS_PARAMS_PARAMS_LIST, QVariant::fromValue(beforeValues));
            }
            else
            {
                LOG_WARNING() << "Invalid contentName (Name with invalid suffix): " << contentName;
            }
        }
        else if(contentName == telemetry::rosname::PARAMETER_MANAGER_DUMP_ROS_PARAMS)
        {
            // ROSパラメータのファイル出力処理のレスポンス.
            ib2_msgs::DumpRosParamsResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある
            subscribedTelemetry.insert(telemetry::Index::DUMP_ROS_PARAMS_SUCCESS, QVariant::fromValue(static_cast<bool>(response.success)));
        }
        else if(contentName == telemetry::rosname::PARAMETER_MANAGER_LOAD_ROS_PARAMS)
        {
            // ROSパラメータのファイル読込処理のレスポンス.
            ib2_msgs::LoadRosParamsResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::LOAD_ROS_PARAMS_SUCCESS, QVariant::fromValue(static_cast<bool>(response.success)));
        }
        else if(contentName == telemetry::rosname::PARAMETER_MANAGER_PUBLISHING_PARAMS)
        {
            // ROSパラメータのリスト.
            // 元のメッセージは配列だが,配列1要素単位に分割されている.
            ib2_msgs::RosParam param;
            deserializeIntBall2TelemetryMessage((*i), param);

            // 既存のリスト内容を追加・修正する.
            QMap<QString, QString> beforeValues;
            if(subscribedTelemetry.contains(telemetry::Index::PUBLISHING_PARAMS_LIST))
            {
                beforeValues = subscribedTelemetry.value(telemetry::Index::PUBLISHING_PARAMS_LIST).value<QMap<QString, QString>>();
            }
            else
            {
                beforeValues = telemetryIntBall2_->data(telemetry::Index::PUBLISHING_PARAMS_LIST).value<QMap<QString, QString>>();
            }
            beforeValues.insert(QString::fromStdString(param.id), QString::fromStdString(param.value));
            subscribedTelemetry.insert(telemetry::Index::PUBLISHING_PARAMS_LIST, QVariant::fromValue(beforeValues));
        }
        else if(std::equal(std::begin(telemetry::rosname::ALIVE_MONITOR_STATUSES),
                           std::end(telemetry::rosname::ALIVE_MONITOR_STATUSES),
                           std::begin(contentName)))
        {
            // 死活監視情報のリスト.
            // 元のメッセージは配列だが,配列1要素単位に分割されている.
            ib2_msgs::AliveStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);

            if(contentName.find(telemetry::rosname::SUFFIX_ALIVE_MONITOR_STATUSES_TOPIC) != std::string::npos)
            {
                //Topic.

                // 既存のリスト内容を追加・修正する.
                QMap<QString, ib2_msgs::AliveStatus> beforeValues;
                if(subscribedTelemetry.contains(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC).value<QMap<QString, ib2_msgs::AliveStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC).value<QMap<QString, ib2_msgs::AliveStatus>>();
                }
                ib2_msgs::AliveStatus setValue;
                setValue.name = status.name;
                setValue.result = status.result;
                setValue.check_time = status.check_time;
                setValue.latest_valid_time = status.latest_valid_time;
                beforeValues.insert(QString::fromStdString(setValue.name), setValue);
                subscribedTelemetry.insert(telemetry::Index::ALIVE_MONITOR_STATUSES_TOPIC, QVariant::fromValue(beforeValues));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_ALIVE_MONITOR_STATUSES_SERVICE) != std::string::npos)
            {
                //Service.

                // 既存のリスト内容を追加・修正する.
                QMap<QString, ib2_msgs::AliveStatus> beforeValues;
                if(subscribedTelemetry.contains(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE).value<QMap<QString, ib2_msgs::AliveStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE).value<QMap<QString, ib2_msgs::AliveStatus>>();
                }
                ib2_msgs::AliveStatus setValue;
                setValue.name = status.name;
                setValue.result = status.result;
                setValue.check_time = status.check_time;
                setValue.latest_valid_time = status.latest_valid_time;
                beforeValues.insert(QString::fromStdString(setValue.name), setValue);
                subscribedTelemetry.insert(telemetry::Index::ALIVE_MONITOR_STATUSES_SERVICE, QVariant::fromValue(beforeValues));
            }
            else
            {
                LOG_WARNING() << "Invalid contentName (Name with invalid suffix): " << contentName;
            }
        }
        else if(contentName == telemetry::rosname::SYSTEM_MONITOR_STATUS)
        {
            // システム監視.
            ib2_msgs::SystemStatus systemStatus;
            deserializeIntBall2TelemetryMessage((*i), systemStatus);

            QMap<QString, float> diskSpaces;
            for(auto i = systemStatus.disk_spaces.begin(); i != systemStatus.disk_spaces.end(); ++i)
            {
                diskSpaces.insert(QString::fromStdString((*i).path), (*i).remain);
            }
            subscribedTelemetry.insert(telemetry::Index::SYSTEM_MONITOR_TIMESTAMP, QVariant::fromValue(systemStatus.check_time));
            subscribedTelemetry.insert(telemetry::Index::SYSTEM_MONITOR_DISK_SPACES, QVariant::fromValue(diskSpaces));
            subscribedTelemetry.insert(telemetry::Index::SYSTEM_MONITOR_TEMPERATURE, QVariant::fromValue(systemStatus.temperature));
            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::SYSTEM_MONITOR_WIFI_CONNECTED, QVariant::fromValue(static_cast<bool>(systemStatus.wifi_connected)));
        }
        else if(contentName == telemetry::rosname::FILE_MONITOR_STATUS)
        {
            // ファイル監視情報.
            ib2_msgs::FileMonitoringStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);

            subscribedTelemetry.insert(telemetry::Index::FILE_MONITOR_CHECK_TIME, QVariant::fromValue(status.check_time));
            subscribedTelemetry.insert(telemetry::Index::FILE_MONITOR_TIMESYNC_LOG, QVariant::fromValue(status.timesync_log));
        }
        else if(contentName == telemetry::rosname::DOCK_BATTERY_CHARGE_INFO)
        {
            // バッテリー充電状態.
            ib2_msgs::BatteryChargeInfo info;
            deserializeIntBall2TelemetryMessage((*i), info);

            subscribedTelemetry.insert(telemetry::Index::DOCK_BATTERY_CHARGE_INFO_REMAIN, QVariant::fromValue(info.battery_remain));
        }
        else if(contentName == telemetry::rosname::NOT_ROS_FLIGHT_SOFTWARE_STATUS | contentName == telemetry::rosname::NOT_ROS_NORMAL_FLIGHT_SOFTWARE_STATUS)
        {
            // ROS外のソフトウェアから受信: Flight Softwareの起動状態
            std_msgs::Bool info;
            deserializeIntBall2TelemetryMessage((*i), info);

            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS, QVariant::fromValue(static_cast<bool>(info.data)));
        }

        // Platform GUI
        else if(contentName == telemetry::rosname::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS)
        {
            // ROS外のソフトウェアから受信: Platform Flight Softwareの起動状態
            std_msgs::Bool info;
            deserializeIntBall2TelemetryMessage((*i), info);

            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS, QVariant::fromValue(static_cast<bool>(info.data)));
        }
        else if(contentName == telemetry::rosname::PLATFORM_MANAGER_STATUS)
        {
            // プラットフォームマネージャの状態
            platform_msgs::ManagerStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);

            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_TIMESTAMP, QVariant::fromValue(status.stamp));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_OPERATION_TYPE, QVariant::fromValue(status.type.type));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_MODE, QVariant::fromValue(status.mode.mode));
            // ROSのC++ヘッダだとdataがuint8となっているため、boolにcastする必要がある.
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_START_CONTAINER, QVariant::fromValue(static_cast<bool>(status.start_container)));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_LAST_USER, QVariant::fromValue(status.last_user));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_LAST_LAUNCH, QVariant::fromValue(status.last_launch));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_LAST_IMAGE, QVariant::fromValue(status.last_image));
            subscribedTelemetry.insert(telemetry::Index::PLATFORM_MANAGER_LAST_LOGIC, QVariant::fromValue(status.last_user_logic.id));
        }
        else if(contentName == telemetry::rosname::SET_OPERATION_TYPE)
        {
            // ユーザノードのレスポンス.
            platform_msgs::SetOperationTypeResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::SET_OPERATION_TYPE_RESULT, QVariant::fromValue(response.result));
        }
        else if(std::equal(std::begin(telemetry::rosname::PLATFORM_MONITOR_STATUS),
                           std::end(telemetry::rosname::PLATFORM_MONITOR_STATUS),
                           std::begin(contentName))) // split対象のテレメトリについては前方一致確認
        {
            // プラットフォームの死活監視情報.
            // 元のメッセージは配列だが,配列1要素単位に分割されている.
            // 死活監視情報は、NodeStatusValueの配列のインデックス0から順に下一桁0 ~ 99のID値が割り振られている想定。
            // ID値の下一桁から配列のインデックスを確定する。

            // 追加のタイムスタンプ.
            ros::Time additionalTimestamp = (*i).additional_timestamp;

            if(contentName.find(telemetry::rosname::SUFFIX_PLATFORM_MONITOR_STATUS_CHECK_TIME) != std::string::npos)
            {
                // Check time
                ros::Time checkTime;
                deserializeIntBall2TelemetryMessage((*i), checkTime);
                subscribedTelemetry.insert(telemetry::Index::PLATFORM_MONITOR_CHECK_TIME, QVariant::fromValue(checkTime));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_PLATFORM_MONITOR_STATUS_PUBLICATIONS) != std::string::npos)
            {
                // Publicaitons.
                platform_msgs::NodeStatusValue nodeStatusValue;
                deserializeIntBall2TelemetryMessage((*i), nodeStatusValue);

                // 既存のリスト内容を追加・修正する.
                QVector<NodeStatus> beforeValues(100);
                if(subscribedTelemetry.contains(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS).value<QVector<NodeStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS).value<QVector<NodeStatus>>();
                }

                NodeStatus nodeStatus;
                nodeStatus.node = nodeStatusValue.node;
                nodeStatus.value = nodeStatusValue.value;
                nodeStatus.stamp = additionalTimestamp;
                beforeValues.replace(i->id % 100, nodeStatus);
                subscribedTelemetry.insert(telemetry::Index::PLATFORM_MONITOR_PUBLICATIONS, QVariant::fromValue(beforeValues));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_PLATFORM_MONITOR_STATUS_SUBSCRIPTIONS) != std::string::npos)
            {
                //Subscriptions.
                platform_msgs::NodeStatusValue nodeStatusValue;
                deserializeIntBall2TelemetryMessage((*i), nodeStatusValue);

                // 既存のリスト内容を追加・修正する.
                QVector<NodeStatus> beforeValues(100);
                if(subscribedTelemetry.contains(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS).value<QVector<NodeStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS).value<QVector<NodeStatus>>();
                }

                NodeStatus nodeStatus;
                nodeStatus.node = nodeStatusValue.node;
                nodeStatus.value = nodeStatusValue.value;
                nodeStatus.stamp = additionalTimestamp;
                beforeValues.replace(i->id % 100, nodeStatus);
                subscribedTelemetry.insert(telemetry::Index::PLATFORM_MONITOR_SUBSCRIPTIONS, QVariant::fromValue(beforeValues));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_PLATFORM_MONITOR_STATUS_SERVICES) != std::string::npos)
            {
                //Services.
                platform_msgs::NodeStatusValue nodeStatusValue;
                deserializeIntBall2TelemetryMessage((*i), nodeStatusValue);

                // 既存のリスト内容を追加・修正する.
                QVector<NodeStatus> beforeValues(100);
                if(subscribedTelemetry.contains(telemetry::Index::PLATFORM_MONITOR_SERVICES))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::PLATFORM_MONITOR_SERVICES).value<QVector<NodeStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::PLATFORM_MONITOR_SERVICES).value<QVector<NodeStatus>>();
                }

                NodeStatus nodeStatus;
                nodeStatus.node = nodeStatusValue.node;
                nodeStatus.value = nodeStatusValue.value;
                nodeStatus.stamp = additionalTimestamp;
                beforeValues.replace(i->id % 100, nodeStatus);
                subscribedTelemetry.insert(telemetry::Index::PLATFORM_MONITOR_SERVICES, QVariant::fromValue(beforeValues));
            }
            else if(contentName.find(telemetry::rosname::SUFFIX_PLATFORM_MONITOR_STATUS_CONTAINERS) != std::string::npos)
            {
                //Containers.
                platform_msgs::ContainerStatus containerStatusValue;
                deserializeIntBall2TelemetryMessage((*i), containerStatusValue);

                // 既存のリスト内容を追加・修正する.
                QVector<ContainerStatus> beforeValues(100);
                if(subscribedTelemetry.contains(telemetry::Index::PLATFORM_MONITOR_CONTAINERS))
                {
                    beforeValues = subscribedTelemetry.value(telemetry::Index::PLATFORM_MONITOR_CONTAINERS).value<QVector<ContainerStatus>>();
                }
                else
                {
                    beforeValues = telemetryIntBall2_->data(telemetry::Index::PLATFORM_MONITOR_CONTAINERS).value<QVector<ContainerStatus>>();
                }

                ContainerStatus containerStatus;
                containerStatus.id = containerStatusValue.id;
                containerStatus.image = containerStatusValue.image;
                containerStatus.status = containerStatusValue.status;
                containerStatus.stamp = additionalTimestamp;
                beforeValues.replace(i->id % 100, containerStatus);
                subscribedTelemetry.insert(telemetry::Index::PLATFORM_MONITOR_CONTAINERS, QVariant::fromValue(beforeValues));
            }
            else
            {
                LOG_WARNING() << "Invalid contentName (Name with invalid suffix): " << contentName;
            }
        }
        else if(contentName == telemetry::rosname::CAMERA_MAIN_STATUS)
        {
            // メインカメラの状態.
            platform_msgs::CameraStatus cameraMainStatus;
            deserializeIntBall2TelemetryMessage((*i), cameraMainStatus);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_MAIN_STREAMING_STATUS, QVariant::fromValue(cameraMainStatus.streaming_status));
        }
        else if(contentName == telemetry::rosname::CAMERA_LEFT_STATUS)
        {
            // 航法カメラ左の状態.
            platform_msgs::CameraStatus cameraLeftStatus;
            deserializeIntBall2TelemetryMessage((*i), cameraLeftStatus);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_LEFT_STREAMING_STATUS, QVariant::fromValue(cameraLeftStatus.streaming_status));
        }
        else if(contentName == telemetry::rosname::CAMERA_RIGHT_STATUS)
        {
            // 航法カメラ右の状態.
            platform_msgs::CameraStatus cameraRightStatus;
            deserializeIntBall2TelemetryMessage((*i), cameraRightStatus);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_RIGHT_STREAMING_STATUS, QVariant::fromValue(cameraRightStatus.streaming_status));
        }
        else if(contentName == telemetry::rosname::MICROPHONE_STATUS)
        {
            // マイクの状態.
            platform_msgs::MicrophoneStatus microphoneStatus;
            deserializeIntBall2TelemetryMessage((*i), microphoneStatus);

            subscribedTelemetry.insert(telemetry::Index::MICROPHONE_STREAMING_STATUS, QVariant::fromValue(microphoneStatus.streaming_status));
        }
        else if(contentName == telemetry::rosname::CAMERA_MAIN_UPDATE_PARAMETER)
        {
            // メインカメラのUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::CAMERA_LEFT_UPDATE_PARAMETER)
        {
            // 航法カメラ左のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::CAMERA_RIGHT_UPDATE_PARAMETER)
        {
            // 航法カメラ右のUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::MICROPHONE_UPDATE_PARAMETER)
        {
            // マイクのUpdateParameterのレスポンス.
            ib2_msgs::UpdateParameterResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::MICROPHONE_UPDATE_PARAMETER_RESPONSE_TIMESTAMP, QVariant::fromValue(response.stamp));
            subscribedTelemetry.insert(telemetry::Index::MICROPHONE_UPDATE_PARAMETER_RESPONSE_RESULT, QVariant::fromValue(response.status));
        }
        else if(contentName == telemetry::rosname::USER_NODE_STATUS)
        {
            // ユーザノードの状態.
            platform_msgs::UserNodeStatus status;
            deserializeIntBall2TelemetryMessage((*i), status);
            std::vector<char> msg(std::begin(status.msg), std::end(status.msg));

            subscribedTelemetry.insert(telemetry::Index::USER_NODE_STATUS_TIMESTAMP, QVariant::fromValue(status.stamp));
            subscribedTelemetry.insert(telemetry::Index::USER_NODE_STATUS_MESSAGE, QVariant::fromValue(QVector<char>::fromStdVector(msg)));
        }
        else if(contentName == telemetry::rosname::PLATFORM_MANAGER_USER_NODE)
        {
            // ユーザノードのレスポンス.
            platform_msgs::UserNodeCommandResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::USER_NODE_RESULT, QVariant::fromValue(response.result));
        }
        else if(contentName == telemetry::rosname::PLATFORM_MANAGER_USER_LOGIC)
        {
            // ユーザロジックのレスポンス.
            platform_msgs::UserLogicCommandResponse response;
            deserializeIntBall2TelemetryMessage((*i), response);

            subscribedTelemetry.insert(telemetry::Index::USER_LOGIC_RESULT, QVariant::fromValue(response.result));
        }
    }
    telemetryIntBall2_->setData(subscribedTelemetry);
}

void TelemetrySubscriber::dockTelemetryCallback(const communication_software::Telemetry::ConstPtr& msg)
{
    emit subscribedDock(msg);
}

void TelemetrySubscriber::parseDockTelemetry(const communication_software::Telemetry::ConstPtr msg)
{
    telemetryDock_->setReceivedTimestamp(msg->received_time);

    // ドッキングステーションのテレメトリは単一メッセージの想定.
    auto targetMessage = *(msg->data.begin());

    QMap<dock::telemetry::Index, QVariant> subscribedTelemetry;
    unsigned char buffUnsinedChar;
    unsigned short buffUnsignedShort;
    short buffShort;

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::TELEMETRY_COUNTER])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::TELEMETRY_COUNTER, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::COMMAND_COUNTER])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::COMMAND_COUNTER, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::CHARGE_STATE])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::CHARGE_STATE, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::MOTOR_STATE])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::MOTOR_STATE, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::SWITCH_STATE])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::SWITCH_STATE, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::COMMAND_STATE])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::COMMAND_STATE, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::VERSION])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::VERSION, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffShort, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::MOTOR_TEMP])), sizeof(buffShort));
    subscribedTelemetry.insert(dock::telemetry::Index::MOTOR_TEMP, QVariant::fromValue(static_cast<double>(buffShort) / 100.0));

    memcpy(&buffShort, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::DCDC_TEMP])), sizeof(buffShort));
    subscribedTelemetry.insert(dock::telemetry::Index::DCDC_TEMP, QVariant::fromValue(static_cast<double>(buffShort) / 100.0));

    memcpy(&buffUnsignedShort, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::POWER1_CURRENT])), sizeof(buffUnsignedShort));
    subscribedTelemetry.insert(dock::telemetry::Index::POWER1_CURRENT, QVariant::fromValue(static_cast<double>(buffUnsignedShort) / 100.0));

    memcpy(&buffUnsignedShort, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::POWER2_CURRENT])), sizeof(buffUnsignedShort));
    subscribedTelemetry.insert(dock::telemetry::Index::POWER2_CURRENT, QVariant::fromValue(static_cast<double>(buffUnsignedShort) / 100.0));

    memcpy(&buffUnsinedChar, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::TEMP_ALERT])), sizeof(buffUnsinedChar));
    subscribedTelemetry.insert(dock::telemetry::Index::TEMP_ALERT, QVariant::fromValue(buffUnsinedChar));

    memcpy(&buffUnsignedShort, &(targetMessage.data.at(dock::telemetry::DataOffset[dock::telemetry::Index::CHARGE_TIME])), sizeof(buffUnsignedShort));
    subscribedTelemetry.insert(dock::telemetry::Index::CHARGE_TIME, QVariant::fromValue(buffUnsignedShort));

    telemetryDock_->setData(subscribedTelemetry);
}
