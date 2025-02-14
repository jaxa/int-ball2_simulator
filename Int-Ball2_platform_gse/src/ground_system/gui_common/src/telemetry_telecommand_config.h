#ifndef TELEMETRY_TELECOMMAND_CONFIG_H
#define TELEMETRY_TELECOMMAND_CONFIG_H
#include <string>
#include <vector>
#include <QMap>
#include <QString>
#include <QVariant>
#include <geometry_msgs/PoseStamped.h>
#include <rviz/display.h>
#include <std_msgs/Time.h>
#include "ib2_msgs.h"
#include "platform_msgs.h"
#include "ros_related_type_definitions.h"

namespace intball
{

/*
 * テレメトリ,テレコマンド共通設定
 */

static const unsigned char INTBALL_MODE_UNKNOWN = 255u;
static const QMap<unsigned char, QString> MODE_TYPE_LABEL =
{
    {ib2_msgs::Mode::STANDBY,          "STANDBY"},
    {ib2_msgs::Mode::MAINTENANCE,      "MAINTENANCE"},
    {ib2_msgs::Mode::RELEASE,          "RELEASE"},
    {ib2_msgs::Mode::DOCKING,          "DOCKING"},
    {ib2_msgs::Mode::OPERATION,        "OPERATION"},
    {ib2_msgs::Mode::RECOVERY,         "RECOVERY"},
    {ib2_msgs::Mode::IDLING,           "IDLING"},
    {ib2_msgs::Mode::OFF_NOMINAL,      "OFF_NOMINAL"},
    {INTBALL_MODE_UNKNOWN,             "UNKNOWN"} // GUI用の未定義値.
};

static const QMap<unsigned char, QString> POWER_STATUS_LABEL
{
    {ib2_msgs::PowerStatus::ON,       "ON"},
    {ib2_msgs::PowerStatus::OFF,      "OFF"},
    {ib2_msgs::PowerStatus::UNKNOWN,  "UNKNOWN"},
};

static const QMap<unsigned char, QString> MAIN_CAMERA_WHITE_BALANCE_MODE_LABEL
{
    {ib2_msgs::MainCameraWhiteBalanceMode::AUTO,             "AUTO"},
    {ib2_msgs::MainCameraWhiteBalanceMode::INCANDESCENT,     "INCANDESCENT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::FLUORESCENT,      "FLUORESCENT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::WARM_FLUORESCENT, "WARM_FLUORESCENT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::DAYLIGHT,         "DAYLIGHT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::CLOUDY_DAYLIGHT,  "CLOUDY_DAYLIGHT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::TWILIGHT,         "TWILIGHT"},
    {ib2_msgs::MainCameraWhiteBalanceMode::SHADE,            "SHADE"},
    {ib2_msgs::MainCameraWhiteBalanceMode::MANUAL,           "MANUAL"},
    {ib2_msgs::MainCameraWhiteBalanceMode::OFF,              "OFF"},
};

static const unsigned char PLATFORM_MODE_UNKNOWN = 255u;
static const QMap<unsigned char, QString> PLATFORM_MODE_TYPE_LABEL =
{
    {platform_msgs::Mode::USER_OFF,         "USER_OFF"},
    {platform_msgs::Mode::USER_READY,       "USER_READY"},
    {platform_msgs::Mode::USER_IN_PROGRESS, "USER_IN_PROGRESS"},
    {PLATFORM_MODE_UNKNOWN,                 "UNKNOWN"} // GUI用の未定義値.
};

static const unsigned char PLATFORM_OPERATION_TYPE_UNKNOWN = 255u;
static const QMap<unsigned char, QString> PLATFORM_OPERATION_TYPE_LABEL =
{
    {platform_msgs::OperationType::NAV_ON,  "NAV_ON"},
    {platform_msgs::OperationType::NAV_OFF, "NAV_OFF"},
    {PLATFORM_OPERATION_TYPE_UNKNOWN,       "UNKNOWN"} // GUI用の未定義値.
};

namespace rosparam
{
    static const QString PREFIX_CTL = "/ctl/";
    static const QString PREFIX_ATT_CTL = "/pos_ctl/";
    static const QString PREFIX_POS_CTL = "/att_ctl/";
    static const QString PREFIX_SENSOR_FUSION = "/sensor_fusion/";
    static const QString PREFIX_LED_LEFT = "/led_display_left/";
    static const QString PREFIX_LED_RIGHT = "/led_display_right/";

    static const std::string CTL_POS_KP = "/pos_ctl/kp";
    static const std::string CTL_POS_KI = "/pos_ctl/ki";
    static const std::string CTL_POS_KD = "/pos_ctl/kd";
    static const std::string CTL_ATT_KP = "/att_ctl/kp";
    static const std::string CTL_ATT_KD = "/att_ctl/kd";
    static const std::string CTL_TOLERANCE_POS = "/ctl/tolerance_pos";
    static const std::string CTL_TOLERANCE_ATT = "/ctl/tolerance_att";
    static const std::string CTL_DURATION_GOAL = "/ctl/duration_goal";
    static const std::string NAVIGATION_Q_POSITION_DIAG_X = "sensor_fusion/Q_position_diag_x";
    static const std::string NAVIGATION_Q_POSITION_DIAG_V = "/sensor_fusion/Q_position_diag_v";
    static const std::string NAVIGATION_R_POSITION_DIAG_X = "/sensor_fusion/R_position_diag_x";
    static const std::string NAVIGATION_R_POSITION_DIAG_V = "/sensor_fusion/R_position_diag_v";
    static const std::string NAVIGATION_Q_ANGLE_DIAG_Q = "/sensor_fusion/Q_angle_diag_q";
    static const std::string NAVIGATION_Q_ANGLE_DIAG_B = "/sensor_fusion/Q_angle_diag_b";
    static const std::string NAVIGATION_R_ANGLE_DIAG = "/sensor_fusion/R_angle_diag";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_EV = "/camera_and_microphone/camera_EV";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_GAIN = "/camera_and_microphone/camera_gain";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_ZOOM = "/camera_and_microphone/camera_zoom";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_WHITEBALANCE = "/camera_and_microphone/camera_white_balance_mode";
    static const std::string CAMERA_ANC_MICROPHONE_RESOLUTION = "/camera_and_microphone/resolution_type";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_FRAME_RATE = "/camera_and_microphone/frame_rate";
    static const std::string CAMERA_ANC_MICROPHONE_CAMERA_BIT_RATE = "/camera_and_microphone/bit_rate";
    static const std::string CAMERA_ANC_MICROPHONE_MICROPHONE_GAIN = "/camera_and_microphone/microphone_gain";
    static const std::string LED_LEFT_GAINS = "/led_display_left/gains";
    static const std::string LED_RIGHT_GAINS = "/led_display_right/gains";
    static const std::string CAMERA_MAIN_STREAMING_ON = "/camera_main/streaming_on";
    static const std::string CAMERA_LEFT_STREAMING_ON = "/camera_left/streaming_on";
    static const std::string CAMERA_RIGHT_STREAMING_ON = "/camera_right/streaming_on";
    static const std::string MICROPHONE_STREAMING_ON = "/microphone/streaming_on";
} // namespace rosparam

namespace telecommand
{
static const std::string NAME_REBOOT = "/trans_communication/reboot";
static const std::string NAME_CTL_ACTION_GOAL = "/trans_communication/action_goal";
static const std::string NAME_NAVIGATION_STARTUP = "/sensor_fusion/navigation_start_up/goal";
static const std::string NAME_CAMERA_ANC_MICROPHONE_RECORD = "/camera_and_microphone/record";
static const std::string NAME_CTL_DUTY = "/ctl/duty";
static const std::string NAME_SET_ROSPARAM = "/parameter_manager/set_ros_param";
static const std::string NAME_SET_ROSPARAMS = "/parameter_manager/set_ros_params";
static const std::string NAME_GET_ROSPARAM = "/parameter_manager/get_ros_param";
static const std::string NAME_GET_ROSPARAMS = "/parameter_manager/get_ros_params";
static const std::string NAME_EXIT_DOCKING_MODE = "/task_manager/exit_docking_mode";
static const std::string NAME_SET_MAINTENANCE_MODE = "/task_manager/set_maintenance_mode";
static const std::string NAME_DUMP_ROSPARAMS = "/parameter_manager/dump_ros_params";
static const std::string NAME_LOAD_ROSPARAMS = "/parameter_manager/load_ros_params";
static const std::string NAME_MARKER_CORRECTION = "/sensor_fusion/marker_correction";
static const std::string NAME_LED_DISPLAY_LEFT_COLORS = "/led_display_left/led_colors";
static const std::string NAME_LED_DISPLAY_RIGHT_COLORS = "/led_display_right/led_colors";
static const std::string NAME_DISPLAY_MANAGER_SWITCH = "/display_manager/switch_power";
static const std::string NAME_DISPLAY_MANAGER_FLASH = "/display_manager/switch_flash";
static const std::string NAME_FORCED_RELEASE = "/task_manager/force_release";
static const std::string NAME_CTL_ACTION_CANCEL = "/ctl/command/cancel";
static const std::string NAME_USER_NODE = "/platform_manager/user_node";
static const std::string NAME_USER_LOGIC = "/platform_manager/user_logic";
static const std::string NAME_SET_OPERATION_TYPE = "/platform_manager/set_operation_type";
/**
 * @brief Rosparam再読み込みの指定.
 */
enum class UPDATE_PARAMETER_TARGET
{
    CTL,
    SENSOR_FUSION,
    PROP,
    IMU,
    SLAM_WRAPPER,
    CAMERA_AND_MICROPHONE,
    LED_LEFT,
    LED_RIGHT,
    DISPLAY_MANAGER,
    CAMERA_MAIN,
    CAMERA_LEFT,
    CAMERA_RIGHT,
    MICROPHONE
};


static const QMap<UPDATE_PARAMETER_TARGET, std::string> UPDATE_PARAMETER_SERVICE_NAME
{
    {UPDATE_PARAMETER_TARGET::CTL,                              "/ctl/update_params"},
    {UPDATE_PARAMETER_TARGET::SENSOR_FUSION,                    "/sensor_fusion/update_params"},
    {UPDATE_PARAMETER_TARGET::PROP,                             "/prop/update_params"},
    {UPDATE_PARAMETER_TARGET::IMU,                              "/imu/update_params"},
    {UPDATE_PARAMETER_TARGET::SLAM_WRAPPER,                     "/slam_wrapper/update_params"},
    {UPDATE_PARAMETER_TARGET::CAMERA_AND_MICROPHONE,            "/camera_and_microphone/update_params"},
    {UPDATE_PARAMETER_TARGET::LED_LEFT,                         "/led_display_left/update_params"},
    {UPDATE_PARAMETER_TARGET::LED_RIGHT,                        "/led_display_right/update_params"},
    {UPDATE_PARAMETER_TARGET::DISPLAY_MANAGER,                  "/display_manager/update_params"},
    {UPDATE_PARAMETER_TARGET::CAMERA_MAIN,                      "/camera_main/update_params"},
    {UPDATE_PARAMETER_TARGET::CAMERA_LEFT,                      "/camera_left/update_params"},
    {UPDATE_PARAMETER_TARGET::CAMERA_RIGHT,                     "/camera_right/update_params"},
    {UPDATE_PARAMETER_TARGET::MICROPHONE,                       "/microphone/update_params"},
};

static const QMap<int, QString> CTL_ACTION_TYPE_LABEL =
{
    {ib2_msgs::CtlStatusType::STAND_BY,                  "STAND_BY"},
    {ib2_msgs::CtlStatusType::KEEP_POSE,                 "KEEP_POSE"},
    {ib2_msgs::CtlStatusType::STOP_MOVING,               "STOP_MOVING"},
    {ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET,   "MOVE_TO_RELATIVE_TARGET"},
    {ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET,   "MOVE_TO_ABSOLUTE_TARGET"},
    {ib2_msgs::CtlStatusType::RELEASE,                   "RELEASE"},
    {ib2_msgs::CtlStatusType::DOCK,                      "DOCK"},
    {ib2_msgs::CtlStatusType::DOCK_WITHOUT_CORRECTION,   "DOCK_WITHOUT_CORRECTION"},
    {ib2_msgs::CtlStatusType::SCAN,                      "SCAN"},
};

/**
 * @brief ON/OFFの送信先.
 */
enum class SWITCH_POWER_TARGET
{
    PROP,
    CAMERA_STREAMING,
    DEVICE_CAMERA,
    DEVICE_MICROPHONE,
    DISPLAY_MANAGER_FLASH,
};

static const QMap<SWITCH_POWER_TARGET, std::string> SWITCH_POWER_SERVICE_NAME
{
    {SWITCH_POWER_TARGET::PROP,                     "/prop/switch_power"},
    {SWITCH_POWER_TARGET::CAMERA_STREAMING,         "/camera_and_microphone/switch_power"},
    {SWITCH_POWER_TARGET::DEVICE_CAMERA,            "/camera_and_microphone/camera/switch_power"},
    {SWITCH_POWER_TARGET::DEVICE_MICROPHONE,        "/camera_and_microphone/microphone/switch_power"},
    {SWITCH_POWER_TARGET::DISPLAY_MANAGER_FLASH,    "/display_manager/switch_flash"},
};

} // namespace telecommand

namespace telemetry
{
namespace rosname
{
const std::string TIMESTAMP = "timestamp";
const std::string LAST_EXECUTED_COMMAND = "last_executed_command";
const std::string SPLIT_NUMBER = "split_number";
const std::string CURRENT_SPILIT_INDEX = "current_split_index";
const std::string SENDING_PORT_INDEX = "sending_port_index";
const std::string TASK_MANAGER_MODE = "/task_manager/mode";
const std::string TASK_MANAGER_EXIT_DOCKING_MODE = "/task_manager/exit_docking_mode";
const std::string TASK_MANAGER_SET_MAINTENANCE_MODE = "/task_manager/set_maintenance_mode";
const std::string CTL_STATUS = "/ctl/status";
const std::string CTL_WRENCH = "/ctl/wrench";
const std::string CTL_ACTION_FEEDBACK = "/ctl/command/feedback";
const std::string CTL_ACTION_RESULT = "/ctl/command/result";
const std::string CTL_UPDATE_PARAMETER = "/ctl/update_params";
const std::string PROP_STATUS = "/prop/status";
const std::string PROP_SWITCH_POWER = "/prop/switch_power";
const std::string PROP_UPDATE_PARAMETER = "/prop/update_params";
const std::string NAVIGATION = "/sensor_fusion/navigation";
const std::string NAVIGATION_DEBUG = "/sensor_fusion/navigation_debug";
const std::string NAVIGATION_STARTUP_FEEDBACK = "/sensor_fusion/navigation_start_up/feedback";
const std::string NAVIGATION_STARTUP_RESULT = "/sensor_fusion/navigation_start_up/result";
const std::string NAVIGATION_UPDATE_PARAMETER = "/sensor_fusion/update_params";
const std::string NAVIGATION_STATUS_TOPIC = "/sensor_fusion/navigation_status";
const std::string IMU_IMU = "/imu/imu";
const std::string IMU_UPDATE_PARAMETER = "/imu/update_params";
const std::string SLAM_WRAPPER_UPDATE_PARAMETER = "/slam_wrapper/update_params";
const std::string MARKER_CORRECTION = "/sensor_fusion/marker_correction";
const std::string CAMERA_MIC_RECORD = "/camera_and_microphone/camera/record";
const std::string CAMERA_MIC_STATUS = "/camera_and_microphone/status";
const std::string CAMERA_MIC_UPDATE_PARAMETER = "/camera_and_microphone/update_params";
const std::string LED_LEFT_UPDATE_PARAMETER = "/led_display_left/update_params";
const std::string LED_RIGHT_UPDATE_PARAMETER = "/led_display_right/update_params";
const std::string LED_LEFT_LED_COLORS = "/led_display_left/led_colors";
const std::string LED_RIGHT_LED_COLORS = "/led_display_right/led_colors";
const std::string DISPLAY_MANAGER_STATUS = "/display_manager/status";
const std::string DISPLAY_MANAGER_SWITCH_POWER = "/display_manager/switch_power";
const std::string DISPLAY_MANAGER_SWITCH_FLASH = "/display_manager/switch_flash";
const std::string DISPLAY_MANAGER_UPDATE_PARAMETER = "/display_manager/update_params";
const std::string PARAMETER_MANAGER_GET_ROS_PARAM = "/parameter_manager/get_ros_param";
const std::string PARAMETER_MANAGER_GET_ROS_PARAMS = "/parameter_manager/get_ros_params";
const std::string PARAMETER_MANAGER_SET_ROS_PARAM = "/parameter_manager/set_ros_param";
const std::string PARAMETER_MANAGER_SET_ROS_PARAMS = "/parameter_manager/set_ros_params";
const std::string SUFFIX_PARAMETER_MANAGER_SET_ROS_PARAMS_SUCCESS_ALL = ".success_all";
const std::string SUFFIX_PARAMETER_MANAGER_SET_ROS_PARAMS_SUCCESS_PARAMS = ".success_params";
const std::string SUFFIX_PARAMETER_MANAGER_GET_ROS_PARAMS_PARAMS = ".params";
const std::string PARAMETER_MANAGER_LOAD_ROS_PARAMS = "/parameter_manager/load_ros_params";
const std::string PARAMETER_MANAGER_DUMP_ROS_PARAMS = "/parameter_manager/dump_ros_params";
const std::string PARAMETER_MANAGER_PUBLISHING_PARAMS = "/parameter_manager/publishing_params.params";
const std::string SYSTEM_MONITOR_STATUS = "/system_monitor/status";
const std::string ALIVE_MONITOR_STATUSES = "/alive_monitor/statuses";
const std::string SUFFIX_ALIVE_MONITOR_STATUSES_TOPIC = ".topic_statuses";
const std::string SUFFIX_ALIVE_MONITOR_STATUSES_SERVICE = ".service_statuses";
const std::string FILE_MONITOR_STATUS = "/file_monitor/status";
const std::string DOCK_BATTERY_CHARGE_INFO = "/dock/battery_charge_info";
const std::string NOT_ROS_FLIGHT_SOFTWARE_STATUS = "/not_ros/flight_software_status";
const std::string NOT_ROS_NORMAL_FLIGHT_SOFTWARE_STATUS = "/not_ros/normal_flight_software_status";
const std::string NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS = "/not_ros/platform_flight_software_status";
const std::string PLATFORM_MANAGER_STATUS = "/platform_manager/status";
const std::string PLATFORM_MONITOR_STATUS = "/platform_monitor/status";
const std::string SUFFIX_PLATFORM_MONITOR_STATUS_CHECK_TIME = ".check_time";
const std::string SUFFIX_PLATFORM_MONITOR_STATUS_PUBLICATIONS = ".publications";
const std::string SUFFIX_PLATFORM_MONITOR_STATUS_SUBSCRIPTIONS = ".subscriptions";
const std::string SUFFIX_PLATFORM_MONITOR_STATUS_SERVICES = ".services";
const std::string SUFFIX_PLATFORM_MONITOR_STATUS_CONTAINERS = ".containers";
const std::string USER_NODE_STATUS = "/ib2_user/status";
const std::string PLATFORM_MANAGER_USER_NODE = "/platform_manager/user_node";
const std::string PLATFORM_MANAGER_USER_LOGIC = "/platform_manager/user_logic";
const std::string SET_OPERATION_TYPE = "/platform_manager/set_operation_type";
const std::string CAMERA_MAIN_STATUS = "/camera_main/status";
const std::string CAMERA_MAIN_UPDATE_PARAMETER = "/camera_main/update_params";
const std::string CAMERA_LEFT_STATUS = "/camera_left/status";
const std::string CAMERA_LEFT_UPDATE_PARAMETER = "/camera_left/update_params";
const std::string CAMERA_RIGHT_STATUS = "/camera_right/status";
const std::string CAMERA_RIGHT_UPDATE_PARAMETER = "/camera_right/update_params";
const std::string MICROPHONE_STATUS = "/microphone/status";
const std::string MICROPHONE_UPDATE_PARAMETER = "/microphone/update_params";
} // namespace rosname

static const int GUIDANCE_CONTROL_TYPE_UNKNOWN = -1;
static const QMap<int, QString> GUIDANCE_CONTROL_TYPE_LABEL =
{
    {ib2_msgs::CtlStatusType::STAND_BY,                  "STAND_BY"},
    {ib2_msgs::CtlStatusType::CAPTURED,                  "CAPTURED"},
    {ib2_msgs::CtlStatusType::DISTURBED,                 "DISTURBED"},
    {ib2_msgs::CtlStatusType::KEEP_POSE,                 "KEEP_POSE"},
    {ib2_msgs::CtlStatusType::KEEPING_POSE_BY_COLLISION, "KEEPING_POSE_BY_COLLISION"},
    {ib2_msgs::CtlStatusType::STOP_MOVING,               "STOP_MOVING"},
    {ib2_msgs::CtlStatusType::MOVE_TO_RELATIVE_TARGET,   "MOVE_TO_RELATIVE_TARGET"},
    {ib2_msgs::CtlStatusType::MOVE_TO_ABSOLUTE_TARGET,   "MOVE_TO_ABSOLUTE_TARGET"},
    {ib2_msgs::CtlStatusType::RELEASE,                   "RELEASE"},
    {ib2_msgs::CtlStatusType::DOCK,                      "DOCK"},
    {ib2_msgs::CtlStatusType::DOCK_WITHOUT_CORRECTION,   "DOCK_WITHOUT_CORRECTION"},
    {ib2_msgs::CtlStatusType::MOVING_TO_AIA_AIP,         "MOVING_TO_AIA_AIP"},
    {ib2_msgs::CtlStatusType::WAITING_AT_AIA_AIP,        "WAITING_AT_AIA_AIP"},
    {ib2_msgs::CtlStatusType::MOVING_TO_RDA_AIP,         "MOVING_TO_RDA_AIP"},
    {ib2_msgs::CtlStatusType::MOVING_TO_RDP,             "MOVING_TO_RDP"},
    {ib2_msgs::CtlStatusType::DOCKING_STAND_BY,          "DOCKING_STAND_BY"},
    {ib2_msgs::CtlStatusType::SCAN,                      "SCAN"},
    {GUIDANCE_CONTROL_TYPE_UNKNOWN,                      "UNKNOWN"},    // GUI向けの無効値を定義.
};

static const QMap<unsigned char, QString> GUIDANCE_CONTROL_ACTION_RESULT_LABEL =
{
    {ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS,        "SUCCESS"},
    {ib2_msgs::CtlCommandResult::TERMINATE_ABORTED,        "ABORTED"},
    {ib2_msgs::CtlCommandResult::TERMINATE_TIME_OUT,       "TIME_OUT"},
    {ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD,    "INVALID_CMD"},
    {ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV,    "INVALID_NAV"},
};

/**
 * @brief Int-Ballテレメトリ用データ項目のインデックス.
 */
enum class Index
{
    TIMESTAMP,
    LAST_EXECUTED_COMMAND,
    SPLIT_NUMBER,
    CURRENT_SPILIT_INDEX,
    SENDING_PORT_INDEX,
    MODE,
    EXIT_DOCKING_MODE_SUCCESS,
    EXIT_DOCKING_MODE_RESULT_MODE,
    SET_MAINTENANCE_MODE_RESULT_MODE,
    NAVIGATION_STARTUP_FEEDBACK_DURATION,
    NAVIGATION_STARTUP_RESULT_TIMESTAMP,
    NAVIGATION_STARTUP_RESULT_TYPE,
    NAVIGATION_HEADER_SEQ,
    NAVIGATION_HEADER_STAMP,
    NAVIGATION_HEADER_FRAME_ID,
    NAVIGATION_POSE_POSITION,
    NAVIGATION_POSE_ORIENTATION,
    NAVIGATION_TWIST_LINEAR,
    NAVIGATION_TWIST_ANGULAR,
    NAVIGATION_A,
    NAVIGATION_STATUS,
    NAVIGATION_STATUS_TOPIC_STASUS,
    MARKER_CORRECTION_TIMESTAMP,
    MARKER_CORRECTION_STATUS,
    NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT,
    NAVIGATION_DEBUG_POINT,
    NAVIGATION_DEBUG_POS_OFFSETS,
    NAVIGATION_DEBUG_ATT_OFFSETS,
    NAVIGATION_DEBUG_IMU_TEMPERATURE,
    IMU_TIMESTAMP,
    IMU_ACC_X,
    IMU_ACC_Y,
    IMU_ACC_Z,
    IMU_GYRO_X,
    IMU_GYRO_Y,
    IMU_GYRO_Z,
    IMU_TEMPERATURE,
    IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    IMU_UPDATE_PARAMETER_RESPONSE_RESULT,
    SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT,
    CTL_STATUS_HEADER_SEQ,
    CTL_STATUS_HEADER_STAMP,
    CTL_STATUS_HEADER_FRAME_ID,
    CTL_STATUS_POSE_POSITION,
    CTL_STATUS_POSE_ORIENTATION,
    CTL_STATUS_TWIST_LINEAR,
    CTL_STATUS_TWIST_ANGULAR,
    CTL_STATUS_A,
    CTL_STATUS_TYPE,
    CTL_WRENCH_HEADER_SEQ,
    CTL_WRENCH_HEADER_STAMP,
    CTL_WRENCH_HEADER_FRAME_ID,
    CTL_WRENCH_FORCE,
    CTL_WRENCH_TORQUE,
    CTL_ACTION_FEEDBACK_STATUS_GOAL_ID,
    CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP,
    CTL_ACTION_FEEDBACK_TIME_TO_GO,
    CTL_ACTION_FEEDBACK_POSE_POSITION,
    CTL_ACTION_FEEDBACK_POSE_ORIENTATION,
    CTL_ACTION_RESULT_TIMESTAMP,
    CTL_ACTION_RESULT_TYPE,
    CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    CTL_UPDATE_PARAMETER_RESPONSE_RESULT,
    PROP_STATUS_HEADER_SEQ,
    PROP_STATUS_HEADER_STAMP,
    PROP_STATUS_HEADER_FRAME_ID,
    PROP_STATUS_DUTY,
    PROP_STATUS_POWER,
    PROP_SWITCH_POWER_RESPONSE,
    PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    PROP_UPDATE_PARAMETER_RESPONSE_RESULT,
    CAMERA_MIC_STREAMING_STATUS,
    CAMERA_MIC_RECORDING_STATUS,
    CAMERA_MIC_CAMERA_POWER,
    CAMERA_MIC_MICROPHONE_POWER,
    CAMERA_MIC_ZOOM,
    CAMERA_MIC_RESOLUTION_TYPE,
    CAMERA_MIC_EV,
    CAMERA_MIC_CAMERA_GAIN,
    CAMERA_MIC_WHITE_BALANCE_MODE,
    CAMERA_MIC_FRAME_RATE,
    CAMERA_MIC_SENDING_BIT_RATE,
    CAMERA_MIC_MICROPHONE_GAIN,
    CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT,
    LED_LEFT_LED_COLORS,
    LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT,
    LED_RIGHT_LED_COLORS,
    LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT,
    DISPLAY_MANAGER_STATUS_MODE,
    DISPLAY_MANAGER_STATUS_CTL_STATUS_TYPE,
    DISPLAY_MANAGER_STATUS_COLOR,
    DISPLAY_MANAGER_STATUS_POWER,
    DISPLAY_MANAGER_STATUS_FLASH,
    DISPLAY_MANAGER_SWITCH_POWER_RESPONSE,
    DISPLAY_MANAGER_SWITCH_FLASH_RESPONSE,
    DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_RESULT,
    GET_ROS_PARAM_SUCCESS,
    GET_ROS_PARAM_VALUES,
    GET_ROS_PARAMS_PARAMS_LIST,
    SET_ROS_PARAM_SUCCESS,
    SET_ROS_PARAM_SUCCESS_TIMESTAMP,
    SET_ROS_PARAMS_SUCCESS_ALL,
    SET_ROS_PARAMS_SUCCESS_ALL_TIMESTAMP,
    SET_ROS_PARAMS_PARAMS_LIST,
    DUMP_ROS_PARAMS_SUCCESS,
    LOAD_ROS_PARAMS_SUCCESS,
    PUBLISHING_PARAMS_LIST,
    SYSTEM_MONITOR_DISK_SPACES,
    SYSTEM_MONITOR_WIFI_CONNECTED,
    SYSTEM_MONITOR_TEMPERATURE,
    SYSTEM_MONITOR_TIMESTAMP,
    ALIVE_MONITOR_STATUSES_TOPIC,
    ALIVE_MONITOR_STATUSES_SERVICE,
    FILE_MONITOR_CHECK_TIME,
    FILE_MONITOR_TIMESYNC_LOG,
    DOCK_BATTERY_CHARGE_INFO_REMAIN,
    NOT_ROS_FLIGHT_SOFTWARE_STATUS,
    NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS,
    PLATFORM_MANAGER_TIMESTAMP,
    PLATFORM_MANAGER_OPERATION_TYPE,
    PLATFORM_MANAGER_MODE,
    PLATFORM_MANAGER_START_CONTAINER,
    PLATFORM_MANAGER_LAST_USER,
    PLATFORM_MANAGER_LAST_LAUNCH,
    PLATFORM_MANAGER_LAST_IMAGE,
    PLATFORM_MANAGER_LAST_LOGIC,
    SET_OPERATION_TYPE_RESULT,
    PLATFORM_MONITOR_CHECK_TIME,
    PLATFORM_MONITOR_PUBLICATIONS,
    PLATFORM_MONITOR_SUBSCRIPTIONS,
    PLATFORM_MONITOR_SERVICES,
    PLATFORM_MONITOR_CONTAINERS,
    CAMERA_MAIN_STREAMING_STATUS,
    CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_RESULT,
    CAMERA_LEFT_STREAMING_STATUS,
    CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT,
    CAMERA_RIGHT_STREAMING_STATUS,
    CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT,
    MICROPHONE_STREAMING_STATUS,
    MICROPHONE_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
    MICROPHONE_UPDATE_PARAMETER_RESPONSE_RESULT,
    USER_NODE_STATUS_TIMESTAMP,
    USER_NODE_STATUS_MESSAGE,
    USER_NODE_RESULT,
    USER_LOGIC_RESULT
};

static const QMap<unsigned char, QString> UPDATE_PARAMETER_RESULT_LABEL =
{
    {ib2_msgs::UpdateParameterResponse::FAILURE_CALL,   "FAILURE_CALL"},
    {ib2_msgs::UpdateParameterResponse::FAILURE_UPDATE, "FAILURE_UPDATE"},
    {ib2_msgs::UpdateParameterResponse::SUCCESS,        "SUCCESS"},
};

static const QMap<unsigned char, QString> NAVIGATION_STARTUP_RESULT_LABEL =
{
    {ib2_msgs::NavigationStartUpResult::OFF,        "OFF"},
    {ib2_msgs::NavigationStartUpResult::ABORTED,    "ABORTED"},
    {ib2_msgs::NavigationStartUpResult::ON_READY,   "ON_READY"},
    {ib2_msgs::NavigationStartUpResult::TIME_OUT,   "TIME_OUT"},
};


static const QMap<short, QString> MARKER_CORRECTION_STATUS_LABEL =
{
    {ib2_msgs::MarkerCorrectionResponse::FAILURE_CALL,    "FAILURE_CALL"},
    {ib2_msgs::MarkerCorrectionResponse::FAILURE_UPDATE,  "FAILURE_UPDATE"},
    {ib2_msgs::MarkerCorrectionResponse::SUCCESS,         "SUCCESS"},
};

static const QMap<unsigned char, QString> ALIVE_STATUS_LABEL =
{
    {ib2_msgs::AliveStatus::SUCCESS,                    "SUCCESS"},
    {ib2_msgs::AliveStatus::FAIL,                       "FAIL"},
    {ib2_msgs::AliveStatus::INVALID_CONFIG_DATA_CLASS,  "INVALID_CONFIG_DATA_CLASS"},
};

static const QMap<unsigned char, QString> NAVIGATION_STATUS_LABEL =
{
    {ib2_msgs::NavigationStatus::NAV_OFF,       "NAV_OFF"},
    {ib2_msgs::NavigationStatus::NAV_SLAM,      "NAV_SLAM"},
    {ib2_msgs::NavigationStatus::NAV_FUSION,    "NAV_FUSION"},
    {ib2_msgs::NavigationStatus::NAV_INERTIAL,  "NAV_INERTIAL"},
};

static const QMap<unsigned char, QString> CTL_COMMAND_RESULT_LABEL =
{
    {ib2_msgs::CtlCommandResult::TERMINATE_SUCCESS,       "TERMINATE_SUCCESS"},
    {ib2_msgs::CtlCommandResult::TERMINATE_ABORTED,       "TERMINATE_ABORTED"},
    {ib2_msgs::CtlCommandResult::TERMINATE_TIME_OUT,      "TERMINATE_TIME_OUT"},
    {ib2_msgs::CtlCommandResult::TERMINATE_INVALID_CMD,   "TERMINATE_INVALID_CMD"},
    {ib2_msgs::CtlCommandResult::TERMINATE_INVALID_NAV,   "TERMINATE_INVALID_NAV"},
};

static const QMap<unsigned char, QString> USER_NODE_RESULT_LABEL =
{
    {platform_msgs::UserNodeCommandResponse::SUCCESS,              "SUCCESS"},
    {platform_msgs::UserNodeCommandResponse::INVALID_COMMAND,      "INVALID_COMMAND"},
    {platform_msgs::UserNodeCommandResponse::ERROR,                "ERROR"},
    {platform_msgs::UserNodeCommandResponse::NODE_ALREADY_RUNNING, "NODE_ALREADY_RUNNING"},
    {platform_msgs::UserNodeCommandResponse::IMAGE_NOT_FOUND,      "IMAGE_NOT_FOUND"},
    {platform_msgs::UserNodeCommandResponse::USER_NOT_FOUND,       "USER_NOT_FOUND"},
    {platform_msgs::UserNodeCommandResponse::LAUNCH_NOT_FOUND,     "LAUNCH_NOT_FOUND"},
    {platform_msgs::UserNodeCommandResponse::INVALID_LAUNCH,       "INVALID_LAUNCH"},
    {platform_msgs::UserNodeCommandResponse::NODE_NOT_STARTED,     "NODE_NOT_STARTED"},
};

static const QMap<unsigned char, QString> CONTAINER_STATUS_LABEL =
{
    {platform_msgs::ContainerStatus::EXITED,     "EXITED"},
    {platform_msgs::ContainerStatus::PAUSED,     "PAUSED"},
    {platform_msgs::ContainerStatus::RESTARTING, "RESTARTING"},
    {platform_msgs::ContainerStatus::RUNNING,    "RUNNING"},
};

/**
 * @brief Int-Ballテレメトリ用データ項目の構造体.
 */
struct ConfigItem
{
    std::string rosMessageName;
    QString name;
    QVariant defaultValue;
};
const std::string LAST_EXECUTED_COMMAND = "last_executed_command";
const std::string SPLIT_NUMBER = "split_number";
const std::string CURRENT_SPILIT_INDEX = "current_split_index";
static const QMap<Index, ConfigItem> Config =
{
    {
        Index::TIMESTAMP,
        {rosname::TIMESTAMP, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::LAST_EXECUTED_COMMAND,
        {rosname::LAST_EXECUTED_COMMAND, "Last executed command", QVariant::fromValue(static_cast<unsigned short>(0))}
    },
    {
        Index::SPLIT_NUMBER,
        {rosname::SPLIT_NUMBER, "Split number", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CURRENT_SPILIT_INDEX,
        {rosname::CURRENT_SPILIT_INDEX, "Current split index", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::SENDING_PORT_INDEX,
        {rosname::SENDING_PORT_INDEX, "Sending port index", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::MODE,
        {rosname::TASK_MANAGER_MODE, "Mode", QVariant::fromValue(static_cast<unsigned char>(INTBALL_MODE_UNKNOWN))}
    },
    {
        Index::EXIT_DOCKING_MODE_SUCCESS,
        {rosname::TASK_MANAGER_EXIT_DOCKING_MODE, "Success", QVariant::fromValue(false)}
    },
    {
        Index::EXIT_DOCKING_MODE_RESULT_MODE,
        {rosname::TASK_MANAGER_EXIT_DOCKING_MODE, "Result mode", QVariant::fromValue(static_cast<unsigned char>(255))}
    },
    {
        Index::SET_MAINTENANCE_MODE_RESULT_MODE,
        {rosname::TASK_MANAGER_SET_MAINTENANCE_MODE, "Result mode", QVariant::fromValue(static_cast<unsigned char>(255))}
    },
    {
        Index::NAVIGATION_STARTUP_FEEDBACK_DURATION,
        {rosname::NAVIGATION_STARTUP_FEEDBACK, "Duration", QVariant::fromValue(ros::Duration())}
    },
    {
        Index::NAVIGATION_STARTUP_RESULT_TIMESTAMP,
        {rosname::NAVIGATION_STARTUP_RESULT, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::NAVIGATION_STARTUP_RESULT_TYPE,
        {rosname::NAVIGATION_STARTUP_RESULT, "Type", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::NAVIGATION_HEADER_SEQ,
        {rosname::NAVIGATION, "Sequence number", QVariant::fromValue(static_cast<unsigned int>(0))}
    },
    {
        Index::NAVIGATION_HEADER_STAMP,
        {rosname::NAVIGATION, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::NAVIGATION_HEADER_FRAME_ID,
        {rosname::NAVIGATION, "Frame ID", QVariant::fromValue(std::string())}
    },
    {
        Index::NAVIGATION_POSE_POSITION,
        {rosname::NAVIGATION, "Position", QVariant::fromValue(geometry_msgs::Point())}
    },
    {
        Index::NAVIGATION_POSE_ORIENTATION,
        {rosname::NAVIGATION, "Orientation", QVariant::fromValue(geometry_msgs::Quaternion())}
    },
    {
        Index::NAVIGATION_TWIST_LINEAR,
        {rosname::NAVIGATION, "Linear", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::NAVIGATION_TWIST_ANGULAR,
        {rosname::NAVIGATION, "Angular", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::NAVIGATION_A,
        {rosname::NAVIGATION, "Acceleration", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::NAVIGATION_STATUS,
        {rosname::NAVIGATION, "Status", QVariant::fromValue(ib2_msgs::NavigationStatus())}
    },
    {
        Index::NAVIGATION_DEBUG_POINT,
        {rosname::NAVIGATION_DEBUG, "Number of points", QVariant::fromValue(static_cast<unsigned int>(0))}
    },
    {
        Index::NAVIGATION_DEBUG_POS_OFFSETS,
        {rosname::NAVIGATION_DEBUG, "Position offsets", QVariant::fromValue(QList<float>())}
    },
    {
        Index::NAVIGATION_DEBUG_ATT_OFFSETS,
        {rosname::NAVIGATION_DEBUG, "Attitude offsets", QVariant::fromValue(QList<float>())}
    },
    {
        Index::NAVIGATION_DEBUG_IMU_TEMPERATURE,
        {rosname::NAVIGATION_DEBUG, "IMU temperature", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::NAVIGATION_STATUS_TOPIC_STASUS,
        {rosname::NAVIGATION_STATUS_TOPIC, "Status", QVariant::fromValue(ib2_msgs::NavigationStatus())}
    },
    {
        Index::MARKER_CORRECTION_TIMESTAMP,
        {rosname::MARKER_CORRECTION, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::MARKER_CORRECTION_STATUS,
        {rosname::MARKER_CORRECTION, "Status", QVariant::fromValue(static_cast<short>(0))}
    },
    {
        Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::NAVIGATION_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::NAVIGATION_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::NAVIGATION_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::IMU_TIMESTAMP,
        {rosname::IMU_IMU, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::IMU_ACC_X,
        {rosname::IMU_IMU, "Acc x", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_ACC_Y,
        {rosname::IMU_IMU, "Acc y", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_ACC_Z,
        {rosname::IMU_IMU, "Acc z", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_GYRO_X,
        {rosname::IMU_IMU, "Gyro x", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_GYRO_Y,
        {rosname::IMU_IMU, "Gyro y", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_GYRO_Z,
        {rosname::IMU_IMU, "Gyro z", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_TEMPERATURE,
        {rosname::IMU_IMU, "Imu temperature", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::IMU_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::IMU_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::IMU_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::IMU_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::SLAM_WRAPPER_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::SLAM_WRAPPER_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::SLAM_WRAPPER_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CTL_STATUS_HEADER_SEQ,
        {rosname::CTL_STATUS, "Sequence number", QVariant::fromValue(static_cast<unsigned int>(0))}
    },
    {
        Index::CTL_STATUS_HEADER_STAMP,
        {rosname::CTL_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CTL_STATUS_HEADER_FRAME_ID,
        {rosname::CTL_STATUS, "Frame ID", QVariant::fromValue(std::string())}
    },
    {
        Index::CTL_STATUS_POSE_POSITION,
        {rosname::CTL_STATUS, "Position", QVariant::fromValue(geometry_msgs::Point())}
    },
    {
        Index::CTL_STATUS_POSE_ORIENTATION,
        {rosname::CTL_STATUS, "Orientation", QVariant::fromValue(geometry_msgs::Quaternion())}
    },
    {
        Index::CTL_STATUS_TWIST_LINEAR,
        {rosname::CTL_STATUS, "Linear", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::CTL_STATUS_TWIST_ANGULAR,
        {rosname::CTL_STATUS, "Angular", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::CTL_STATUS_A,
        {rosname::CTL_STATUS, "Acceleration", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::CTL_STATUS_TYPE,
        {rosname::CTL_STATUS, "Status", QVariant::fromValue(static_cast<int>(GUIDANCE_CONTROL_TYPE_UNKNOWN))}
    },
    {
        Index::CTL_WRENCH_HEADER_SEQ,
        {rosname::CTL_WRENCH, "Sequence number", QVariant::fromValue(static_cast<unsigned int>(0))}
    },
    {
        Index::CTL_WRENCH_HEADER_STAMP,
        {rosname::CTL_WRENCH, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CTL_WRENCH_HEADER_FRAME_ID,
        {rosname::CTL_WRENCH, "Frame ID", QVariant::fromValue(std::string())}
    },
    {
        Index::CTL_WRENCH_FORCE,
        {rosname::CTL_WRENCH, "Force", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::CTL_WRENCH_TORQUE,
        {rosname::CTL_WRENCH, "Torque", QVariant::fromValue(geometry_msgs::Vector3())}
    },
    {
        Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_ID,
        {rosname::CTL_ACTION_FEEDBACK, "Goal id", QVariant::fromValue(std::string())}
    },
    {
        Index::CTL_ACTION_FEEDBACK_STATUS_GOAL_STAMP,
        {rosname::CTL_ACTION_FEEDBACK, "Goal timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CTL_ACTION_FEEDBACK_TIME_TO_GO,
        {rosname::CTL_ACTION_FEEDBACK, "Time to go", QVariant::fromValue(ros::Duration())}
    },
    {
        Index::CTL_ACTION_FEEDBACK_POSE_POSITION,
        {rosname::CTL_ACTION_FEEDBACK, "Position", QVariant::fromValue(geometry_msgs::Point())}
    },
    {
        Index::CTL_ACTION_FEEDBACK_POSE_ORIENTATION,
        {rosname::CTL_ACTION_FEEDBACK, "Orientation", QVariant::fromValue(geometry_msgs::Quaternion())}
    },
    {
        Index::CTL_ACTION_RESULT_TIMESTAMP,
        {rosname::CTL_ACTION_RESULT, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CTL_ACTION_RESULT_TYPE,
        {rosname::CTL_ACTION_RESULT, "Type", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CTL_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::CTL_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CTL_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::CTL_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::PROP_STATUS_HEADER_SEQ,
        {rosname::PROP_STATUS, "Sequence number", QVariant::fromValue(static_cast<unsigned int>(0))}
    },
    {
        Index::PROP_STATUS_HEADER_STAMP,
        {rosname::PROP_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::PROP_STATUS_HEADER_FRAME_ID,
        {rosname::PROP_STATUS, "Frame ID", QVariant::fromValue(std::string())}
    },
    {
        Index::PROP_STATUS_DUTY,
        {rosname::PROP_STATUS, "Duty", QVariant::fromValue(std::vector<double>())}
    },
    {
        Index::PROP_STATUS_POWER,
        {rosname::PROP_STATUS, "Power status", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::PROP_SWITCH_POWER_RESPONSE,
        {rosname::PROP_SWITCH_POWER, "Switch power response", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::PROP_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::PROP_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::PROP_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::PROP_STATUS, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CAMERA_MIC_STREAMING_STATUS,
        {rosname::CAMERA_MIC_STATUS, "Streaming status", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_MIC_RECORDING_STATUS,
        {rosname::CAMERA_MIC_STATUS, "Recording status", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_MIC_CAMERA_POWER,
        {rosname::CAMERA_MIC_STATUS, "Camera power", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_MIC_MICROPHONE_POWER,
        {rosname::CAMERA_MIC_STATUS, "Microphone power", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_MIC_ZOOM,
        {rosname::CAMERA_MIC_STATUS, "Zoom", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_RESOLUTION_TYPE,
        {rosname::CAMERA_MIC_STATUS, "Resolution type", QVariant::fromValue(ib2_msgs::MainCameraResolutionType())}
    },
    {
        Index::CAMERA_MIC_EV,
        {rosname::CAMERA_MIC_STATUS, "EV", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_CAMERA_GAIN,
        {rosname::CAMERA_MIC_STATUS, "Camera gain", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_WHITE_BALANCE_MODE,
        {rosname::CAMERA_MIC_STATUS, "White balance mode", QVariant::fromValue(ib2_msgs::MainCameraWhiteBalanceMode())}
    },
    {
        Index::CAMERA_MIC_FRAME_RATE,
        {rosname::CAMERA_MIC_STATUS, "Frame rate", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_SENDING_BIT_RATE,
        {rosname::CAMERA_MIC_STATUS, "Bit rate", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_MICROPHONE_GAIN,
        {rosname::CAMERA_MIC_STATUS, "Microphone gain", QVariant::fromValue(static_cast<float>(0.0))}
    },
    {
        Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::CAMERA_MIC_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CAMERA_MIC_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::CAMERA_MIC_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::LED_LEFT_LED_COLORS,
        {rosname::LED_LEFT_LED_COLORS, "Colors", QVariant::fromValue(QVector<std_msgs::ColorRGBA>(8))}
    },
    {
        Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::LED_LEFT_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::LED_RIGHT_LED_COLORS,
        {rosname::LED_RIGHT_LED_COLORS, "Colors", QVariant::fromValue(QVector<std_msgs::ColorRGBA>(8))}
    },
    {
        Index::LED_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::LED_LEFT_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::LED_RIGHT_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::LED_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::LED_RIGHT_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::DISPLAY_MANAGER_STATUS_MODE,
        {rosname::DISPLAY_MANAGER_STATUS, "Mode", QVariant::fromValue(static_cast<unsigned char>(255))}
    },
    {
        Index::DISPLAY_MANAGER_STATUS_CTL_STATUS_TYPE,
        {rosname::DISPLAY_MANAGER_STATUS, "Status type", QVariant::fromValue(-1)}
    },
    {
        Index::DISPLAY_MANAGER_STATUS_COLOR,
        {rosname::DISPLAY_MANAGER_STATUS, "Color", QVariant::fromValue(std_msgs::ColorRGBA())}
    },
    {
        Index::DISPLAY_MANAGER_STATUS_POWER,
        {rosname::DISPLAY_MANAGER_STATUS, "Power", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::DISPLAY_MANAGER_STATUS_FLASH,
        {rosname::DISPLAY_MANAGER_STATUS, "Lighting", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::DISPLAY_MANAGER_SWITCH_POWER_RESPONSE,
        {rosname::DISPLAY_MANAGER_SWITCH_POWER, "Switch power response", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::DISPLAY_MANAGER_SWITCH_FLASH_RESPONSE,
        {rosname::DISPLAY_MANAGER_SWITCH_FLASH, "Switch flash response", QVariant::fromValue(ib2_msgs::PowerStatus())}
    },
    {
        Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::DISPLAY_MANAGER_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::DISPLAY_MANAGER_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::DISPLAY_MANAGER_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::GET_ROS_PARAMS_PARAMS_LIST,
        {rosname::PARAMETER_MANAGER_GET_ROS_PARAMS, "Parameters", QVariant::fromValue(QMap<QString, RosParam>())}
    },
    {
        Index::GET_ROS_PARAM_SUCCESS,
        {rosname::PARAMETER_MANAGER_GET_ROS_PARAM, "Success", QVariant::fromValue(false)}
    },
    {
        Index::GET_ROS_PARAM_VALUES,
        {rosname::PARAMETER_MANAGER_GET_ROS_PARAM, "Values", QVariant::fromValue(RosParam())}
    },
    {
        Index::SET_ROS_PARAM_SUCCESS,
        {rosname::PARAMETER_MANAGER_SET_ROS_PARAM, "Success", QVariant::fromValue(false)}
    },
    {
        Index::SET_ROS_PARAM_SUCCESS_TIMESTAMP,
        {rosname::PARAMETER_MANAGER_SET_ROS_PARAM, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::SET_ROS_PARAMS_SUCCESS_ALL,
        {rosname::PARAMETER_MANAGER_SET_ROS_PARAMS, "Success", QVariant::fromValue(false)}
    },
    {
        Index::SET_ROS_PARAMS_SUCCESS_ALL_TIMESTAMP,
        {rosname::PARAMETER_MANAGER_SET_ROS_PARAMS, "Success", QVariant::fromValue(ros::Time())}
    },
    {
        Index::SET_ROS_PARAMS_PARAMS_LIST,
        {rosname::PARAMETER_MANAGER_SET_ROS_PARAMS, "Parameters", QVariant::fromValue(QMap<QString, RosParam>())}
    },
    {
        Index::DUMP_ROS_PARAMS_SUCCESS,
        {rosname::PARAMETER_MANAGER_DUMP_ROS_PARAMS, "Success", QVariant::fromValue(false)}
    },
    {
        Index::LOAD_ROS_PARAMS_SUCCESS,
        {rosname::PARAMETER_MANAGER_LOAD_ROS_PARAMS, "Success", QVariant::fromValue(false)}
    },
    {
        Index::PUBLISHING_PARAMS_LIST,
        {rosname::PARAMETER_MANAGER_PUBLISHING_PARAMS, "Parameters", QVariant::fromValue(QMap<QString, QString>())}
    },
    {
        Index::SYSTEM_MONITOR_DISK_SPACES,
        {rosname::SYSTEM_MONITOR_STATUS, "Disk spaces", QVariant::fromValue(QMap<QString, float>())}
    },
    {
        Index::SYSTEM_MONITOR_WIFI_CONNECTED,
        {rosname::SYSTEM_MONITOR_STATUS, "WiFi connected", QVariant::fromValue(false)}
    },
    {
        Index::SYSTEM_MONITOR_TEMPERATURE,
        {rosname::SYSTEM_MONITOR_STATUS, "Temperature", QVariant::fromValue(static_cast<float>(0))}
    },
    {
        Index::SYSTEM_MONITOR_TIMESTAMP,
        {rosname::SYSTEM_MONITOR_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::ALIVE_MONITOR_STATUSES_TOPIC,
        {rosname::SYSTEM_MONITOR_STATUS, "Topic statuses", QVariant::fromValue(QMap<QString, ib2_msgs::AliveStatus>())}
    },
    {
        Index::ALIVE_MONITOR_STATUSES_SERVICE,
        {rosname::SYSTEM_MONITOR_STATUS, "Service statuses", QVariant::fromValue(QMap<QString, ib2_msgs::AliveStatus>())}
    },
    {
        Index::FILE_MONITOR_CHECK_TIME,
        {rosname::FILE_MONITOR_STATUS, "Check time", QVariant::fromValue(ros::Time())}
    },
    {
        Index::FILE_MONITOR_TIMESYNC_LOG,
        {rosname::FILE_MONITOR_STATUS, "Time synchronization timestamp", QVariant::fromValue(std::string())}
    },
    {
        Index::DOCK_BATTERY_CHARGE_INFO_REMAIN,
        {rosname::DOCK_BATTERY_CHARGE_INFO, "Battery Remain", QVariant::fromValue(static_cast<char>(0))}
    },
    {
        Index::NOT_ROS_FLIGHT_SOFTWARE_STATUS,
        {rosname::NOT_ROS_FLIGHT_SOFTWARE_STATUS, "Flight software status", QVariant::fromValue(false)}
    },
    {
        Index::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS,
        {rosname::NOT_ROS_PLATFORM_FLIGHT_SOFTWARE_STATUS, "Platform Flight software status", QVariant::fromValue(false)}
    },
    {
        Index::PLATFORM_MANAGER_TIMESTAMP,
        {rosname::PLATFORM_MANAGER_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::PLATFORM_MANAGER_OPERATION_TYPE,
        {rosname::PLATFORM_MANAGER_STATUS, "Operation type", QVariant::fromValue(static_cast<unsigned char>(PLATFORM_OPERATION_TYPE_UNKNOWN))}
    },
    {
        Index::PLATFORM_MANAGER_MODE,
        {rosname::PLATFORM_MANAGER_STATUS, "Mode", QVariant::fromValue(static_cast<unsigned char>(PLATFORM_MODE_UNKNOWN))}
    },
    {
        Index::PLATFORM_MANAGER_START_CONTAINER,
        {rosname::PLATFORM_MANAGER_STATUS, "Start container", QVariant::fromValue(false)}
    },
    {
        Index::PLATFORM_MANAGER_LAST_USER,
        {rosname::PLATFORM_MANAGER_STATUS, "Last user", QVariant::fromValue(std::string())}
    },
    {
        Index::PLATFORM_MANAGER_LAST_LAUNCH,
        {rosname::PLATFORM_MANAGER_STATUS, "Last launch", QVariant::fromValue(std::string())}
    },
    {
        Index::PLATFORM_MANAGER_LAST_IMAGE,
        {rosname::PLATFORM_MANAGER_STATUS, "Last image", QVariant::fromValue(std::string())}
    },
    {
        Index::PLATFORM_MANAGER_LAST_LOGIC,
        {rosname::PLATFORM_MANAGER_STATUS, "Last image", QVariant::fromValue(static_cast<unsigned short>(0))}
    },
    {
        Index::SET_OPERATION_TYPE_RESULT,
        {rosname::SET_OPERATION_TYPE, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::PLATFORM_MONITOR_CHECK_TIME,
        {rosname::PLATFORM_MONITOR_STATUS, "Check time", QVariant::fromValue(ros::Time())}
    },
    {
        Index::PLATFORM_MONITOR_PUBLICATIONS,
        {rosname::PLATFORM_MONITOR_STATUS, "Publications", QVariant::fromValue(QVector<NodeStatus>(100))}
    },
    {
        Index::PLATFORM_MONITOR_SUBSCRIPTIONS,
        {rosname::PLATFORM_MONITOR_STATUS, "Subscriptions", QVariant::fromValue(QVector<NodeStatus>(100))}
    },
    {
        Index::PLATFORM_MONITOR_SERVICES,
        {rosname::PLATFORM_MONITOR_STATUS, "Services", QVariant::fromValue(QVector<NodeStatus>(100))}
    },
    {
        Index::PLATFORM_MONITOR_CONTAINERS,
        {rosname::PLATFORM_MONITOR_STATUS, "Containers", QVariant::fromValue(QVector<ContainerStatus>(100))}
    },
    {
        Index::CAMERA_MAIN_STREAMING_STATUS,
        {rosname::CAMERA_MAIN_STATUS, "Streaming status", QVariant::fromValue(platform_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::CAMERA_MAIN_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CAMERA_MAIN_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::CAMERA_MAIN_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CAMERA_LEFT_STREAMING_STATUS,
        {rosname::CAMERA_LEFT_STATUS, "Streaming status", QVariant::fromValue(platform_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::CAMERA_LEFT_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CAMERA_LEFT_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::CAMERA_LEFT_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::CAMERA_RIGHT_STREAMING_STATUS,
        {rosname::CAMERA_RIGHT_STATUS, "Streaming status", QVariant::fromValue(platform_msgs::PowerStatus())}
    },
    {
        Index::CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::CAMERA_RIGHT_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::CAMERA_RIGHT_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::CAMERA_RIGHT_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::MICROPHONE_STREAMING_STATUS,
        {rosname::MICROPHONE_STATUS, "Streaming status", QVariant::fromValue(platform_msgs::PowerStatus())}
    },
    {
        Index::MICROPHONE_UPDATE_PARAMETER_RESPONSE_TIMESTAMP,
        {rosname::MICROPHONE_UPDATE_PARAMETER, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::MICROPHONE_UPDATE_PARAMETER_RESPONSE_RESULT,
        {rosname::MICROPHONE_UPDATE_PARAMETER, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::USER_NODE_STATUS_TIMESTAMP,
        {rosname::USER_NODE_STATUS, "Timestamp", QVariant::fromValue(ros::Time())}
    },
    {
        Index::USER_NODE_STATUS_MESSAGE,
        {rosname::USER_NODE_STATUS, "Message", QVariant::fromValue(QVector<char>(800))}
    },
    {
        Index::USER_NODE_RESULT,
        {rosname::PLATFORM_MANAGER_USER_NODE, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    },
    {
        Index::USER_LOGIC_RESULT,
        {rosname::PLATFORM_MANAGER_USER_LOGIC, "Result", QVariant::fromValue(static_cast<unsigned char>(0))}
    }
};

} // namespace telemery

namespace dock
{
namespace telecommand
{

enum class Index {
    SET_HOST_IP_ADDR,
    SET_COMMAND_PORT,
    CHARGE_ON_OFF,
    MOTOR_ON_OFF,
    SET_IB_ADDR,
};

enum class CHARGE_ON_OFF_TYPE {
    ON = 1,
    OFF = 2,
};

enum class MOTOR_ON_OFF_TYPE {
    STOP = 1,
    RELEASE = 2,
    DOCK = 3,
};

static const QMap<Index, QList<unsigned char>> CODE =
{
    {Index::SET_HOST_IP_ADDR, {0x02, 0x12}},
    {Index::SET_COMMAND_PORT, {0x03, 0x12}},
    {Index::SET_IB_ADDR, {0x04, 0x12}},
    {Index::CHARGE_ON_OFF, {0x50, 0x12}},
    {Index::MOTOR_ON_OFF, {0x51, 0x12}},
}; // telecommand


} // namespace telecommand

namespace telemetry
{

/**
 * @brief ドッキングステーションテレメトリ用データ項目のインデックス.
 */
enum class Index
{
    TELEMETRY_COUNTER,
    COMMAND_COUNTER,
    CHARGE_STATE,
    MOTOR_STATE,
    SWITCH_STATE,
    COMMAND_STATE,
    VERSION,
    MOTOR_TEMP,
    DCDC_TEMP,
    POWER1_CURRENT,
    POWER2_CURRENT,
    TEMP_ALERT,
    CHARGE_TIME,
};

/**
 * @brief CHARGE_STATEの値定義.
 */
enum CHARGE_STATE_TYPE
{
    ON = 1,
    OFF = 2,
};

/**
 * @brief MOTOR_STATEの値定義.
 */
enum MOTOR_STATE_TYPE
{
    STOP = 1,
    RELEASE = 2,
    DOCKING = 3,
};

/**
 * @brief SWITCH_STATEの値定義.
 */
enum SWITCH_STATE_TYPE
{
    ALL_OFF,
    DONE_RELEASE,
    DONE_DOCKING,
    ALL_ON,
};

/**
 * @brief COMMAND_STATEの値定義.
 */
enum COMMAND_STATE_TYPE
{
    SUCCESS = 1,
    FAILED = 2,
    UNKNOWN = 3,
};

/**
 * @brief TEMP_ALERTの値定義.
 */
enum TEMP_ALERT_TYPE
{
    NORMAL,
    ALERT_MOTOR_TEMP,
    ALERT_DCDC_TEMP,
    ALERT_ALL,
};

/**
 * @brief ドッキングステーションテレメトリ用データ項目の構造体.
 */
struct ConfigItem
{
    QString name;
    QString description;
    QVariant defaultValue;
};

static const QMap<Index, ConfigItem> Config =
{
    {Index::TELEMETRY_COUNTER, {"TelemetryCounter", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::COMMAND_COUNTER, {"CommandCounter", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::CHARGE_STATE, {"ChargeState", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::MOTOR_STATE, {"MotorState", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::SWITCH_STATE, {"SwitchState", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::COMMAND_STATE, {"CommandState", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::VERSION, {"Version", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::MOTOR_TEMP, {"MotorTemperature", "", QVariant::fromValue(static_cast<double>(0.0))}},
    {Index::DCDC_TEMP, {"DcdcTemperature", "", QVariant::fromValue(static_cast<double>(0.0))}},
    {Index::POWER1_CURRENT, {"Power1Current", "", QVariant::fromValue(static_cast<double>(0.0))}},
    {Index::POWER2_CURRENT, {"Power2Current", "", QVariant::fromValue(static_cast<double>(0.0))}},
    {Index::TEMP_ALERT, {"TemperatureAlert", "", QVariant::fromValue(static_cast<unsigned char>(0))}},
    {Index::CHARGE_TIME, {"ChargeTime", "", QVariant::fromValue(static_cast<unsigned short>(0))}},
};

static const QMap<Index, unsigned long> DataOffset =
{
    {Index::TELEMETRY_COUNTER, 0},
    {Index::COMMAND_COUNTER,   1},
    {Index::CHARGE_STATE,      2},
    {Index::MOTOR_STATE,       3},
    {Index::SWITCH_STATE,      4},
    {Index::COMMAND_STATE,     5},
    {Index::VERSION,           6},
    {Index::MOTOR_TEMP,        7},
    {Index::DCDC_TEMP,         9},
    {Index::POWER1_CURRENT,    11},
    {Index::POWER2_CURRENT,    13},
    {Index::TEMP_ALERT,        15},
    {Index::CHARGE_TIME,       16},
};

} // namespace telemetry

} // namespace dock

} // namespace intball

// 一部クラスについてはTelemetrySubscriberのコンストラクタで定義.。
Q_DECLARE_METATYPE(intball::telecommand::UPDATE_PARAMETER_TARGET);
Q_DECLARE_METATYPE(intball::dock::telecommand::CHARGE_ON_OFF_TYPE);
Q_DECLARE_METATYPE(intball::dock::telecommand::MOTOR_ON_OFF_TYPE);
Q_DECLARE_METATYPE(std::string);
Q_DECLARE_METATYPE(geometry_msgs::Point);
Q_DECLARE_METATYPE(geometry_msgs::Quaternion);
Q_DECLARE_METATYPE(geometry_msgs::Vector3);
Q_DECLARE_METATYPE(ib2_msgs::NavigationStatus);
Q_DECLARE_METATYPE(ib2_msgs::PowerStatus);
Q_DECLARE_METATYPE(ib2_msgs::MainCameraResolutionType);
Q_DECLARE_METATYPE(ib2_msgs::MainCameraWhiteBalanceMode);
Q_DECLARE_METATYPE(platform_msgs::OperationType);
Q_DECLARE_METATYPE(platform_msgs::Mode);
Q_DECLARE_METATYPE(platform_msgs::PowerStatus);
Q_DECLARE_METATYPE(ros::Duration);
Q_DECLARE_METATYPE(std_msgs::ColorRGBA);
Q_DECLARE_METATYPE(std_msgs::Float64MultiArray);
Q_DECLARE_METATYPE(ib2_msgs::AliveStatus);
Q_DECLARE_METATYPE(intball::RosParam);
Q_DECLARE_METATYPE(intball::NodeStatus);
Q_DECLARE_METATYPE(intball::ContainerStatus);
Q_DECLARE_METATYPE(platform_msgs::NodeStatusValue);
Q_DECLARE_METATYPE(platform_msgs::ContainerStatus);
Q_DECLARE_METATYPE(platform_msgs::UserLogic);
#endif // TELEMETRY_TELECOMMAND_CONFIG_H
