#ifndef GUI_COMMON_H
#define GUI_COMMON_H

#include <QString>

namespace intball
{

namespace message
{
const static QString COMMAND_NAME_GO = "Start moving the Int-Ball2";
const static QString COMMAND_NAME_RECORD = "Control of video recording";
const static QString COMMAND_NAME_STREAMING = "Control of video streaming";
const static QString COMMAND_NAME_LIGHTING = "Lighting control";
const static QString COMMAND_NAME_CAMERA_POWER = "Switch the camera";
const static QString COMMAND_NAME_MICROPHONE_POWER = "Switch the microphone";
const static QString COMMAND_NAME_MOVE_INTBALL2 = "Move Int-Ball2";
const static QString COMMAND_NAME_EMERGENCY_STOP = "Emergency stop";
const static QString COMMAND_NAME_DOCKING = "Docking";
const static QString COMMAND_NAME_RELEASE = "Release";
const static QString COMMAND_NAME_PARAMETER_SEND = "Update parameter(s) (no reload)";
const static QString COMMAND_NAME_PARAMETER_RELOAD = "Execute parameter reloading";
const static QString COMMAND_NAME_PARAMETER_SEND_RELOAD = "Update parameters and reload";
const static QString COMMAND_NAME_PARAMETER_GET = "Get parameter(s)";
const static QString COMMAND_NAME_DUTY = "Set the duty values of the propellers";
const static QString COMMAND_NAME_MARKER_CORRECTION = "Execute marker correction";
const static QString COMMAND_NAME_LED_GAINS_LEFT = "Set the gain value of the left LED and execute reload";
const static QString COMMAND_NAME_LED_GAINS_RIGHT = "Set the gain value of the right LED and execute reload";
const static QString COMMAND_NAME_LED_COLOR_LEFT = "Set the lighting color of the left LED";
const static QString COMMAND_NAME_LED_COLOR_RIGHT = "Set the lighting color of the right LED";
const static QString COMMAND_NAME_EXIT_DOCKING_MODE = "Forced termination of DOCKING mode";
const static QString COMMAND_NAME_EXIT_DOCKING_MODE_FINISH = "Completes the docking operation and transits to STANDBY mode";
const static QString COMMAND_NAME_EXIT_DOCKING_MODE_CANCEL = "Cancels the docking operation and transits to OPERATION mode";
const static QString COMMAND_NAME_SET_MAINTENANCE_MODE = "MAINTENANCE mode switching";
const static QString COMMAND_NAME_DUMP_ROSPARAM = "Output all rosparams to a YAML file";
const static QString COMMAND_NAME_LOAD_ROSPARAM = "Import file contents as rosparams";
const static QString COMMAND_NAME_CTL_COMMAND = "Executes guidance control commands";
const static QString COMMAND_NAME_CANCEL_CTL_COMMAND = "Cancels the currently executing guidance control command.";
const static QString COMMAND_NAME_FORCED_RELEASE = "Forced release (Change the mode of Int-Ball2 from STANDBY mode to OPERATION mode immediately).";
const static QString COMMAND_NAME_REBOOT = "Reboot Int-Ball2.";
const static QString FORMAT_COMMAND_NAME_SWITCH_POWER = "Enables / disables the function（%1）";
const static QString ARG_FUNCTION_PROPULSION = "Propulsion";
const static QString ARG_FUNCTION_NAVIGATION = "Navigation(sensor_fusion)";
const static QString ARG_FUNCTION_DISPLAY_MANAGEMENT = "Display management";
const static QString ARG_FUNCTION_LIGHTING = "Lighting";
const static QString COMMAND_NAME_DOCK_MOTOR = "Move the docking station's motor";
const static QString COMMAND_NAME_DOCK_CHARGE = "Switch the power supply function of the docking station";
const static QString COMMAND_NAME_DOCK_PORT = "Set docking stations's telecommand receive port";
const static QString COMMAND_NAME_DOCK_SEND_IP_GROUND = "Set the destination IP (Ground System) for docking station's telemetry";
const static QString COMMAND_NAME_DOCK_SEND_IP_INTBALL2 = "Set the destination IP (Int-Ball2) for docking station's telemetry";
const static QString DIALOG_MSG_COMMAND_SEND_ERROR = "The command could not be sent.";
const static QString DIALOG_MSG_COMMAND_SEND_SUCCESS = "The command has been sent.";
const static QString COMMAND_NAME_USER_PROGRAMING_NODE_START = "Execute user programing";
const static QString COMMAND_NAME_USER_PROGRAMING_NODE_STOP = "Stop user programing";
const static QString COMMAND_NAME_USER_LOGIC_START = "Execute user logic";
const static QString COMMAND_NAME_USER_LOGIC_STOP = "Stop user logic";
const static QString COMMAND_NAME_SET_OPERATION_TYPE = "Switch the operation type";
const static QString COMMAND_NAME_AUDIO_STREAMING = "Control of audio streaming";
}

} // namespace intball

#endif // GUI_COMMON_H
