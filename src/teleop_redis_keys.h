#include <iostream>

std::string CONTROLLER_RUNNING_KEY = "robot::controller_key";
std::string ROBOT_JOINT_POSITION_KEY = "robot::q";
std::string ROBOT_JOINT_VELOCITY_KEY = "robot::dq";
std::string ROBOT_JOINT_TORQUE_KEY = "robot::command_torque";
// Force sensor.
std::string FORCE_SENSOR_FORCE = "robot::force_sensor::force";
std::string FORCE_SENSOR_MOMENT = "robot::force_sensor::moment";

// default pose keys
std::string ROBOT_DEFAULT_POS_KEY = "robot::ee::pose::default::current_pose";
std::string ROBOT_DEFAULT_ROT_KEY = "robot::ee::pose::default::current_rotation";

std::string ROBOT_CURRENT_POS_KEY = ROBOT_DEFAULT_POS_KEY;
std::string ROBOT_CURRENT_ROT_KEY = ROBOT_DEFAULT_ROT_KEY;

// robot force key
std::string ROBOT_SENSED_FORCE_KEY = "robot::ee::sensed_force";
std::string ROBOT_SENSED_TORQUE_KEY = "robot::ee::sensed_torque";

// robot pos key.
std::string ROBOT_POS_KEY = "robot::ee::pose::target_pose";
std::string ROBOT_ROT_KEY = "robot::ee::pose::target_rotation";

std::string ROBOT_TARGET_POS_KEY = ROBOT_POS_KEY;
std::string ROBOT_TARGET_ROT_KEY = ROBOT_ROT_KEY;

// haptic and robot ready state keys.
std::string ROBOT_READY_STATE_KEY = "robot::state::ready";
std::string HAPTIC_READY_STATE_KEY = "device::state::ready"; 

// haptic driver keys

const std::string CHAI_REDIS_DRIVER_NAMESPACE = "chai_haptic_devices_driver";

const std::string DEVICE_MAX_STIFFNESS_KEY_SUFFIX = "specifications::max_stiffness";
const std::string DEVICE_MAX_DAMPING_KEY_SUFFIX = "specifications::max_damping";
const std::string DEVICE_MAX_FORCE_KEY_SUFFIX = "specifications::max_force";
const std::string DEVICE_MAX_GRIPPER_ANGLE = "specifications::max_gripper_angle";
const std::string DEVICE_COMMANDED_FORCE_KEY_SUFFIX = "actuators::commanded_force";
const std::string DEVICE_COMMANDED_TORQUE_KEY_SUFFIX = "actuators::commanded_torque";
const std::string DEVICE_COMMANDED_GRIPPER_FORCE_KEY_SUFFIX =
	"actuators::commanded_force_gripper";
const std::string DEVICE_POSITION_KEY_SUFFIX = "sensors::current_position";
const std::string DEVICE_ROTATION_KEY_SUFFIX = "sensors::current_rotation";
const std::string DEVICE_GRIPPER_POSITION_KEY_SUFFIX =
	"sensors::current_position_gripper";
const std::string DEVICE_LINEAR_VELOCITY_KEY_SUFFIX = "sensors::current_trans_velocity";
const std::string DEVICE_ANGULAR_VELOCITY_KEY_SUFFIX = "sensors::current_rot_velocity";
const std::string DEVICE_GRIPPER_VELOCITY_KEY_SUFFIX = "sensors::current_velocity_gripper";
const std::string DEVICE_SENSED_FORCE_KEY_SUFFIX = "sensors::sensed_force";
const std::string DEVICE_SENSED_TORQUE_KEY_SUFFIX = "sensors::sensed_torque";
const std::string DEVICE_USE_GRIPPER_AS_SWITCH_KEY_SUFFIX = "parametrization::use_gripper_as_switch";
const std::string DEVICE_SWITCH_PRESSED_KEY_SUFFIX = "sensors::switch_pressed";
const std::string DRIVER_RUNNING_KEY_SUFFIX = "driver_running";

const std::string HAPTIC_DEVICES_SWAP_KEY = CHAI_REDIS_DRIVER_NAMESPACE + "::swap_devices";


std::string createRedisKey(const std::string& key_suffix, int device_number) {
	return CHAI_REDIS_DRIVER_NAMESPACE + "::device" + std::to_string(device_number) + "::" + key_suffix;
}

std::string createRobotRedisKey(const std::string& key_suffix,int robot_name) {
    return "robot_app::robot" + std::to_string(robot_name) + "::" + key_suffix;
}

// state transition key.
std::string STATE_TRANSITION_KEY = "system::state";
std::string ROBOT_NAME_KEY = "robot::name";