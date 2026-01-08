#include <HapticController.h>
#include <teleop_redis_keys.h>
// #include <teleop_redis_keys.h>
#include <redisclient.h>
#include <LoopTimer.h>

enum State {
    INIT = 0,
    FREE,
    SURGICAL,
    RECENTER
};
std::string state_names[] = {"INIT","FREE","SURGICAL","RECENTER"};

double clampVal(double val, double ll, double ul)
{
    if (val < ll)
        return ll;
    else if (val > ul)
        return ul;
    else
        return val;
}

/** function to handle delayed communication. Will run in a separate thread with low frequency.
    Helpful to have an estimate of delay and pass that as an argument.
    Unit is milliseconds.
 **/
void communication(unsigned int delay = 0);

bool runloop = true;
void sig_handler(int signum) {runloop = false;}

std::string _controller_running = "0";
int haptic_ready = 0;
int robot_ready = 0;

// no. of robots to be controlled.
int num_robots = 2;
int haptic_group = 0;
int robot_num = 1;

// Create redis client objects.
// define two redis clients one for local and anther for remote connection.
RedisClient redis_client, redis_client_remote;

int main(int argc, char const *argv[])
{
    signal(SIGINT,sig_handler);
    // RedisClient redis_client;
    // RedisClient redis_client_remote("127.0.0.1", 6379); // robot side redis server.

    // RedisClient redis_client_remote("100.88.10.20");
    
    redis_client.connect();
    redis_client_remote.connect("100.114.124.2");
    

    int device_z_rotation = 90; // relative rotation of haptic w.r.t to robot frame.
    // setup state 
    int state = INIT;
    int prev_state = state;

    // setup max and min force.
    double max_force = 10.0;
    double min_force = 0.1;
    double max_force_diff = 0.1; 

    double max_torque = 0.1;
    double min_torque = 0.001;
    double max_torque_diff = 0.001;

    // check whether the robot controller is running.
    if (redis_client_remote.get(CONTROLLER_RUNNING_KEY) == "0")
    {
        std::cout << "Run the robot controller before running haptic controller" << std::endl;
        return 0;
    }

    Eigen::Vector3d _haptic_proxy = Eigen::Vector3d::Zero();
    Eigen::Vector3d _robot_proxy = Eigen::Vector3d::Zero();
    Eigen::Matrix3d _robot_proxy_rot = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d _R_device_robot = Eigen::Matrix3d::Identity();

    Eigen::Vector3d robot_workspace_center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d robot_rotation_default = Eigen::Matrix3d::Identity();

    redis_client_remote.getEigenMatrix(createRobotRedisKey(ROBOT_DEFAULT_POS_KEY,1),robot_workspace_center);
    redis_client_remote.getEigenMatrix(createRobotRedisKey(ROBOT_DEFAULT_ROT_KEY,1),robot_rotation_default);

    // redis_client.getEigenMatrix(ROBOT_DEFAULT_POS_KEY,robot_workspace_center);
    // redis_client.getEigenMatrix(ROBOT_DEFAULT_ROT_KEY,robot_rotation_default);

    _R_device_robot *= Eigen::AngleAxisd(device_z_rotation* M_PI/180,Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Vector3d device_pos_center = Eigen::Vector3d::Zero();
    Eigen::Matrix3d device_rot_center = Eigen::Matrix3d::Identity();
    // Eigen::Matrix3d device_release_rot = Eigen::Matrix3d::Identity();

    auto teleop_task = new Primitives::HapticController(robot_workspace_center,robot_rotation_default,_R_device_robot);
    teleop_task->_send_haptic_feedback = true;
    Eigen::Vector3d _sensed_force_robot_frame = Eigen::Vector3d::Zero();
    Eigen::Vector3d _sensed_torque_robot_frame = Eigen::Vector3d::Zero();
    
    // gripper states.
    bool gripper_state = false;
    bool gripper_state_prev = false;
    // set device num for fetching info from haptic driver.
    int device_num = 0;

    _robot_proxy = robot_workspace_center;
    _robot_proxy_rot = robot_rotation_default;

    Eigen::Vector3d robot_pos = Eigen::Vector3d(0,0,0);
    Eigen::Matrix3d robot_rot = Eigen::Matrix3d::Identity();

    // fetch haptic device properties.
    Eigen::Vector3d _max_stiffness_device;
    Eigen::Vector3d _max_damping_device;
    Eigen::Vector3d _max_force_device;

    redis_client.getEigenMatrix(createRedisKey(DEVICE_MAX_STIFFNESS_KEY_SUFFIX,device_num),_max_stiffness_device);
    redis_client.getEigenMatrix(createRedisKey(DEVICE_MAX_DAMPING_KEY_SUFFIX,device_num),_max_damping_device);
    redis_client.getEigenMatrix(createRedisKey(DEVICE_MAX_FORCE_KEY_SUFFIX,device_num),_max_force_device);

    // set HapticController parameters.
    teleop_task->_max_linear_stiffness_device = _max_stiffness_device[0];
    teleop_task->_max_angular_stiffness_device = _max_stiffness_device[1];
    teleop_task->_max_linear_damping_device = _max_damping_device[0];
    teleop_task->_max_angular_damping_device = _max_damping_device[1];
    teleop_task->_max_force_device = _max_force_device[0];
    teleop_task->_max_torque_device = _max_force_device[1];

    // add scaling factors.
    double Ks = 1.5;
    double KsR = 1.0;
    teleop_task->setScalingFactors(Ks,KsR);

    // Vector to store desired force and prev_desired_force;
    Eigen::Vector3d _desired_force = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_desired_force = Eigen::Vector3d::Zero();

    // Vector to store desired torque and prev_desired_torque;
    Eigen::Vector3d desired_torque = Eigen::Vector3d::Zero();
    Eigen::Vector3d prev_desired_torque = Eigen::Vector3d::Zero();

    Eigen::Vector3d haptic_center = Eigen::Vector3d::Zero();

    // create a redis_client group for interacting with haptic driver.
    // read keys.
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_POSITION_KEY_SUFFIX,device_num),teleop_task->_current_position_device);
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_ROTATION_KEY_SUFFIX,device_num),teleop_task->_current_rotation_device);
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_LINEAR_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_trans_velocity_device);
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_ANGULAR_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_rot_velocity_device);
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_SENSED_FORCE_KEY_SUFFIX,device_num),teleop_task->_sensed_force_device);
    redis_client.createEigenGroupReadCallback(haptic_group,createRedisKey(DEVICE_SENSED_TORQUE_KEY_SUFFIX,device_num),teleop_task->_sensed_torque_device);
    redis_client.createDoubleGroupReadCallback(haptic_group,createRedisKey(DEVICE_GRIPPER_POSITION_KEY_SUFFIX,device_num),teleop_task->_current_gripper_position_device,1);
    redis_client.createDoubleGroupReadCallback(haptic_group,createRedisKey(DEVICE_GRIPPER_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_gripper_velocity_device,1);

    // write  robot keys.
    redis_client.createEigenGroupWriteCallback(haptic_group,createRedisKey(DEVICE_COMMANDED_FORCE_KEY_SUFFIX,device_num),teleop_task->_commanded_force_device);
    redis_client.createEigenGroupWriteCallback(haptic_group,createRedisKey(DEVICE_COMMANDED_TORQUE_KEY_SUFFIX,device_num),teleop_task->_commanded_torque_device);
    redis_client.createDoubleGroupWriteCallback(haptic_group,createRedisKey(DEVICE_COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),teleop_task->_commanded_gripper_force_device,1);
    // read haptic ready state.
    redis_client_remote.createIntGroupWriteCallback(haptic_group,HAPTIC_READY_STATE_KEY,haptic_ready,1);
   
    /* ------------------------------------------------------------------ */
    // Objects to read from redis server.
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_POSITION_KEY_SUFFIX,device_num),teleop_task->_current_position_device);
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_ROTATION_KEY_SUFFIX,device_num),teleop_task->_current_rotation_device);
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_LINEAR_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_trans_velocity_device);
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_ANGULAR_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_rot_velocity_device);
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_SENSED_FORCE_KEY_SUFFIX,device_num),teleop_task->_sensed_force_device);
    redis_client.createEigenReadCallback(createRedisKey(DEVICE_SENSED_TORQUE_KEY_SUFFIX,device_num),teleop_task->_sensed_torque_device);
    redis_client.createDoubleReadCallback(createRedisKey(DEVICE_GRIPPER_POSITION_KEY_SUFFIX,device_num),teleop_task->_current_gripper_position_device,1);
    redis_client.createDoubleReadCallback(createRedisKey(DEVICE_GRIPPER_VELOCITY_KEY_SUFFIX,device_num),teleop_task->_current_gripper_velocity_device,1);
    // read from robot.
    redis_client_remote.createEigenReadCallback(ROBOT_SENSED_FORCE_KEY,_sensed_force_robot_frame);
    redis_client_remote.createEigenReadCallback(ROBOT_SENSED_TORQUE_KEY,_sensed_torque_robot_frame);
    redis_client_remote.createEigenReadCallback(ROBOT_CURRENT_POS_KEY,robot_pos);
    redis_client_remote.createEigenReadCallback(ROBOT_CURRENT_ROT_KEY,robot_rot);
    redis_client_remote.createIntReadCallback(ROBOT_READY_STATE_KEY,robot_ready,1);

    // objects to write to redis.
    redis_client.createEigenWriteCallback(createRedisKey(DEVICE_COMMANDED_FORCE_KEY_SUFFIX,device_num),teleop_task->_commanded_force_device);
    redis_client.createEigenWriteCallback(createRedisKey(DEVICE_COMMANDED_TORQUE_KEY_SUFFIX,device_num),teleop_task->_commanded_torque_device);
    redis_client.createDoubleWriteCallback(createRedisKey(DEVICE_COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),teleop_task->_commanded_gripper_force_device,1);

    // write to robot keys.
    redis_client_remote.createEigenWriteCallback(ROBOT_TARGET_POS_KEY,_robot_proxy);
    redis_client_remote.createEigenWriteCallback(ROBOT_TARGET_ROT_KEY,_robot_proxy_rot);
    redis_client_remote.createIntWriteCallback(HAPTIC_READY_STATE_KEY,haptic_ready,1);
    /* ------------------------------------------------------------------ */


    // create redis keys to read and write to robots.
    // for (int i = 1; i <= num_robots; i++)
    // {
    //     // robot read keys.
    //     redis_client.createEigenGroupReadCallback(i,createRobotRedisKey(ROBOT_SENSED_FORCE_KEY,i),_sensed_force_robot_frame);
    //     redis_client.createEigenGroupReadCallback(i,createRobotRedisKey(ROBOT_SENSED_TORQUE_KEY,i),_sensed_torque_robot_frame);
    //     redis_client.createEigenGroupReadCallback(i,createRobotRedisKey(ROBOT_CURRENT_POS_KEY,i),robot_pos);
    //     redis_client.createEigenGroupReadCallback(i,createRobotRedisKey(ROBOT_CURRENT_ROT_KEY,i),robot_rot);
    //     redis_client.createIntGroupReadCallback(i,createRobotRedisKey(ROBOT_READY_STATE_KEY,i),robot_ready,1);
    //     // robot write keys.
    //     redis_client.createEigenGroupWriteCallback(i,createRobotRedisKey(ROBOT_TARGET_POS_KEY,i),_robot_proxy);
    //     redis_client.createEigenGroupWriteCallback(i,createRobotRedisKey(ROBOT_TARGET_ROT_KEY,i),_robot_proxy_rot);
    // }
    // // state specific callbacks.
    // redis_client.createIntGroupReadCallback(3,STATE_TRANSITION_KEY,state,1);
    // redis_client.createIntGroupReadCallback(3,ROBOT_NAME_KEY,robot_num,1);

    // create timer.
    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    runloop = true;

    while (runloop && timer.WaitForNextLoop())
    {
        // make the state obtain from redis as prev state.
        prev_state = state;
        // read from redis.
        redis_client.executeAllReadCallbacks();
        // use gripper as switch.
        // teleop_task->UseGripperAsSwitch();
        gripper_state_prev = gripper_state;
        gripper_state = teleop_task->gripper_state;
        // handle state transition.
        if (state != prev_state)
        {
            std::cout << "Transitioning from state " << state_names[prev_state] << " to "
                                            << state_names[state] << std::endl;
        }

        if (state == INIT)
        {
            // home the device.
            // teleop_task->HomingTask();
            if (teleop_task->device_homed && gripper_state && robot_ready)
            {
                teleop_task->setDeviceCenter(teleop_task->_current_position_device,teleop_task->_current_rotation_device);
                device_pos_center = teleop_task->_current_position_device;
                device_rot_center = teleop_task->_current_rotation_device;
                // reinitialize the haptic controller.
                teleop_task->reInitializeTask();
                // make haptic device ready.
                haptic_ready = 1;
                // state = FREE;
                return 0;
            }
            teleop_task->setDeviceCenter(teleop_task->_current_position_device,teleop_task->_current_rotation_device);
            device_pos_center = teleop_task->_current_position_device;
            device_rot_center = teleop_task->_current_rotation_device;
            // reinitialize the haptic controller.
            teleop_task->reInitializeTask();
            state = FREE;

        }
        else if (state == FREE)
        {
            device_rot_center = teleop_task->_current_rotation_device;
            
            teleop_task->computeHapticCommand6d(_robot_proxy,_robot_proxy_rot);

            Eigen::Vector3d desired_force = 0.5 * teleop_task->_Rotation_Matrix_DeviceToRobot * (_sensed_force_robot_frame) +(-10.0)* teleop_task->_current_trans_velocity_device;
            // Eigen::Vector3d desired_force_diff = desired_force - prev_desired_force;

            // if (desired_force_diff.norm() > max_force_diff)
            // {
            //     desired_force = prev_desired_force + max_force_diff * desired_force_diff/desired_force_diff.norm();
            // }

            if (desired_force.norm() > max_force)
            {
                desired_force = max_force * desired_force/desired_force.norm();
            }
            // else if (_desired_force.norm() < min_force)
            // {
            //     desired_force.setZero();
            // }

            desired_torque = 1.0 * teleop_task->_Rotation_Matrix_DeviceToRobot* (_sensed_torque_robot_frame);
            // set limit for torque.
            if (desired_torque.norm() > max_torque)
            {
                desired_torque = max_torque * desired_torque/desired_torque.norm();
            }
            // else if (desired_torque.norm() < min_torque)
            // {
            //     desired_torque.setZero();
            // }
            
            // std::cout << "desired torque: " << desired_torque.transpose() << std::endl;
            // send desired force to command force device.
            teleop_task->_commanded_force_device = desired_force;
            teleop_task->_commanded_torque_device = desired_torque; // ignoring force feedback at this moment.

            prev_desired_force = desired_force;
            prev_desired_torque = desired_torque;
            // std::cout << "commanded torque: " << teleop_task->_commanded_torque_device.transpose() << std::endl;
        }
        else if (state == SURGICAL)
        {
            teleop_task->setRobotCenter(_robot_proxy,_robot_proxy_rot);
            teleop_task->computeHapticCommand6d(_robot_proxy,_robot_proxy_rot);
        }

        redis_client.executeAllWriteCallbacks();
    }
    std::cout << "Exited haptic control loop " << std::endl;
    // send zero force and torque to haptic device.    
    Eigen::Vector3d _zero_value_vector = Eigen::Vector3d::Zero();
    double _zero_value = 0.0;
    redis_client.setEigenMatrix(createRedisKey(DEVICE_COMMANDED_FORCE_KEY_SUFFIX,device_num),_zero_value_vector);
    redis_client.setEigenMatrix(createRedisKey(DEVICE_COMMANDED_TORQUE_KEY_SUFFIX,device_num),_zero_value_vector);
    redis_client.set(createRedisKey(DEVICE_COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),std::to_string(_zero_value));

    timer.printTimerHistory();
    return 0;
}

void communication(unsigned int delay)
{
    double communication_freq = 50;
    if (delay != 0)
        communication_freq = 1000/delay;
    
    LoopTimer timer;
    timer.setLoopFrequency(communication_freq);
    timer.InitializeTimer();
    while (runloop)
    {
        timer.WaitForNextLoop();
        redis_client_remote.executeAllReadCallbacks();
        redis_client_remote.executeAllWriteCallbacks();
    }
    std::cout << "Exited comm loop" << std::endl;
    redis_client_remote.set(HAPTIC_READY_STATE_KEY,"0");
    timer.printTimerHistory();
}