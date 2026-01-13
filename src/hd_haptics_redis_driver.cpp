#include <iostream>
#include <ostream>
/**
 * include Phantom HD headers.
*/
#include "hd_haptics_helper.h"
/**
 * include Eigen headers.
*/
#include <Eigen/Core>
#include <Eigen/Dense>

#include <chai_haptics_driver_redis_keys.h>
#include <LoopTimer.h>
#include <csignal>

using namespace ChaiHapticDriverKeys;
using namespace PhantomDevice;

bool calibrate_flag = false;
unsigned int device_num = 0;
int null = 0;
bool runloop = true;
void sighandler(int sig) { runloop = false; }

int main(int argc, char const *argv[])
{
    signal(SIGABRT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGINT,sighandler);

    // create some variables.
    int switch_pressed = 0;
    double gripper_force = 0.0;
    double gripper_pose = 0.0;
    int use_gripper_switch = 0;

    hdPhantomDeviceHandler handler;
    handler.open();
    // define a redis_client object.
    // std::unique_ptr<RedisClient> redis_client = std::make_unique<RedisClient>();

    RedisClient* redis_client = handler.getRedisClient();
    redis_client->connect();
    // send device parameters to redis.
    redis_client->setEigenMatrix(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX,device_num),
                                    Eigen::Vector3d(handler.m_LinearStiffnes,
                                                    handler.m_AngularStiffness,0));
    redis_client->setEigenMatrix(createRedisKey(MAX_DAMPING_KEY_SUFFIX,device_num),
                                    Eigen::Vector3d(handler.m_LinearDamping,
                                                    handler.m_AngularDamping,0));
    redis_client->setEigenMatrix(createRedisKey(MAX_FORCE_KEY_SUFFIX,device_num),
                                            Eigen::Vector3d(handler.m_maxLinearForce,
                                                handler.m_maxAngularForce,0));
    redis_client->set(createRedisKey(MAX_GRIPPER_ANGLE,device_num),
                          std::to_string(null));

    // set commanded parameters to zero in redis.
    redis_client->setEigenMatrix(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,device_num), 
                                    handler._applied_force);
    redis_client->setEigenMatrix(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,device_num),
                                    handler._applied_torque);
    redis_client->set(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),
                        std::to_string(null));
    redis_client->set(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX,device_num),
                        std::to_string(null));
    redis_client->set(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX,device_num),
                        std::to_string(null));

    // set swap_device.
    Eigen::Vector2d swap_devices = Eigen::Vector2d::Zero();
    redis_client->setEigenMatrix(HAPTIC_DEVICES_SWAP_KEY,swap_devices);

    // setup read callbacks first.
    redis_client->createEigenReadCallback(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,device_num),
                                            handler._applied_force);
    redis_client->createEigenReadCallback(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,device_num),
                                            handler._applied_torque);
    redis_client->createDoubleReadCallback(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),
                                            gripper_force,1);
    redis_client->createIntReadCallback(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX,device_num),
                                            use_gripper_switch,1);
    // setup write callbacks.
    redis_client->createEigenWriteCallback(createRedisKey(POSITION_KEY_SUFFIX,device_num),
                                            handler._position);
    redis_client->createEigenWriteCallback(createRedisKey(ROTATION_KEY_SUFFIX,device_num),
                                            handler._rotation);
    redis_client->createDoubleWriteCallback(createRedisKey(GRIPPER_POSITION_KEY_SUFFIX,device_num),
                                            gripper_pose,1);
    redis_client->createEigenWriteCallback(createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX,device_num),
                                            handler._linear_velocity);
    redis_client->createEigenWriteCallback(createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX,device_num),
                                            handler._angular_velocity);
    redis_client->createDoubleWriteCallback(createRedisKey(GRIPPER_VELOCITY_KEY_SUFFIX,device_num),
                                            gripper_force,1);
    redis_client->createEigenWriteCallback(createRedisKey(SENSED_FORCE_KEY_SUFFIX,device_num),
                                            handler._sensed_force);
    redis_client->createEigenWriteCallback(createRedisKey(SENSED_TORQUE_KEY_SUFFIX,device_num),
                                            handler._sensed_torque);
    redis_client->createIntWriteCallback(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX,device_num),
                                            switch_pressed,1);  

    // Initializing redis_driver.
    std::cout << "Starting Phantom haptic redis driver..." << std::endl;
    redis_client->set(createRedisKey(DRIVER_RUNNING_KEY_SUFFIX,device_num),std::to_string(true));
    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();
    // start haptic device servoloop.
    handler.start();

    try
    {
        while (runloop && timer.WaitForNextLoop())
        {
            // redis_client->executeAllReadCallbacks();
            // // handler._applied_force.setZero();
            // // handler._applied_torque.setZero();
            // redis_client->executeAllWriteCallbacks();
            redis_client->executeBatchAllReadWriteCallbacks();
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    timer.printTimerHistory();

    // close the haptic device and write to redis.
    redis_client->set(createRedisKey(DRIVER_RUNNING_KEY_SUFFIX,device_num),std::to_string(false));
    redis_client->setEigenMatrix(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,device_num),
                                    Eigen::Vector3d(0,0,0));
    redis_client->setEigenMatrix(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,device_num),
                                    Eigen::Vector3d(0,0,0));
    redis_client->set(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,device_num),
                        std::to_string(0.0));
    handler.setForceAndTorque(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0));
    handler.close();

    
    return 0;
}

