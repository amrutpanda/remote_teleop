#include <signal.h>
#include <Eigen/Dense>
#include <string>

#include <chai3d.h>
#include <redisclient.h>
#include <chai_haptics_driver_redis_keys.h>

#include <LoopTimer.h>

using namespace ChaiHapticDriverKeys;

bool runloop = true;
void sighandler(int sig) { runloop = false; }

namespace 
{
    std::vector <chai3d::cGenericHapticDevicePtr> haptic_devices_ptr;
    // for swapping device.
    Eigen::Vector2d swap_devices = Eigen::Vector2d(0,0);
    // variables for haptic device.
    // set force and torque feedback for haptic device
    std::vector<Eigen::Vector3d> commanded_forces;
    std::vector<Eigen::Vector3d> commanded_torques;
    std::vector<double> commanded_force_grippers;
    // set position and rotation vectors.
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Matrix3d> rotations;
    std::vector<double> gripper_positions;
    // set linear velocity and angular velocity vectors.
    std::vector<Eigen::Vector3d> linear_velocities;
    std::vector<Eigen::Vector3d> angular_velocities;
    std::vector<double> gripper_velocities;
    // set sensed force and torque vectors.
    std::vector<Eigen::Vector3d> sensed_forces;
    std::vector<Eigen::Vector3d> sensed_torques;
    // variables for using gripper as switch.
    std::vector<int> use_gripper_as_switch;
    std::vector<int> switch_pressed;


} // namespace name


int main(int argc, char const *argv[])
{
    signal(SIGABRT,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGINT, sighandler);

    // start redis client.
    RedisClient* redis_client = new RedisClient();
    // RedisClient* redis_client = new RedisClient("10.11.2.107");
    redis_client->connect();

    // find and initialize connected hapic devices.
    auto handler = chai3d::cHapticDeviceHandler();
    const int num_devices = handler.getNumDevices();

    // check if haptic devices connected.
    if (num_devices == 0)
    {
        throw std::runtime_error("No Devices found. Connect a haptic device and try again\n");
    }
    else
    {
        std::cout << "Found device !\n" << "No. of Haptics device connected : " << num_devices << std::endl;
    }

    // check whether the device is open or not.
    for (int i = 0; i < num_devices; i++)
    {
        chai3d::cGenericHapticDevicePtr current_device_ptr;
        handler.getDevice(current_device_ptr,i);
        // check device open status.
        if(!current_device_ptr->open())
        {
            for (int k = 0; k < i; k++)
            {
                haptic_devices_ptr[k]->close();
            }
            throw std::runtime_error("Could not open haptic device. No: " + i);
        }
        // check for calibration.
        if (!current_device_ptr->calibrate())
        {
            for (int k = 0; k < i; k++)
            {
                haptic_devices_ptr[k]->close();
            }   
            throw std::runtime_error("Couldn't calibrate haptic device. You can calibrate manually. Device No: " + i);
        }  
        
        // get device info
        chai3d::cHapticDeviceInfo current_device_info;
        handler.getDeviceSpecifications(current_device_info,i);
        std::cout << "Device No: " << i << "\nDevice Info: " 
                                        << current_device_info.m_modelName << std::endl;

        if (current_device_info.m_modelName == "sigma7")
        {
            current_device_ptr.get()->enableForces(true);
        }
        
        // send zero force feedback to haptic devices.
        current_device_ptr->setForceAndTorqueAndGripperForce(Eigen::Vector3d(0,0,0),
                                                        Eigen::Vector3d(0,0,0),0);
        
        // save haptic device pointers.
        haptic_devices_ptr.push_back(current_device_ptr);
        redis_client->setEigenMatrix(createRedisKey(MAX_STIFFNESS_KEY_SUFFIX,i),
                                    Eigen::Vector3d(current_device_info.m_maxLinearStiffness,
                                                    current_device_info.m_maxAngularStiffness,
                                                    current_device_info.m_maxGripperLinearStiffness));
        redis_client->setEigenMatrix(createRedisKey(MAX_DAMPING_KEY_SUFFIX,i),
                                    Eigen::Vector3d(current_device_info.m_maxLinearDamping,
                                    current_device_info.m_maxAngularDamping,
                                    current_device_info.m_maxGripperAngularDamping));

        redis_client->setEigenMatrix(createRedisKey(MAX_FORCE_KEY_SUFFIX,i),
                                    Eigen::Vector3d(current_device_info.m_maxLinearForce,
                                                    current_device_info.m_maxAngularTorque,
                                                    current_device_info.m_maxGripperForce));

        redis_client->set(createRedisKey(MAX_GRIPPER_ANGLE,i),
                          std::to_string(current_device_info.m_gripperMaxAngleRad));
        // populate the vectors.
        commanded_forces.push_back(Eigen::Vector3d::Zero());
		commanded_torques.push_back(Eigen::Vector3d::Zero());
		commanded_force_grippers.push_back(0.0);
		positions.push_back(Eigen::Vector3d::Zero());
		rotations.push_back(Eigen::Matrix3d::Identity());
		gripper_positions.push_back(0.0);
		linear_velocities.push_back(Eigen::Vector3d::Zero());
		angular_velocities.push_back(Eigen::Vector3d::Zero());
		gripper_velocities.push_back(0.0);
		sensed_forces.push_back(Eigen::Vector3d::Zero());
		sensed_torques.push_back(Eigen::Vector3d::Zero());
		use_gripper_as_switch.push_back(0);
		switch_pressed.push_back(0);
        // finished populating.
        // set commanded parameters to zero in redis.
        redis_client->setEigenMatrix(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,i), 
                                        commanded_forces[i]);
        redis_client->setEigenMatrix(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,i),
                                        commanded_torques[i]);
        redis_client->set(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,i),
                            std::to_string(commanded_force_grippers[i]));
        redis_client->set(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX,i),
                            std::to_string(use_gripper_as_switch[i]));
        redis_client->set(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX,i),
                            std::to_string(switch_pressed[i]));
                                   
    }
    redis_client->setEigenMatrix(HAPTIC_DEVICES_SWAP_KEY,swap_devices);
    // create redis callback setup.
    for (int i = 0; i < num_devices; i++)
    {
        // setup read callbacks first.
        redis_client->createEigenReadCallback(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,i),
                                                commanded_forces.at(i));
        redis_client->createEigenReadCallback(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,i),
                                                commanded_torques.at(i));
        redis_client->createDoubleReadCallback(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,i),
                                                commanded_force_grippers.at(i),1);
        redis_client->createIntReadCallback(createRedisKey(USE_GRIPPER_AS_SWITCH_KEY_SUFFIX,i),
                                                use_gripper_as_switch.at(i),1);
        // setup write callbacks.
        redis_client->createEigenWriteCallback(createRedisKey(POSITION_KEY_SUFFIX,i),
                                                positions.at(i));
        redis_client->createEigenWriteCallback(createRedisKey(ROTATION_KEY_SUFFIX,i),
                                                rotations.at(i));
        redis_client->createDoubleWriteCallback(createRedisKey(GRIPPER_POSITION_KEY_SUFFIX,i),
                                                gripper_positions.at(i),1);
        redis_client->createEigenWriteCallback(createRedisKey(LINEAR_VELOCITY_KEY_SUFFIX,i),
                                                linear_velocities.at(i));
        redis_client->createEigenWriteCallback(createRedisKey(ANGULAR_VELOCITY_KEY_SUFFIX,i),
                                                angular_velocities.at(i));
        redis_client->createDoubleWriteCallback(createRedisKey(GRIPPER_VELOCITY_KEY_SUFFIX,i),
                                                gripper_velocities.at(i),1);
        redis_client->createEigenWriteCallback(createRedisKey(SENSED_FORCE_KEY_SUFFIX,i),
                                                sensed_forces.at(i));
        redis_client->createEigenWriteCallback(createRedisKey(SENSED_TORQUE_KEY_SUFFIX,i),
                                                sensed_torques.at(i)); 
        redis_client->createIntWriteCallback(createRedisKey(SWITCH_PRESSED_KEY_SUFFIX,i),
                                                switch_pressed.at(i),1);                         
    }
    redis_client->setEigenMatrix(HAPTIC_DEVICES_SWAP_KEY,swap_devices);
    // redis setup complete.

    std::cout << "Device initialization complete. Starting driver now...." << std::endl;
    
    for (int i = 0; i < num_devices; i++)
    {
        redis_client->set(createRedisKey(DRIVER_RUNNING_KEY_SUFFIX,i),std::to_string(true));
    }
    
    LoopTimer timer;
    timer.setLoopFrequency(1000);
    timer.InitializeTimer();

    try
    {
        while (runloop)
        {
            timer.WaitForNextLoop();
            redis_client->executeAllReadCallbacks();
            // continue;
            if (swap_devices(0) != 0 || swap_devices(1) != 0) {
				// cout << "swapping devices" << endl;
				if (swap_devices(0) != swap_devices(1) &&
					swap_devices(0) >= 0 && swap_devices(1) >= 0 &&
					swap_devices(0) < num_devices &&
					swap_devices(1) < num_devices) {
					std::cout << "swapping devices " << swap_devices(0) << " and "
						 << swap_devices(1) << endl;
					std::swap(haptic_devices_ptr.at((int)swap_devices(0)),
						 haptic_devices_ptr.at((int)swap_devices(1)));
				}
				redis_client->setEigenMatrix(HAPTIC_DEVICES_SWAP_KEY, Eigen::Vector2i(0, 0));
			}
            for (int i = 0; i < num_devices; i++)
            {
                // use haptic device as switch.
                haptic_devices_ptr.at(i)->setEnableGripperUserSwitch(use_gripper_as_switch.at(i));
                // send command quantities to haptic device.
                // if switch not enabled in any device. send zero command.
                if (use_gripper_as_switch.at(i))
                    commanded_force_grippers.at(i) = 0;
                if (!haptic_devices_ptr.at(i)->setForceAndTorqueAndGripperForce(commanded_forces.at(i),
                                                    commanded_torques.at(i), commanded_force_grippers.at(i)))
                {
                    std::cout << "Error in sending force command to haptic device\n" << 
                                "closing the driver" << std::endl;
                    runloop = false;
                    break;
                }

                chai3d::cVector3d position;
                chai3d::cMatrix3d rotation;
                chai3d::cVector3d linear_velocity;
                chai3d::cVector3d angular_velocity;

                haptic_devices_ptr.at(i)->getPosition(position);
                haptic_devices_ptr.at(i)->getRotation(rotation);
                haptic_devices_ptr.at(i)->getLinearVelocity(linear_velocity);
                haptic_devices_ptr.at(i)->getAngularVelocity(angular_velocity);

                positions.at(i) = position.eigen();
                rotations.at(i) = rotation.eigen();
                linear_velocities.at(i) = linear_velocity.eigen();
                angular_velocities.at(i) = angular_velocity.eigen();
                // get gripper switch/button press
				bool pressed;
				haptic_devices_ptr.at(i)->getUserSwitch(0, pressed);
				switch_pressed.at(i) = pressed ? 1 : 0;
				gripper_positions.at(i) = 0;
				gripper_velocities.at(i) = 0;
				if (!use_gripper_as_switch.at(i)) {
					haptic_devices_ptr.at(i)->getGripperAngleRad(
						gripper_positions.at(i));
					haptic_devices_ptr.at(i)->getGripperAngularVelocity(
						gripper_velocities.at(i));
				}

                // get sensed force and torque from the haptic device
				chai3d::cVector3d sensed_force;
				chai3d::cVector3d sensed_torque;
				haptic_devices_ptr.at(i)->getForce(sensed_force);
				haptic_devices_ptr.at(i)->getTorque(sensed_torque);
				sensed_forces.at(i) = sensed_force.eigen();
				sensed_torques.at(i) = sensed_torque.eigen();

            }
            redis_client->executeAllWriteCallbacks();
        }
        
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        runloop = false;
    }
    // prints timer summary.
    timer.printTimerHistory();
    
    // close all haptic devices.
    for (int i = 0; i < num_devices; i++)
    {
        redis_client->set(createRedisKey(DRIVER_RUNNING_KEY_SUFFIX,i),std::to_string(false));
        redis_client->setEigenMatrix(createRedisKey(COMMANDED_FORCE_KEY_SUFFIX,i),
                                        Eigen::Vector3d(0,0,0));
        redis_client->setEigenMatrix(createRedisKey(COMMANDED_TORQUE_KEY_SUFFIX,i),
                                        Eigen::Vector3d(0,0,0));
        redis_client->set(createRedisKey(COMMANDED_GRIPPER_FORCE_KEY_SUFFIX,i),
                            std::to_string(0.0));
        haptic_devices_ptr.at(i)->setForceAndTorqueAndGripperForce(chai3d::cVector3d(0,0,0),
                                        chai3d::cVector3d(0,0,0),0.0);
        haptic_devices_ptr.at(i)->close();
    }
    
    return 0;
}
