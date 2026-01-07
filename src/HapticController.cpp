/**
 * HapticController class implementation.
*/
#include <HapticController.h>
namespace Primitives
{
    HapticController::HapticController(const Eigen::Vector3d center_position_robot,const Eigen::Matrix3d center_rotation_robot,
                                        const Eigen::Matrix3d Rotation_Matrix_DeviceToRobot)
    {
        //Send zero force feedback to the haptic device
        _commanded_force_device.setZero();
        _commanded_torque_device.setZero();
        _commanded_gripper_force_device = 0.0;
        // Initialize sensed force to zero.
        _sensed_task_force.setZero();
        // Initialize homing task.
        device_homed = true;
        // reset home position. (Can be set using setDeviceCenter Function)
        _home_position_device.setZero();
        _home_rotation_device.setIdentity();

        // Initialize workspace center of the controlled robot (Can use SetRobotCenter function)
        _center_position_robot = center_position_robot;
        _center_rotation_robot = center_rotation_robot;

        //Initialize the frame trasform from device to robot
	    _Rotation_Matrix_DeviceToRobot = Rotation_Matrix_DeviceToRobot;

        //Initialize the set position and orientation of the controlled robot
        _desired_position_robot = _center_position_robot;
        _desired_rotation_robot = _center_rotation_robot;

        // set Initial robot position and velocity.
        _current_position_robot = _center_position_robot;
        _current_rotation_robot = _center_rotation_robot;

        _current_gripper_position_device = 0.0;
	    _current_gripper_velocity_device = 0.0;

        //Initialize position controller parameters
        _kp_position_ctrl_device = 0.15;
        _kv_position_ctrl_device = 0.5;
        _kp_orientation_ctrl_device = 0.5;
        _kv_orientation_ctrl_device = 0.2;

        _reduction_factor_torque_feedback << 1/20.0, 0.0, 0.0,
						  0.0, 1/20.0, 0.0,
						  0.0, 0.0, 1/20.0;

        _reduction_factor_force_feedback << 1/2.0, 0.0, 0.0,
                            0.0, 1/2.0, 0.0,
                            0.0, 0.0, 1/2.0;

        // Initialize scaling factors (can be set through setScalingFactors())
        _scaling_factor_trans=1.0;
        _scaling_factor_rot=1.0;

        //Device specifications
        _max_linear_stiffness_device = 0.0;
        _max_angular_stiffness_device = 0.0;
        _max_linear_damping_device = 0.0;
        _max_angular_damping_device = 0.0;
        _max_force_device = 0.0;
        _max_torque_device = 0.0;
    }

    HapticController::~HapticController()
    {
        _commanded_force_device.setZero();
        _commanded_torque_device.setZero();
        _commanded_gripper_force_device = 0.0;

        _desired_position_robot = _center_position_robot;
        _desired_rotation_robot = _center_rotation_robot;
    }

    void HapticController::reInitializeTask()
    {
        //Send zero force feedback to the haptic device
        _commanded_force_device.setZero();
        _commanded_torque_device.setZero();
        _commanded_gripper_force_device = 0.0;

        //Initialize the set position and orientation of the controlled robot
        _desired_position_robot = _current_position_robot;
        _desired_rotation_robot = _current_rotation_robot;

        first_iteration = true; // To initialize timer and integral terms
        _max_rot_velocity_device=0.001;
        _max_trans_velocity_device=0.001;
    }


    void HapticController::computeHapticCommand6d(Eigen::Vector3d& desired_position_robot, 
                                    Eigen::Matrix3d& desired_rotation_robot)
    {
        device_homed = false;
        if (first_iteration)
        {
            first_iteration = false;
            _desired_position_robot = _current_position_robot;
            _desired_rotation_robot = _current_rotation_robot;
        }
        // position of the device w.r.t home position.
        Eigen::Vector3d relative_position_device;
        relative_position_device = _current_position_device - _home_position_device;
        // Rotation of device w.r.t to home orientation.
        Eigen::Matrix3d relative_rotation_device = _current_rotation_device * _home_rotation_device.transpose();
        
        if (_ignore_z_rotation)
            removeZrotation(relative_rotation_device);

        Eigen::AngleAxisd relative_orientation_angle_device = Eigen::AngleAxisd(relative_rotation_device);
        
        // compute force feedback in robot frame.
        Eigen::Vector3d f_task_trans;
        Eigen::Vector3d f_task_rot;

        // use sensed force as the feedback force at this moment.
        // Read sensed task force
        f_task_trans = _sensed_task_force;
       
        f_task_rot = Eigen::Vector3d::Zero();

        // Apply reduction factors to force feedback
        f_task_trans = _reduction_factor_force_feedback * f_task_trans;
        f_task_rot = _reduction_factor_torque_feedback * f_task_rot;

        //Transfer task force from robot to haptic device global frame
        _commanded_force_device = _Rotation_Matrix_DeviceToRobot * f_task_trans;
        _commanded_torque_device = _Rotation_Matrix_DeviceToRobot * f_task_rot;

        // Scaling of the force feedback
        Eigen::Matrix3d scaling_factor_trans;
        Eigen::Matrix3d scaling_factor_rot;

        scaling_factor_trans << 1/_scaling_factor_trans, 0.0, 0.0,
						  0.0, 1/_scaling_factor_trans, 0.0,
						  0.0, 0.0, 1/_scaling_factor_trans;
		scaling_factor_rot << 1/_scaling_factor_rot, 0.0, 0.0,
						  0.0, 1/_scaling_factor_rot, 0.0,
						  0.0, 0.0, 1/_scaling_factor_rot;

        _commanded_force_device = scaling_factor_trans * _commanded_force_device;
        _commanded_torque_device = scaling_factor_rot * _commanded_torque_device;

        if(!_send_haptic_feedback)
        {
            _commanded_force_device.setZero();
            _commanded_torque_device.setZero();
        }
        // Compute the new desired position from the haptic device
        _desired_position_robot = _scaling_factor_trans * relative_position_device;
        // Compute set orientation from the haptic device
        Eigen::AngleAxisd desired_rotation_robot_aa = Eigen::AngleAxisd(_scaling_factor_rot* relative_orientation_angle_device.angle(), relative_orientation_angle_device.axis());
        _desired_rotation_robot = desired_rotation_robot_aa.toRotationMatrix();

        // Transfer set position and orientation from device to robot frame.
        _desired_position_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_position_robot;
        _desired_rotation_robot = _Rotation_Matrix_DeviceToRobot.transpose() * _desired_rotation_robot * _Rotation_Matrix_DeviceToRobot * _center_rotation_robot;

        // Adjust set position to the center of the task workspace
        _desired_position_robot = _desired_position_robot + _center_position_robot;

        // Send set position and orientation to the robot
        desired_position_robot = _desired_position_robot;
        desired_rotation_robot = _desired_rotation_robot;

    }

    void HapticController::HomingTask()
    {
        // Haptice device position controller gains
        double kp_position_ctrl_device =_kp_position_ctrl_device * _max_linear_stiffness_device;
        double kv_position_ctrl_device =_kv_position_ctrl_device * _max_linear_damping_device;
        double kp_orientation_ctrl_device =_kp_orientation_ctrl_device * _max_angular_stiffness_device;
        double kv_orientation_ctrl_device =_kv_orientation_ctrl_device * _max_angular_damping_device;

        _commanded_force_device = - kp_position_ctrl_device *( _current_position_device - _home_position_device)
                                - kv_position_ctrl_device * (_current_trans_velocity_device );
        // compute control torque.
        Eigen::Vector3d orientation_error;
        orientationError(orientation_error,_home_rotation_device,_current_rotation_device);
        
        _commanded_torque_device = -kp_orientation_ctrl_device*orientation_error -_kv_orientation_ctrl_device*_current_rot_velocity_device;

        // saturate to max force and torque of the device.

        if (_commanded_force_device.norm() > _max_force_device)
        {
            _commanded_force_device = _max_force_device * _commanded_force_device/_commanded_force_device.norm();
        }
        if (_commanded_torque_device.norm() > _max_torque_device)
        {
            _commanded_torque_device = _max_torque_device * _commanded_torque_device/ _commanded_torque_device.norm();
        }
        
        // check whether the device has been homed properly.
        if ((_current_position_device - _home_position_device).norm() < 0.001)
            device_homed = true;
    }

    void HapticController::UseGripperAsSwitch()
    {
        double gripper_start_angle = 10*M_PI/180;
        double gripper_switch_angle = 5*M_PI/180;
        
        double gripper_force_click = 3.0;
        double gripper_force_switched = 1.0;

        double damping_force = -0.05 * _current_gripper_velocity_device;

        // gripper initialization.

        if (!gripper_init)
        {
            gripper_state = false;
            if (_current_gripper_position_device <= gripper_switch_angle)
            {
                _commanded_gripper_force_device = gripper_force_switched + damping_force;
            }
            else
            {
                _commanded_gripper_force_device = 0;
                gripper_init = true;
            }
            
        }
        else
        {
            if(_current_gripper_position_device > gripper_start_angle)
            {
                _commanded_gripper_force_device = 0.0;
                gripper_state = false;
            }
            else if(_current_gripper_position_device <= gripper_start_angle && 
                    _current_gripper_position_device >= gripper_switch_angle)
            {
                _commanded_gripper_force_device = damping_force + 0.1* gripper_force_switched;
                gripper_state = false;
            }
            else
            {
                _commanded_gripper_force_device = damping_force + gripper_force_switched;
                gripper_state = true;
            }
            
        }
        
    }

    void HapticController::setScalingFactors(const double scaling_factor_trans, const double scaling_factor_rot)
    {
        _scaling_factor_trans = scaling_factor_trans;
        _scaling_factor_rot = scaling_factor_rot;
    }

    void HapticController::setRobotCenter(const Eigen::Vector3d center_position_robot, 
                                                const Eigen::Matrix3d center_rotation_robot)
    {
        _center_position_robot = center_position_robot;
        _center_rotation_robot = center_rotation_robot;

        /// set desired position and rotation same as center position and rotation.
        _desired_position_robot = _center_position_robot;
        _desired_rotation_robot = _center_rotation_robot;
    }

    void HapticController::setDeviceCenter(const Eigen::Vector3d home_position_device, const Eigen::Matrix3d home_rotation_device)
    {
        _home_position_device = home_position_device;
        _home_rotation_device = home_rotation_device;
    }

    void HapticController::setDeviceRobotRotation(const Eigen::Matrix3d Rotation_Matrix_DeviceToRobot)
    {
        _Rotation_Matrix_DeviceToRobot = Rotation_Matrix_DeviceToRobot;
    }

    void HapticController::removeZrotation(Eigen::Matrix3d& _R)
    {
        // check for valid rotation.
        Eigen::Matrix3d Q1 = _R*_R.transpose() - Eigen::Matrix3d::Identity();
        if (Q1.norm() > 0.001)
            throw std::invalid_argument("Invalid rotation matrix. Inside removeZrotation()\n");
        double angle = atan2(_R(1,0),_R(0,0));
        Eigen::AngleAxisd newAxis = Eigen::AngleAxisd(-angle,Eigen::Vector3d::UnitZ());
        _R = newAxis.toRotationMatrix() * _R;
    }

    void HapticController::orientationError(Eigen::Vector3d& orientation_error, Eigen::Matrix3d& _desired_orientation,
                                                                Eigen::Matrix3d& _current_orientation)
    {
         //check for valid rotation.
        Eigen::Matrix3d Q1 = _desired_orientation*_desired_orientation.transpose() - Eigen::Matrix3d::Identity();
        Eigen::Matrix3d Q2 = _current_orientation*_current_orientation.transpose() - Eigen::Matrix3d::Identity();

        if (Q1.norm() >0.001 || Q2.norm() > 0.001)
        {
            std::cout << "Desired Orientation: " << _desired_orientation << std::endl;
            std::cout << "Current Orientation: " <<_current_orientation << std::endl;
            std::cout << "Q1 norm: " << Q1.norm() << std::endl;
            std::cout << "Q2 norm: " << Q2.norm() << std::endl;
            throw std::invalid_argument("Invalid rotation matrices. DModel orientation error\n");
            return; 
        }
        else
        {
            Eigen::Vector3d rc1 = _current_orientation.block<3,1>(0,0);
            Eigen::Vector3d rc2 = _current_orientation.block<3,1>(0,1);
            Eigen::Vector3d rc3 = _current_orientation.block<3,1>(0,2);
            Eigen::Vector3d rd1 = _desired_orientation.block<3,1>(0,0);
            Eigen::Vector3d rd2 = _desired_orientation.block<3,1>(0,1);
            Eigen::Vector3d rd3 = _desired_orientation.block<3,1>(0,2);
            orientation_error = (-1/2)*(rc1.cross(rd1) + rc2.cross(rd2) + rc3.cross(rd3));
        }
    }
} // namespace Primitives

