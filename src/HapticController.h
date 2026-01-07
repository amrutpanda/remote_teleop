/**
 * Haptic controller Class to control haptic device.
 * This class works by setting up a redis connection with the chai_haptic_device_redis_driver.
*/

#ifndef _HAPTIC_CONTROLLER_H
#define _HAPTIC_CONTROLLER_H

#include <iostream>
#include <Eigen/Dense>
#include <chrono>

namespace Primitives
{
    
    class HapticController
    {
    private:
        double _scaling_factor_trans;
        double _scaling_factor_rot;

        //Position controller parameters for homing task
        double _kp_position_ctrl_device;
        double _kv_position_ctrl_device;
        double _kp_orientation_ctrl_device;
        double _kv_orientation_ctrl_device;

        // Force feedback controller parameters
        double _kp_robot_trans_velocity;
        double _kp_robot_rot_velocity;
        double _kv_robot_rot_velocity;
        double _kv_robot_trans_velocity;

        Eigen::Matrix3d _reduction_factor_force_feedback;
        Eigen::Matrix3d _reduction_factor_torque_feedback;

        // Time parameters.
        std::chrono::high_resolution_clock::time_point _t_prev;
        std::chrono::high_resolution_clock::time_point _t_curr;
        std::chrono::duration<double> _t_diff;

    public:
        HapticController(const Eigen::Vector3d center_position_robot,const Eigen::Matrix3d center_rotation_robot,
                                        const Eigen::Matrix3d Rotation_Matrix_DeviceToRobot);
        ~HapticController();

        void reInitializeTask();

        void computeHapticCommand6d(Eigen::Vector3d& _desired_robot_position, 
                                    Eigen::Matrix3d& _desired_robot_rotation);
        void computeHapticCommand3d(Eigen::Vector3d& _desired_robot_rotation);

        void HomingTask();

        void UseGripperAsSwitch();

        // Parameter setting methods.
        void setScalingFactors(const double _scaling_factor_trans,
                                const double _scaling_factor_rot);
        void setRobotCenter(const Eigen::Vector3d center_position_robot,
                                const Eigen::Matrix3d center_rotation_robot);
        void setDeviceCenter(const Eigen::Vector3d home_position_device,
                                const Eigen::Matrix3d home_rotation_device = Eigen::Matrix3d::Identity());
        void setDeviceRobotRotation(const Eigen::Matrix3d _Rotation_Matrix_DeviceToRobot = 
                                                Eigen::Matrix3d::Identity());
        void removeZrotation(Eigen::Matrix3d& _R);

        void orientationError(Eigen::Vector3d& orientation_error, Eigen::Matrix3d& _desired_orientation,
                                                                Eigen::Matrix3d& _current_orientation);

        // these variables to set by user.
        bool _send_haptic_feedback = false;
        // remove rotation along z direction.
        bool _ignore_z_rotation = false;

        double _max_linear_stiffness_device;
        double _max_angular_stiffness_device;
        double _max_linear_damping_device;
        double _max_angular_damping_device;
        double _max_force_device;
        double _max_torque_device;

        double _device_workspace_radius_limit;
        double _device_workspace_angle_limit;
        // Device status
        bool device_homed;
        bool first_iteration;
        // Gripper status
        bool gripper_init = false;
        bool gripper_state = false;
        // Set force and torque feedback of haptic device
        Eigen::Vector3d _commanded_force_device;
        Eigen::Vector3d _commanded_torque_device;
        double _commanded_gripper_force_device;
        // Sensed force and torque from the haptic device
        Eigen::Vector3d _sensed_force_device;
        Eigen::Vector3d _sensed_torque_device;

        // haptic device position and rotation.
        Eigen::Vector3d _current_position_device;
        Eigen::Matrix3d _current_rotation_device;
        // haptic device linear and angular velocity.
        Eigen::Vector3d _current_trans_velocity_device;
        Eigen::Vector3d _current_rot_velocity_device;
        double _current_gripper_position_device;
        double _current_gripper_velocity_device;
        // haptic device maximum velocity.
        double _max_trans_velocity_device = 0.0;
        double _max_rot_velocity_device = 0.0;



        // commanded position and orientation of the robot.
        Eigen::Vector3d _desired_position_robot;
        Eigen::Matrix3d _desired_rotation_robot;
        // current position and orientation of the robot.
        Eigen::Vector3d _current_position_robot;
        Eigen::Matrix3d _current_rotation_robot;
        // Sensed task force.
        Eigen::Vector3d _sensed_task_force;
        Eigen::Vector3d _sensed_task_torque;
        // Device task force
        Eigen::Vector3d _device_force;
        Eigen::Vector3d _device_torque;

        // Workspace related attributes.
        // haptic device home position and rotation.
        Eigen::Vector3d _home_position_device;
        Eigen::Matrix3d _home_rotation_device;
        // Workspace center of the controlled robot in robot frame.
        Eigen::Vector3d _center_position_robot;
        Eigen::Matrix3d _center_rotation_robot;

        // Transformation matrix from device frame to robot frame;
        Eigen::Matrix3d _Rotation_Matrix_DeviceToRobot;   
    };


} // namespace Primitives

#endif