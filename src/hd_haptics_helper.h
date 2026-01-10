#ifndef HD_HAPTICS_HELPER_H
#define HD_HAPTICS_HELPER_H

#include <iostream>
#include <ostream>
#include <memory>

#include <HD/hd.h>
#include <HDU/hdu.h>
#include <HDU/hduVector.h>
#include <HDU/hduHapticDevice.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduError.h>
/**
 * include Eigen headers.
*/
#include <Eigen/Core>
#include <Eigen/Dense>

#include <redisclient.h>




namespace PhantomDevice
{
    // struct DeviceInfo
    // {
    //     std::string model;
    //     std::string driver_version;
    //     std::string firmware_version;
    //     int serial_number;
    //     std::string vendor;

    //     DeviceInfo()
    //     {
    //         model = std::to_string(HD_DEVICE_MODEL_TYPE);
    //         driver_version = std::to_string(HD_DEVICE_DRIVER_VERSION);
    //         firmware_version = std::to_string(HD_DEVICE_FIRMWARE_VERSION);
    //         serial_number = (int)HD_DEVICE_SERIAL_NUMBER;
    //         vendor = std::to_string(HD_DEVICE_VENDOR);
    //     }

    // };

    // std::ostream& operator<< (std::ostream& os, const DeviceInfo& device)
    // {
    //     std::cout << "----------------------------------------------" << std::endl;
    //     os << "Model: " << device.model << "\n"
    //        << "Driver Version: " << device.driver_version << "\n"
    //        << "Firmware Version: " << device.firmware_version << "\n"
    //        << "Serial Number: " << device.serial_number << "\n"
    //        << "Vendor: " << device.vendor << std::endl;
    //     std::cout << "----------------------------------------------" << std::endl;
    //     return os;
    // }

    class hdPhantomDeviceHandler
    {
    private:
        HHD device;
        HDSchedulerHandle gSchedulerCallback = HD_INVALID_HANDLE;

        // DeviceInfo info;
        HDErrorInfo error;
        // declare a pointer for redis client.
        std::unique_ptr<RedisClient> redis_client;
    public:
        hdPhantomDeviceHandler() {device = HD_INVALID_HANDLE;};
        ~hdPhantomDeviceHandler() {};

        static HDCallbackCode HDCALLBACK phantomCallback(void* data)
        {
            hdPhantomDeviceHandler* _handler = static_cast<hdPhantomDeviceHandler*>(data);
            // read all data from redis.
            // _handler->redis_client->executeAllReadCallbacks();
            hdBeginFrame(_handler->device);
            hduVector3Dd _v;
            hdGetDoublev(HD_CURRENT_POSITION,_v);
            _handler->_position << _v[0]/1000, -_v[2]/1000, _v[1]/1000;
            // std::cout << _v << std::endl;
            // rotation to be set.
            hduMatrix mat;
            hdGetDoublev(HD_CURRENT_TRANSFORM,mat);
            mat.transpose();
            _handler->_rotation(0,0) = mat[0][0];
            _handler->_rotation(0,1) = mat[0][1];
            _handler->_rotation(0,2) = mat[0][2];

            _handler->_rotation(1,0) = mat[1][0];
            _handler->_rotation(1,1) = mat[1][1];
            _handler->_rotation(1,2) = mat[1][2];

            _handler->_rotation(2,0) = mat[2][0];
            _handler->_rotation(2,1) = mat[2][1];
            _handler->_rotation(2,2) = mat[2][2];
            // linear and angular velocities.
            hdGetDoublev(HD_CURRENT_VELOCITY, _v);
            _handler->_linear_velocity << _v[0]/1000, _v[1]/1000, _v[2]/1000;
            hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY, _v);
            _handler->_angular_velocity << _v[0], _v[1], _v[2];

            hdGetDoublev(HD_CURRENT_FORCE,_v);
            _handler->_sensed_force << _v[0], _v[1], _v[2];
            hdGetDoublev(HD_CURRENT_TORQUE, _v);
            _handler->_sensed_torque << _v[0], _v[1], _v[2];

            // clamp the value.
            _handler->ClampValue(_handler->_applied_force,-1.0,1.0);
            // set applied force and torque.
            _v[0] = _handler->_applied_force[0];
            _v[1] = _handler->_applied_force[1];
            _v[2] = _handler->_applied_force[2];
            hdSetDoublev(HD_CURRENT_FORCE,_v);

            _v[0] = _handler->_applied_torque[0];
            _v[1] = _handler->_applied_torque[1];
            _v[2] = _handler->_applied_torque[2];
            hdSetDoublev(HD_CURRENT_TORQUE,_v);
            hdEndFrame(_handler->device);

            // write data to redis.
            // _handler->redis_client->executeAllWriteCallbacks();

            HDErrorInfo error;
            if (HD_DEVICE_ERROR(error = hdGetError()))
            {
                std::cout << "Error during scheduling callback" << "Error: " << &error << std::endl;
                // if (hduIsSchedulerError(&error))
                // {
                //     return HD_CALLBACK_DONE;
                // }
                return HD_CALLBACK_DONE;
            }
            return HD_CALLBACK_CONTINUE;
        }

        void open();
        void close();
        bool calibrate();

        void getPosition(Eigen::Vector3d& pos);
        void getRotation(Eigen::Matrix3d& rot);
        void getLinearVelocity(Eigen::Vector3d& lin_vel);
        void getAngularVelocity(Eigen::Vector3d& ang_vel);

        void getForce(Eigen::Vector3d& _force);
        void getTorque(Eigen::Vector3d& _torque);

        void setForceAndTorque(const Eigen::Vector3d& _F, const Eigen::Vector3d& _T);
        void start();
        RedisClient* getRedisClient();
    
        Eigen::Vector3d _position;
        Eigen::Vector3d _linear_velocity;
        Eigen::Vector3d _angular_velocity;
        Eigen::Matrix3d _rotation;
        Eigen::Vector3d _sensed_force;
        Eigen::Vector3d _sensed_torque;

        Eigen::Vector3d _applied_force;
        Eigen::Vector3d _applied_torque;
        void ClampValue(Eigen::Vector3d& _v, double lv, double uv);

        // device constants.
        double m_LinearStiffnes;
        double m_AngularStiffness;
        
        double m_LinearDamping;
        double m_AngularDamping;

        double m_maxLinearForce;
        double m_maxAngularForce;
    };

    // void exithandler();
    
} // namespace PhantomDevice




#endif