#include "hd_haptics_helper.h"


namespace PhantomDevice
{
    // Phantomdevice functions.


    void hdPhantomDeviceHandler::open()
    {
        // setup haptic device.
        device = hdInitDevice(HD_DEFAULT_DEVICE);
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            // hduPrintError(stderr,&error, "failed to initialize device.");
            std::cout << "failed to initialize device." << std::endl;
            exit(-1);
        }
        std::cout << "found device: " << hdGetString(HD_DEVICE_MODEL_TYPE) << std::endl;
        // enable force feedback and clamping.
        hdEnable(HD_FORCE_OUTPUT);
        hdEnable(HD_MAX_FORCE_CLAMPING);
        // get the current device.
        device = hdGetCurrentDevice();
        hdStartScheduler();
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            std::cout << "failed to start scheduler. " << "Error: " << &error << std::endl;
            exit(-1);
        }
    
        // set the variables.
        _position.setZero();
        _rotation.setZero();
        _linear_velocity.setZero();
        _angular_velocity.setZero();
        _sensed_force.setZero();
        _sensed_torque.setZero();
        _applied_force.setZero();
        _applied_torque.setZero();

        m_LinearStiffnes = (double)HD_NOMINAL_MAX_STIFFNESS;
        m_AngularStiffness = (double)HD_NOMINAL_MAX_TORQUE_STIFFNESS;

        m_LinearDamping = (double) HD_NOMINAL_MAX_DAMPING;
        m_AngularDamping = (double) HD_NOMINAL_MAX_TORQUE_DAMPING;

        m_maxLinearForce = (double) HD_NOMINAL_MAX_FORCE;
        m_maxAngularForce = (double) HD_NOMINAL_MAX_TORQUE_FORCE;

        // setup RedisClient.
        redis_client = std::make_unique<RedisClient>();
    }

    void hdPhantomDeviceHandler::close()
    {
        hdStopScheduler();
        if (device != HD_INVALID_HANDLE)
        {
            hdDisableDevice(device);
            device = HD_INVALID_HANDLE;
        }
    }

    bool hdPhantomDeviceHandler::calibrate()
    {
        HDenum ret = hdCheckCalibration();
        if (ret == HD_CALIBRATION_OK)
            return true;
        else if (ret == HD_CALIBRATION_NEEDS_UPDATE)
        {
            hdUpdateCalibration(HD_CALIBRATION_AUTO);
            return true;
        }
        else
            return false;
        
    }

    void hdPhantomDeviceHandler::start()
    {
        gSchedulerCallback = hdScheduleAsynchronous(&phantomCallback,this,HD_DEFAULT_SCHEDULER_PRIORITY);

        HDErrorInfo error;
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            std::cout << "failed to start scheduler. " << "Error: " << &error << std::endl;
            exit(-1);
        }
        std::cout << "configured callback." << std::endl;
    }

    void hdPhantomDeviceHandler::setForceAndTorque(const Eigen::Vector3d& _f,const Eigen::Vector3d& _t)
    {
        _applied_force = _f;
        _applied_torque = _t;
    }

    RedisClient* hdPhantomDeviceHandler::getRedisClient()
    {
        return redis_client.get();
    }

    void hdPhantomDeviceHandler::ClampValue(Eigen::Vector3d& _v, double lv, double uv)
    {
        for (int i = 0; i < 3; i++)
        {
            if (_v[i] < lv)
                _v[i] = lv;
            else if (_v[i] > uv)
                _v[i] = uv;
        }
    }

} // namespace PhantomDevice


// int main(int argc, char const *argv[])
// {
//     auto handler = PhantomDevice::hdPhantomDeviceHandler();
//     handler.open();
//     handler.start();
//     while (runloop)
//     {
//         std::cout << handler._position << std::endl;
//     }
//     handler.close();
    
//     return 0;
// }
