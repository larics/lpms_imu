/*! \file lpms_imu_node.cpp
 *  \brief ROS driver for the LP IMU sensor
 */

#include <string>

#include "ros/ros.h"

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
 */
class LpImuProxy
{
 public:
    LpImuProxy(int devType, const std::string& devAddr)
    {
        manager = LpmsSensorManagerFactory();
        imu = manager->addSensor(devType, devAddr.c_str());
    }
    
    ~LpImuProxy(void)
    {
        manager->removeSensor(imu);
        delete manager;
    }

    void update(void)
    {
        if (imu->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
                imu->hasImuData())
        {
            data = imu->getCurrentData();
            ROS_INFO("Ts=%f, qW=%f, qX=%f, qY=%f, qZ=%f, x=%f, y=%f, z=%f\n", data.timeStamp, data.q[0], data.q[1], data.q[2], data.q[3], data.r[0], data.r[1], data.r[2]);
        }

    }

 private:
    LpmsSensorManagerI* manager;
    LpmsSensorI* imu;
    ImuData data;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_imu");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    LpImuProxy lpImu(DEVICE_LPMS_U2, "/dev/ttyUSB0");
    while (ros::ok())
    {
        
        lpImu.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
