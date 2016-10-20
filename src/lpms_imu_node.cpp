/** 
 * @file
 * @brief ROS driver for the LP IMU sensor
 * 
 * @par Advertises
 *
 * - @b data Calibrated IMU data 
 * 
 * @par Parameters
 * 
 * - @b ~imu_type
 * - @b ~serial_port Serial port that the IMU is connected to (default /dev/ttyUSB0)
 * - ~frame_id Frame identifier if IMU reference frame for message header
 * - ~rate Update rate (default 50Hz)
 */

#include <string>
#include <map>

#include <boost/assign/list_of.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
 */
class LpImuProxy
{
 public:
    LpImuProxy() : nh("~")
    {
		// Initialize mapping of LPMS sensor types
		using boost::assign::map_list_of;
		device_map = map_list_of("DEVICE_LPMS_B", DEVICE_LPMS_B)
			                    ("DEVICE_LPMS_U", DEVICE_LPMS_U)
			                    ("DEVICE_LPMS_C", DEVICE_LPMS_C)
			                    ("DEVICE_LPMS_BLE", DEVICE_LPMS_BLE)
			                    ("DEVICE_LPMS_RS232", DEVICE_LPMS_RS232)
			                    ("DEVICE_LPMS_B2", DEVICE_LPMS_B2)
			                    ("DEVICE_LPMS_U2", DEVICE_LPMS_U2)
                      			("DEVICE_LPMS_C2", DEVICE_LPMS_C2);
		
		// Connect to the LP IMU device
        manager = LpmsSensorManagerFactory();
        imu = manager->addSensor(DEVICE_LPMS_U2, "/dev/ttyUSB0");

		imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);
		mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
    }
    
    ~LpImuProxy(void)
    {
        manager->removeSensor(imu);
        delete manager;
    }

    void update(const ros::TimerEvent& te)
    {
        if (imu->getConnectionStatus() == SENSOR_CONNECTION_CONNECTED &&
                imu->hasImuData())
        {
            data = imu->getCurrentData();

			/* Fill the IMU message */

			// Fill the header
			// TODO: Use the timestamp provided by the IMU
			imu_msg.header.stamp = ros::Time::now();
			imu_msg.header.frame_id = "imu_global";

			// Fill orientation quaternion
			imu_msg.orientation.w = data.q[0];
			imu_msg.orientation.x = data.q[1];
			imu_msg.orientation.y = data.q[2];
			imu_msg.orientation.z = data.q[3];
			
			// Fill angular velocity data
			imu_msg.angular_velocity.x = data.w[0];
			imu_msg.angular_velocity.y = data.w[1];
			imu_msg.angular_velocity.z = data.w[2];

			// Fill linear acceleration data
			imu_msg.linear_acceleration.x = data.a[0];
			imu_msg.linear_acceleration.y = data.a[1];
			imu_msg.linear_acceleration.z = data.a[2];

			// \TODO: Fill covariance matrices
			// msg.orientation_covariance = ...
			// msg.angular_velocity_covariance = ...
			// msg linear_acceleration_covariance = ...

			/* Fill the magnetometer message */
			mag_msg.header.stamp = imu_msg.header.stamp;
			mag_msg.header.frame_id = imu_msg.header.frame_id;

			mag_msg.magnetic_field.x = data.b[0];
			mag_msg.magnetic_field.y = data.b[1];
			mag_msg.magnetic_field.z = data.b[2];

			// Publish the messages
			imu_pub.publish(imu_msg);
			mag_pub.publish(mag_msg);
        }
    }

	void run(void)
	{
		// The timer ensures periodic data publishing
		updateTimer = ros::Timer(nh.createTimer(ros::Duration(0.005),
												&LpImuProxy::update,
												this));
	}

 private:

    LpmsSensorManagerI* manager;
    LpmsSensorI* imu;
    ImuData data;

	ros::NodeHandle nh;
	ros::Timer updateTimer;
	ros::Publisher imu_pub, mag_pub;
	sensor_msgs::Imu imu_msg;
	sensor_msgs::MagneticField mag_msg;

	std::map<std::string,int> device_map;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_imu");
    ros::NodeHandle nh;

    LpImuProxy lpImu;

	lpImu.run();

    ros::spin();

    return 0;
}
