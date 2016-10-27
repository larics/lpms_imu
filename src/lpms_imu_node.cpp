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
 * - @b ~sensor_model LP sensor model identifier (the node has so far been tested with DEVICE_LPMS_U2)
 * - @b ~port The port that the IMU is connected to (default /dev/ttyUSB0)
 * - @b ~frame_id Frame identifier if IMU reference frame for message header (default imu_global)
 * - @b ~rate Update rate, in Hz (default 50)
 */

#include <string>
#include <map>
#include <iostream>

#include <boost/assign/list_of.hpp>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include "lpsensor/LpmsSensorI.h"
#include "lpsensor/LpmsSensorManagerI.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
  \TODO: Check axis orientation!
 */
class LpImuProxy
{
 public:
    LpImuProxy() : private_nh("~")
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
		
		// Get node parameters
		private_nh.param<std::string>("sensor_model", sensor_model, "DEVICE_LPMS_U2");
		private_nh.param<std::string>("port", port, "/dev/ttyUSB0");
        private_nh.param<std::string>("frame_id", frame_id, "imu");
		private_nh.param("rate", rate, 50);

		// Connect to the LP IMU device
        manager = LpmsSensorManagerFactory();
        imu = manager->addSensor(device_map[sensor_model], port.c_str());

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
			imu_msg.header.frame_id = frame_id;

            // Fill orientation quaternion
            imu_msg.orientation.w = data.q[0];
            imu_msg.orientation.x = -data.q[1];
            imu_msg.orientation.y = -data.q[2];
            imu_msg.orientation.z = -data.q[3];
			
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
			mag_msg.header.frame_id = frame_id;

			mag_msg.magnetic_field.x = data.b[0];
			mag_msg.magnetic_field.y = data.b[1];
			mag_msg.magnetic_field.z = data.b[2];

			// Publish the messages
			imu_pub.publish(imu_msg);
			mag_pub.publish(mag_msg);
        }
    }

    void quaternionToEuler(float *q, float *r)
    {
        // ZYX Rotation sequence
        const float r2d = 57.2958f;
        float w = q[0];
        float x = q[1];
        float y = q[2];
        float z = q[3];
        float r11 = 2 * (x*y + w*z);
        float r12 = w*w + x*x - y*y - z*z;
        float r21 = -2 * (x*z - w*y);
        float r31 = 2 * (y*z + w*x);
        float r32 = w*w - x*x - y*y + z*z;
        r[2] = (float)atan2(r11, r12) * r2d;
        r[1] = (float)asin(r21) * r2d;
        r[0] = (float)atan2(r31, r32) * r2d;
    }

    void eulerToQuaternion(float *r, float *q) {
        const float r2d = 57.2958f;
        float x = r[0] / r2d;
        float z = r[1] / r2d;
        float y = r[2] / r2d;
        float c1 = cos(z/2);
        float s1 = sin(z/2);
        float c2 = cos(y/2);
        float s2 = sin(y/2);
        float c3 = cos(x/2);
        float s3 = sin(x/2);
        q[0] =c1*c2*c3 - s1*s2*s3;
        q[1] =c1*c2*s3 + s1*s2*c3;
        q[2] =s1*c2*c3 + c1*s2*s3;
        q[3] =c1*s2*c3 - s1*c2*s3;
      }

	void run(void)
	{
		// The timer ensures periodic data publishing
		updateTimer = ros::Timer(nh.createTimer(ros::Duration(1.0/rate),
												&LpImuProxy::update,
												this));
	}

 private:

	// Access to LPMS data
    LpmsSensorManagerI* manager;
    LpmsSensorI* imu;
    ImuData data;
	std::map<std::string,int> device_map;

	// Access to ROS node
	ros::NodeHandle nh, private_nh;
	ros::Timer updateTimer;
	ros::Publisher imu_pub, mag_pub;
	sensor_msgs::Imu imu_msg;
	sensor_msgs::MagneticField mag_msg;

	// Parameters
	std::string sensor_model;
	std::string port;
	std::string frame_id;
	int rate;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_imu");
    ros::NodeHandle nh, private_nh;

    LpImuProxy lpImu;

	lpImu.run();

    ros::spin();

    return 0;
}
