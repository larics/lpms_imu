# ros_lpms_imu

A ROS driver for [LP-RESEARCH IMU sensors](https://www.lp-research.com/products/).

## Installation

Currently, the driver needs to be installed from source. Clone the [timesync package](https://github.com/larics/timesync_ros) (a utility for providing more accurate timestamps for ROS messages) and the driver into the `src` subfolder of your [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) and run `catkin_make`:

```
roscd
cd ../src
git clone https://github.com/larics/timesync_ros.git
git clone https://github.com/larics/lpms_imu.git
cd ..
catkin_make
```

## Using the driver

Assuming you have `roscore` running, just rosrun the driver:

```
rosrun lpms_imu lpms_imu_node
```

The driver assumes your sensor is connected to the `/dev/ttyUSB0` port. If this is not the case, use the `_port` parameter, e.g.
```
rosrun lpms_imu lpms_imu_node _port:=/dev/ttyUSB1
```

## More info

For more information check out the [Wiki page](http://wiki.ros.org/lpms_imu).

## References

[REP 145](https://github.com/paulbovbel/rep/blob/master/rep-0145.rst): Conventions for IMU Sensor Drivers