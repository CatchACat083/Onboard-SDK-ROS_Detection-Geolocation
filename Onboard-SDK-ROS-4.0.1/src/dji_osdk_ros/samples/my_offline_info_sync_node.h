/*
 * @Author: your name
 * @Date: 2020-08-30 15:46:29
 * @LastEditTime: 2020-11-11 20:08:50
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\samples\my_drone_info_sync.h
 */
// System includes
#include "chrono"
#include <signal.h>

// DJI SDK includes
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/dji_vehicle_node.h>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//darknet includes
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <tf/tf.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

void M210StatesCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                        const geometry_msgs::QuaternionStampedConstPtr &attitude_msg,
                        const geometry_msgs::Vector3Stamped::ConstPtr &gimbal_msg,
                        const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg,
                        const int flag);
                        
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);
void gimbal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

