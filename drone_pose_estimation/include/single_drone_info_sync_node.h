/*
 * @Author: your name
 * @Date: 2020-12-15 15:17:25
 * @LastEditTime: 2020-12-17 13:44:51
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/include/single_drone_info_sync_node.h
 */
#include "chrono"
#include <signal.h>

// DJI SDK includes
// #include <dji_osdk_ros/common_type.h>
// #include <dji_osdk_ros/dji_vehicle_node.h>

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

// #include <dji_osdk_ros/MyDroneState.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <drone_pose_estimation/DroneState.h>
#include <drone_pose_estimation/ImagePosition.h>

//darknet includes
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <tf/tf.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

using namespace ros;
using namespace std;

class singleDroneInfoSync{
public:
    singleDroneInfoSync(ros::NodeHandle nh);
    ~singleDroneInfoSync();

private:
    void init();

    void singleDroneInfoCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                                  const geometry_msgs::QuaternionStampedConstPtr &attitude_msg,
                                                  const geometry_msgs::Vector3Stamped::ConstPtr &gimbal_msg,
                                                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg);

    geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat);

private:
    ros::NodeHandle nodeHandle_;    ///< ros nodeHandle.

    std::string pub_drone_state_topic_;   ///< output topic name of drone state.
    std::string sub_gps_ros_topic_;
    std::string sub_attitude_ros_topic_;
    std::string sub_gimbal_ros_topic_;
    std::string sub_bbox_ros_topic_;

    ros::Publisher drone_states_publisher;

};
