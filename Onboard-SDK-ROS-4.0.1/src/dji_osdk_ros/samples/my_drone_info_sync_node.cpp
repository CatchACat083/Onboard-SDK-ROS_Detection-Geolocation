/*
 * @Author: your name
 * @Date: 2020-08-30 15:46:14
 * @LastEditTime: 2020-11-16 14:40:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\samples\my_drone_info_sync.cpp
 */

#include "my_drone_info_sync_node.h"
#include <dji_osdk_ros/MyDroneState.h>
#include <dji_osdk_ros/DroneState.h>
#include <dji_osdk_ros/ImagePosition.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// -------------- DEFINE CAMERA PARAMETERS ------------- //
// pixel_size = [5280, 2970]
// camera_matrix = [4.4932360907364246e+03,  0.0, 2640.0,
//                   0.0, 4.4932360907364246e+03, 1485.0, 
//                   0.0, 0.0, 1.0]
// distortion_matrix = [ 4.4932360907364246e+03, 0.0, 2640.0, 0.0, 4.4932360907364246e+03 1485.0, 0.0, 0.0, 1.0]
// -4.7904824257196547e-02 3.9317299223372695e-02 0. 0.
//    3.1901403501810498e-02

#define X5S_FOCAL_LENGTH  4493.23609 * 0.0173 / 5280 //(Camera Matrix F) * (Sensor Size) / (Pixel Size)
#define X5S_PIXEL_WIDTH  1280
#define X5S_PIXEL_HEIGHT  720
#define X5S_SENSOR_WIDTH  0.0173 // [m]
#define X5S_SENSOR_HEIGHT  0.013 // [m]

using namespace dji_osdk_ros;

sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 currect_atti_euler;
geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3 current_gimbal;
sensor_msgs::Imu current_imu;

darknet_ros_msgs::BoundingBoxes current_bbox;

ros::Publisher drone_states_publisher;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

// int main(int argc, char** argv){
//     ros::init(argc, argv, "my_drone_info_sync_node");
//     ros::NodeHandle nh;

//     //----------------- Define Subscriber --------------------//

//     //GPS subscriber
//     message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "dji_osdk_ros/gps_position", 1);
//     //Attitude subscriber
//     message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(nh, "dji_osdk_ros/attitude", 1);
//     //Gimbal subscriber
//     message_filters::Subscriber<geometry_msgs::Vector3Stamped> gimbal_sub(nh, "dji_osdk_ros/gimbal_angle", 1);

//     //Synchronize camera and attitube info using @ROS TimeSynchronizer
//     typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, geometry_msgs::QuaternionStamped, geometry_msgs::Vector3Stamped> topic_sync_policy;

//     //Subscribe main camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/main_camera"
//     //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/main_camera_images", 1, &camera_callback);
//     //Subscribe fpv camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/fpv_camera_images"
//     //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/fpv_camera_images", 1, &camera_callback);
//     message_filters::Synchronizer<topic_sync_policy> topic_synchronizer(topic_sync_policy(10), gps_sub, attitude_sub, gimbal_sub);
       
//     //------------- Subscriber callback ---------------//
//     topic_synchronizer.registerCallback(boost::bind(&M210StatesCallback, _1, _2, _3));

//     //------------- Define MyDroneState.msg publisher -------------//
//     drone_states_publisher = nh.advertise<dji_osdk_ros::MyDroneState>("dji_osdk_ros/my_drone_state",100);
    
//     ros::spin();
//     return 0;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "my_drone_info_sync_node");
    ros::NodeHandle nh;

    //----------------- Define Subscriber --------------------//

    //GPS subscriber
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub(nh, "vehicle_node/dji_osdk_ros/gps_position", 1);
    //Attitude subscriber
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> attitude_sub(nh, "vehicle_node/dji_osdk_ros/attitude", 1);
    //Gimbal subscriber
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> gimbal_sub(nh, "vehicle_node/dji_osdk_ros/gimbal_angle", 1);

    //Bounding Box subscriber
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbox_sub(nh, "darknet_ros/bounding_boxes", 1);

    //Synchronize camera and attitube info using @ROS TimeSynchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, 
                                                            geometry_msgs::QuaternionStamped, 
                                                            geometry_msgs::Vector3Stamped, 
                                                            darknet_ros_msgs::BoundingBoxes> topic_sync_policy;

    //Subscribe main camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/main_camera"
    //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/main_camera_images", 1, &camera_callback);
    //Subscribe fpv camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/fpv_camera_images"
    //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/fpv_camera_images", 1, &camera_callback);

    // ApproximateTime takes a queue size as its constructor argument, in @M210_OSDK topic_sync_policy(500)
    message_filters::Synchronizer<topic_sync_policy> topic_synchronizer(topic_sync_policy(500), gps_sub, attitude_sub, gimbal_sub, bbox_sub);
       
    //------------- Subscriber callback ---------------//
    topic_synchronizer.registerCallback(boost::bind(&M210StatesCallback, _1, _2, _3, _4));

    //------------- Define MyDroneState.msg publisher -------------//
    drone_states_publisher = nh.advertise<dji_osdk_ros::DroneState>("info_node/drone_state",10);
    
    ros::spin();
    return 0;
}

void M210StatesCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                        const geometry_msgs::QuaternionStampedConstPtr &attitude_msg,
                        const geometry_msgs::Vector3Stamped::ConstPtr &gimbal_msg,
                        const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg){

    current_gimbal = gimbal_msg->vector;
    
    std::cout << "Current_gimbal: roll_ " << current_gimbal.x <<std::endl;
    std::cout << "Current_gimbal: pitch_ " << current_gimbal.y <<std::endl;
    std::cout << "Current_gimbal: yaw_ " << current_gimbal.z <<std::endl;

    current_atti = attitude_msg->quaternion;
    geometry_msgs::QuaternionStamped current_gimbal_stamped = *attitude_msg;
    std::cout << "Current_atti: w_ " << current_atti.w << std::endl;
    std::cout << "Current_atti: x_ " << current_atti.x << std::endl;
    std::cout << "Current_atti: y_ " << current_atti.y << std::endl;
    std::cout << "Current_atti: z_ " << current_atti.z << std::endl;

    currect_atti_euler = toEulerAngle(current_atti);
    std::cout << "Current_atti: Roll_inRad_ " << currect_atti_euler.x << std::endl;
    std::cout << "Current_atti: Pitch_inRad_ " << currect_atti_euler.y << std::endl;
    std::cout << "Current_atti: Yaw_inRad_ " << currect_atti_euler.z << std::endl;
    std::cout << "Current_atti: Roll_inDeg_ " << currect_atti_euler.x * rad2deg << std::endl;
    std::cout << "Current_atti: Pitch_inDeg_ " << currect_atti_euler.y * rad2deg << std::endl;
    std::cout << "Current_atti: Yaw_inDeg_ " << currect_atti_euler.z * rad2deg << std::endl;

    current_gps = *gps_msg;
    std::cout << "Current_gps: Latitude_ " << current_gps.latitude << std::endl;
    std::cout << "Current_gps: Longitude_ " << current_gps.longitude << std::endl;
    std::cout << "Current_gps: Altitude_ " << current_gps.altitude << std::endl;

    current_bbox = *bbox_msg;
    std::cout << "Current_bbox: image_x " << (current_bbox.bounding_boxes[0].xmin + current_bbox.bounding_boxes[0].xmax) / 2 << std::endl;
    std::cout << "Current_bbox: image_y " << (current_bbox.bounding_boxes[0].ymin + current_bbox.bounding_boxes[0].ymax) / 2 << std::endl;


    //-------------- Publish MyDroneState.msg publisher -------------//
    dji_osdk_ros::DroneState this_drone_state;
    this_drone_state.header = current_gimbal_stamped.header;
    this_drone_state.header.frame_id = "drone_state";
    this_drone_state.gps = current_gps;
    this_drone_state.atti = currect_atti_euler;
    this_drone_state.gimbal = current_gimbal;

    // ------------- Image Position Calculator ------------- //
    dji_osdk_ros::ImagePosition image_position;
    for(int i = 0; i < current_bbox.bounding_boxes.size(); i++){
      image_position.probability = current_bbox.bounding_boxes[i].probability;
      image_position.xmin = current_bbox.bounding_boxes[i].xmin;
      image_position.ymin = current_bbox.bounding_boxes[i].ymin;
      image_position.xmax = current_bbox.bounding_boxes[i].xmax;
      image_position.ymax = current_bbox.bounding_boxes[i].ymax;
      image_position.id = current_bbox.bounding_boxes[i].id;
      image_position.Class = current_bbox.bounding_boxes[i].Class;

      this_drone_state.image_positions.push_back(image_position);
    }

    /// image position of target's bounding box center.
    double pixel_x = (current_bbox.bounding_boxes[0].xmin + current_bbox.bounding_boxes[0].xmax) / 2;
    double pixel_y = (current_bbox.bounding_boxes[0].ymin + current_bbox.bounding_boxes[0].ymax) / 2;

    /// target's bounding box center position in camera sensor.
    //double sensor_x = ((X5S_PIXEL_WIDTH / 2) - pixel_x) * X5S_SENSOR_WIDTH / X5S_PIXEL_WIDTH;
    //double sensor_y = ((X5S_PIXEL_HEIGHT / 2) - pixel_y) * X5S_SENSOR_HEIGHT / X5S_PIXEL_HEIGHT;

    //this_drone_state.image_position.x = sensor_x;
    //this_drone_state.image_position.y = sensor_y;

    //this_drone_state.image_position.x = pixel_x;
    //this_drone_state.image_position.y = pixel_y;

    //this_drone_state.focal_length = X5S_FOCAL_LENGTH;

    drone_states_publisher.publish(this_drone_state);
}

/**
 * @description: Quaterion to EulerAngle @Onboard-SDK-ROS-3.8 demo_flight_control.cpp
 * @param {type} 
 * @return {type} 
 */
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat){
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}

/**
 * @description: Callback function for attitude_publisher @ OSDK Function getQuaternion()
 * @param geometry_msgs::QuaternionStamped msg @ ROS api
 * @return {type} 
 */
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
    current_atti = msg->quaternion;
    // std::cout << "Current_atti: w_ " << current_atti.w << std::endl;
    // std::cout << "Current_atti: x_ " << current_atti.x << std::endl;
    // std::cout << "Current_atti: y_ " << current_atti.y << std::endl;
    // std::cout << "Current_atti: z_ " << current_atti.z << std::endl;
}

/**
 * @description: Callback function for gimbal_angle_publisher @ OSDK Function getGimbal()
 *              According to the Doc of OSDK, Data Accuracy: 0.1 deg in all axes
 * @param geometry_msgs::Vector3Stamped msg @ ROS api
 * @return {type} 
 */
void gimbal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    current_gimbal = msg->vector;
    std::cout << "Current_gimbal: roll_ " << current_gimbal.x <<std::endl;
    std::cout << "Current_gimbal: pitch_ " << current_gimbal.y <<std::endl;
    std::cout << "Current_gimbal: yaw_ " << current_gimbal.z <<std::endl;
}

/**
 * @description: Callback function for gps_position_publisher @ OSDK Function getGlobalPosition()\
 *              According to the Doc of OSDK, the error <3m with open sky without multipath
 * @param sensor_msgs::NavSatFix msg @ ROS api
 * @return {type} 
 */
void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  current_gps = *msg;
  // std::cout << "Current_gps: Latitude_ " << current_gps.latitude << std::endl;
  // std::cout << "Current_gps: Longitude_ " << current_gps.longitude << std::endl;
  // std::cout << "Current_gps: Altitude_ " << current_gps.altitude << std::endl;
}