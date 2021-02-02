/*
 * @Author: your name
 * @Date: 2020-08-30 15:46:14
 * @LastEditTime: 2020-12-09 19:17:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\samples\my_drone_info_sync.cpp
 */

#include "my_offline_info_sync_node.h"
#include <dji_osdk_ros/MyDroneState.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <dji_osdk_ros/DroneState.h>
#include <dji_osdk_ros/ImagePosition.h>

// -------------- DEFINE CAMERA PARAMETERS ------------- //
// pixel_size = [5280, 2970]
// camera_matrix = [4.4932360907364246e+03,  0.0, 2640.0,
//                   0.0, 4.4932360907364246e+03, 1485.0, 
//                   0.0, 0.0, 1.0]
// distortion_matrix = [ 4.4932360907364246e+03, 0.0, 2640.0, 0.0, 4.4932360907364246e+03 1485.0, 0.0, 0.0, 1.0]

#define X5S_FOCAL_LENGTH  4493.23609 * 0.0173 / 5280 //(Camera Matrix F) * (Sensor Size) / (Pixel Size)
#define X5S_PIXEL_WIDTH  1280
#define X5S_PIXEL_HEIGHT  720
#define X5S_SENSOR_WIDTH  0.0173 // [m]
#define X5S_SENSOR_HEIGHT  0.013 // [m]

#define MASTER_FLAG 0
#define SLAVER_FLAG 1

using namespace dji_osdk_ros;

ros::Publisher master_states_publisher;
ros::Publisher slaver_states_publisher;

ros::Publisher master_states_2_publisher;
ros::Publisher slaver_states_2_publisher;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;


int main(int argc, char** argv){
    ros::init(argc, argv, "my_offline_info_sync_node");
    ros::NodeHandle nh;

    //----------------- Define Subscriber --------------------//

    /// master GPS subscriber
    //message_filters::Subscriber<sensor_msgs::NavSatFix> master_gps_sub(nh, "/master/vehicle_node/dji_osdk_ros/gps_position", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> master_gps_sub(nh, "/master/dgps_node/dgps_fix_position", 1);
    /// master attitude subscriber
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> master_attitude_sub(nh, "/master/vehicle_node/dji_osdk_ros/attitude", 1);
    /// master gimbal subscriber
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> master_gimbal_sub(nh, "/master/vehicle_node/dji_osdk_ros/gimbal_angle", 1);
    /// master bounding box subscriber
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> master_bbox_sub(nh, "/master/darknet_ros/bounding_boxes", 1);

    /// slaver GPS subscriber
    //message_filters::Subscriber<sensor_msgs::NavSatFix> slaver_gps_sub(nh, "/slaver1/vehicle_node/dji_osdk_ros/gps_position", 1);
    message_filters::Subscriber<sensor_msgs::NavSatFix> slaver_gps_sub(nh, "/slaver1/dgps_node/dgps_fix_position", 1);
    /// slaver attitude subscriber
    message_filters::Subscriber<geometry_msgs::QuaternionStamped> slaver_attitude_sub(nh, "/slaver1/vehicle_node/dji_osdk_ros/attitude", 1);
    /// slaver gimbal subscriber
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> slaver_gimbal_sub(nh, "/slaver1/vehicle_node/dji_osdk_ros/gimbal_angle", 1);
    /// slaver bounding box subscriber
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> slaver_bbox_sub(nh, "/slaver1/darknet_ros/bounding_boxes", 1);


    //Synchronize camera and attitube info using @ROS TimeSynchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, 
                                                            geometry_msgs::QuaternionStamped, 
                                                            geometry_msgs::Vector3Stamped, 
                                                            darknet_ros_msgs::BoundingBoxes> topic_sync_policy;

    //Subscribe main camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/main_camera"
    //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/main_camera_images", 1, &camera_callback);
    //Subscribe fpv camera @ DJI_ROS_OSDK_TOPIC "dji_sdk/fpv_camera_images"
    //ros::Subscriber mainCameraSub = nh.subscribe("dji_sdk/fpv_camera_images", 1, &camera_callback);

    // approximateTime takes a queue size as its constructor argument, in @M210_OSDK topic_sync_policy(500)
    message_filters::Synchronizer<topic_sync_policy> master_topic_synchronizer(topic_sync_policy(500), 
                                                          master_gps_sub, master_attitude_sub, master_gimbal_sub, master_bbox_sub);

    // approximateTime takes a queue size as its constructor argument, in @M210_OSDK topic_sync_policy(500)
    message_filters::Synchronizer<topic_sync_policy> slaver_topic_synchronizer(topic_sync_policy(500), 
                                                          slaver_gps_sub, slaver_attitude_sub, slaver_gimbal_sub, slaver_bbox_sub);
       
    //------------- Subscriber callback ---------------//
    master_topic_synchronizer.registerCallback(boost::bind(&M210StatesCallback, _1, _2, _3, _4, MASTER_FLAG));

    slaver_topic_synchronizer.registerCallback(boost::bind(&M210StatesCallback, _1, _2, _3, _4, SLAVER_FLAG));

    //------------- Define MyDroneState.msg publisher -------------//
    master_states_publisher = nh.advertise<dji_osdk_ros::MyDroneState>("/master/info_node/drone_state", 10);
    slaver_states_publisher = nh.advertise<dji_osdk_ros::MyDroneState>("/slaver1/info_node/drone_state", 10);

    master_states_2_publisher = nh.advertise<dji_osdk_ros::DroneState>("/master/info_node/drone_state_2", 10);
    slaver_states_2_publisher = nh.advertise<dji_osdk_ros::DroneState>("/slaver1/info_node/drone_state_2", 10);
    
    ros::spin();
    return 0;
}

void M210StatesCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                        const geometry_msgs::QuaternionStampedConstPtr &attitude_msg,
                        const geometry_msgs::Vector3Stamped::ConstPtr &gimbal_msg,
                        const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg,
                        const int flag)
{
    sensor_msgs::NavSatFix current_gps;
    geometry_msgs::Quaternion current_atti;
    geometry_msgs::Vector3 currect_atti_euler;
    geometry_msgs::Point current_local_pos;
    geometry_msgs::Vector3 current_gimbal;
    sensor_msgs::Imu current_imu;

    darknet_ros_msgs::BoundingBoxes current_bbox;

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


    // //-------------- Publish MyDroneState.msg publisher -------------//
    // dji_osdk_ros::MyDroneState this_drone_state;
    // //this_drone_state.header.stamp = current_gimbal_stamped.header.stamp;
    // this_drone_state.header.stamp = ros::Time::now();
    // this_drone_state.header.frame_id = "my_drone_state";
    // this_drone_state.gps = current_gps;
    // this_drone_state.atti = currect_atti_euler;
    // this_drone_state.gimbal = current_gimbal;

    // // ------------- Image Position Calculator ------------- //  
    // /// image position of target's bounding box center.
    // double pixel_x = (current_bbox.bounding_boxes[0].xmin + current_bbox.bounding_boxes[0].xmax) / 2;
    // double pixel_y = (current_bbox.bounding_boxes[0].ymin + current_bbox.bounding_boxes[0].ymax) / 2;

    // /// target's bounding box center position in camera sensor.
    // double sensor_x = ((X5S_PIXEL_WIDTH / 2) - pixel_x) * X5S_SENSOR_WIDTH / X5S_PIXEL_WIDTH;
    // double sensor_y = ((X5S_PIXEL_HEIGHT / 2) - pixel_y) * X5S_SENSOR_HEIGHT / X5S_PIXEL_HEIGHT;

    // /// define publish.
    // // this_drone_state.image_position.x = sensor_x;
    // // this_drone_state.image_position.y = sensor_y;
    // this_drone_state.image_position.x = pixel_x;
    // this_drone_state.image_position.y = pixel_y;

    // this_drone_state.focal_length = X5S_FOCAL_LENGTH;


    //-------------- Publish DroneState.msg publishier --------------//
    dji_osdk_ros::DroneState this_drone_state_2;
    //this_drone_state_2.header.stamp = current_gimbal_stamped.header.stamp;
    this_drone_state_2.header.stamp = ros::Time::now();
    this_drone_state_2.header.frame_id = "drone_state";

    this_drone_state_2.gps = current_gps;
    this_drone_state_2.atti = currect_atti_euler;
    this_drone_state_2.gimbal = current_gimbal;

    for(int i = 0; i < current_bbox.bounding_boxes.size(); i++)
    {
      dji_osdk_ros::ImagePosition this_image_position;
      this_image_position.Class = current_bbox.bounding_boxes[i].Class;
      this_image_position.id = current_bbox.bounding_boxes[i].id;
      this_image_position.probability = current_bbox.bounding_boxes[i].probability;
      this_image_position.xmax = current_bbox.bounding_boxes[i].xmax;
      this_image_position.xmin = current_bbox.bounding_boxes[i].xmin;
      this_image_position.ymax = current_bbox.bounding_boxes[i].ymax;
      this_image_position.ymin = current_bbox.bounding_boxes[i].ymin;

      this_drone_state_2.image_positions.push_back(this_image_position);
    }

    if(flag == MASTER_FLAG){
      //master_states_publisher.publish(this_drone_state);
      master_states_2_publisher.publish(this_drone_state_2);

    }else{
      //slaver_states_publisher.publish(this_drone_state);
      slaver_states_2_publisher.publish(this_drone_state_2);
    }

    
}

/**
 * @description: Quaterion to EulerAngle @Onboard-SDK-ROS-3.8 demo_flight_control.cpp
 * @param [in]    quat      source quaterion.
 * @return [out]  ans       euler angle after translation.
 */
geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat){
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}
