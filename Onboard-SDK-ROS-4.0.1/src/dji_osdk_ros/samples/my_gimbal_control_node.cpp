/*
 * @Author: your name
 * @Date: 2020-11-28 15:27:17
 * @LastEditTime: 2020-11-28 19:07:02
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/Onboard-SDK-ROS-4.0.1/src/dji_osdk_ros/samples/my_gimbal_control_node.cpp
 */

#include <ros/ros.h>
#include <dji_osdk_ros/common_type.h>
#include <dji_osdk_ros/GimbalAction.h>
#include <dji_osdk_ros/CameraEV.h>
#include <dji_osdk_ros/CameraShutterSpeed.h>
#include <dji_osdk_ros/CameraAperture.h>
#include <dji_osdk_ros/CameraISO.h>
#include <dji_osdk_ros/CameraFocusPoint.h>
#include <dji_osdk_ros/CameraTapZoomPoint.h>
#include <dji_osdk_ros/CameraZoomCtrl.h>
#include <dji_osdk_ros/CameraStartShootBurstPhoto.h>
#include <dji_osdk_ros/CameraStartShootAEBPhoto.h>
#include <dji_osdk_ros/CameraStartShootSinglePhoto.h>
#include <dji_osdk_ros/CameraStartShootIntervalPhoto.h>
#include <dji_osdk_ros/CameraStopShootPhoto.h>
#include <dji_osdk_ros/CameraRecordVideoAction.h>

#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#define X5S_FOCAL_LENGTH    4493.23609 * 0.0173 / 5280.0 //(Camera Matrix F) * (Sensor Size) / (Pixel Size)
#define X5S_PIXEL_WIDTH     1280.0
#define X5S_PIXEL_HEIGHT    720.0
#define X5S_SENSOR_WIDTH    0.0173              /// sensor width [m]
#define X5S_SENSOR_HEIGHT   0.013               /// sensor height [m]
#define X5S_K1              -0.047904824        /// distortion matrix k1
#define X5S_K2              0.0393172992        /// distortion matrix k2
#define X5S_P1              0.0                 /// distortion matrix p1
#define X5S_P2              0.0                 /// distortion matrix p2

//CODE
using namespace dji_osdk_ros;

class gimbalControl
{
public:
    /// @brief constructed functinon.
    gimbalControl();
    gimbalControl(ros::NodeHandle nh);

    /// @brief destructor.
    ~gimbalControl();
    
    /// @brief init function of gimbal control.
    void init();

    /// @brief callback function of targets' bounding_boxes from darknet_ros.
    void bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg);

public:

    ros::NodeHandle nodeHandle_;    ///< ros::NodeHandle

    int tracking_target_id_;        ///< the target's id needs to be tracked.

    float u0_;    ///< camera intrinsic parameters: x-axis center in pixels
    float v0_;    ///< camera intrinsic parameters: y-axis center in pixels

    float zero_u_;      ///< define the x-axis "center" of image, stop gimbal control in this area. 
    float zero_v_;      ///< define the y-axis "center" of image, stop gimbal control in this area. 
};  


/**
 * @description: constructed function of gimbal control node
 * @param[in]   nh  ros::NodeHandle
 */
gimbalControl::gimbalControl(ros::NodeHandle nh): nodeHandle_(nh)
{
    ROS_INFO("gimbal control node started");

    tracking_target_id_ = 0;

    nodeHandle_.param("camera_u0", u0_, float(X5S_PIXEL_WIDTH/2.0));
    nodeHandle_.param("camera_v0", v0_, float(X5S_PIXEL_HEIGHT/2.0));
    nodeHandle_.param("camera_u_center", zero_u_, float(200.0));
    nodeHandle_.param("camera_v_center", zero_v_, float(100.0));
    
    init();
}

gimbalControl::~gimbalControl(){
    ROS_INFO("gimbal control node stoped");
}

void gimbalControl::init()
{
    ROS_INFO("gimbale control node init");

    /// Define dji_osdk_ros service for gimbal control
    auto gimbal_control_client = nodeHandle_.serviceClient<GimbalAction>("gimbal_task_control");

    /// Define darknet_ros bounding_box subscriber
    ros::Subscriber bbox_sub = nodeHandle_.subscribe("darknet_ros/bounding_boxes", 1, &bboxCallback);

}

void gimbalControl::bboxCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bbox_msg)
{   
    int target_num;             ///< number of ground targets.
    bool has_target = false;    ///< flag if target in camera sight
    float u_error = 0.0;        ///< error in pixel of x-axis
    float v_error = 0.0;        ///< error in pixel of y-axis

    target_num = bbox_msg->bounding_boxes.size();   ///< number of ground targets.
    
    /// find target's image position and calculate pixel errors.
    for(int i = 0; i < target_num; i++)
    {
        if(bbox_msg->bounding_boxes[i].id == tracking_target_id_)
        {
            float this_u = (bbox_msg->bounding_boxes[i].xmax + bbox_msg->bounding_boxes[i].xmin) / 2.0;     ///< pixel-center of target in x-axis
            float this_v = (bbox_msg->bounding_boxes[i].ymax + bbox_msg->bounding_boxes[i].ymin) / 2.0;     ///< pixel-center of target in y-axis

            u_error = u0_ - this_u;
            v_error = v0_ - this_v;

            has_target = true;
        }
    }

    /// if there is no target, exit this function.
    if(!has_target) return;

    /// if target's position in the center of the image.
    if(abs(u_error) < zero_u_)  u_error = 0;
    if(abs(v_error) < zero_v_)  v_error = 0;


}
