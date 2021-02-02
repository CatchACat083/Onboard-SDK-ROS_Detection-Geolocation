/*
 * @Author: your name
 * @Date: 2020-12-15 15:13:38
 * @LastEditTime: 2020-12-18 11:31:55
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/single_drone_info_sync_node.cpp
 */
#include "single_drone_info_sync_node.h"


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


const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;


singleDroneInfoSync::singleDroneInfoSync(ros::NodeHandle nh)
    : nodeHandle_(nh)
{
    ROS_INFO("single_drone_info_sync_node start.");

    /// publish topic name initilzation.
    nodeHandle_.param<std::string>("pub_drone_state_ros_topic", pub_drone_state_topic_, "info_node/drone_state");
    /// subscriber topic name initilzation.
    nodeHandle_.param<std::string>("sub_gps_ros_topic", sub_gps_ros_topic_, "dgps_node/dgps_fix_position");
    nodeHandle_.param<std::string>("sub_attitude_ros_topic", sub_attitude_ros_topic_, "vehicle_node/dji_osdk_ros/attitude");
    nodeHandle_.param<std::string>("sub_gimbal_ros_topic", sub_gimbal_ros_topic_, "vehicle_node/dji_osdk_ros/gimbal_angle");
    nodeHandle_.param<std::string>("sub_bbox_ros_topic", sub_bbox_ros_topic_, "darknet_ros/bounding_boxes");

    /// init singleDroneInfoSync and start sub & pub
    init();

}

singleDroneInfoSync::~singleDroneInfoSync()
{
    
}
/**
 * @description: initilzation of ros_topic subscriber and publisher. synchronize camera, attitube, gps position
 *               and darknet bounding boxes information when subscribing drone informations.
 */
void singleDroneInfoSync::init()
{
    ROS_INFO("single_drone_info_sync_node init.");

    /// define subscriber
    message_filters::Subscriber<sensor_msgs::NavSatFix>* gps_sub;
    gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nodeHandle_, sub_gps_ros_topic_, 1);                        ///< gps position subscriber
    message_filters::Subscriber<geometry_msgs::QuaternionStamped>* attitude_sub;
    attitude_sub = new message_filters::Subscriber<geometry_msgs::QuaternionStamped>(nodeHandle_, sub_attitude_ros_topic_, 1);     ///< attitude subscriber
    message_filters::Subscriber<geometry_msgs::Vector3Stamped>* gimbal_sub;
    gimbal_sub = new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(nodeHandle_, sub_gimbal_ros_topic_, 1);           /// gimbal subscriber
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>* bbox_sub;
    bbox_sub = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(nodeHandle_, sub_bbox_ros_topic_, 1);             ///< bounding box subscriber

    ROS_INFO("subscribe to gimbal ros_topic: %s", sub_gimbal_ros_topic_.c_str());
    ROS_INFO("subscribe to attitude ros_topic: %s", sub_attitude_ros_topic_.c_str());
    ROS_INFO("subscribe to gps ros_topic: %s", sub_gps_ros_topic_.c_str());
    ROS_INFO("subscribe to bbox ros_topic: %s", sub_bbox_ros_topic_.c_str());


    //Synchronize camera and attitube info using @ROS TimeSynchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, 
                                                            geometry_msgs::QuaternionStamped, 
                                                            geometry_msgs::Vector3Stamped, 
                                                            darknet_ros_msgs::BoundingBoxes> topic_sync_policy;

    // approximateTime takes a queue size as its constructor argument, in @M210_OSDK topic_sync_policy(500)
    message_filters::Synchronizer<topic_sync_policy>* topic_synchronizer;
    topic_synchronizer = new message_filters::Synchronizer<topic_sync_policy> (topic_sync_policy(1000), *gps_sub, *attitude_sub, *gimbal_sub, *bbox_sub);
       
    /// subscriber callback
    topic_synchronizer->registerCallback(boost::bind(&singleDroneInfoSync::singleDroneInfoCallback, this, _1, _2, _3, _4));

    /// define DroneState.msg publisher
    drone_states_publisher = nodeHandle_.advertise<drone_pose_estimation::DroneState>(pub_drone_state_topic_, 10);

    ROS_INFO("publish drone_state ros_topic: %s", pub_drone_state_topic_.c_str());
}

/**
 * @description: drone info callback. subscribe gps position, attitude position, gimbal position and bbox frome drone.
 *               publish one drone_state ros_message with all these information in time sync.
 * @param       &gps_msg            gps position 
 * @param       &attitude_msg       attitude of drone in quaterntion
 * @param       &gimbal_msg         gimbal of drone in vector3
 * @param       &bbox_msg           bounding boxes frome darknet_ros
 */
void singleDroneInfoSync::singleDroneInfoCallback(const sensor_msgs::NavSatFix::ConstPtr &gps_msg,
                                                  const geometry_msgs::QuaternionStampedConstPtr &attitude_msg,
                                                  const geometry_msgs::Vector3Stamped::ConstPtr &gimbal_msg,
                                                  const darknet_ros_msgs::BoundingBoxes::ConstPtr &bbox_msg)
{
    std::cout << ">>> Get one frame of drone information" << std::endl;
    sensor_msgs::NavSatFix current_gps;
    geometry_msgs::Quaternion current_atti;
    geometry_msgs::Vector3 currect_atti_euler;
    geometry_msgs::Point current_local_pos;
    geometry_msgs::Vector3 current_gimbal;

    darknet_ros_msgs::BoundingBoxes current_bbox;
    
    /// define gimbal information.
    current_gimbal = gimbal_msg->vector;
    std::cout << "  Current_gimbal: roll_ " << current_gimbal.x <<
                                    " pitch_ " << current_gimbal.y <<
                                    " yaw_ " << current_gimbal.z <<std::endl;

    /// define attitude information.
    current_atti = attitude_msg->quaternion;
    geometry_msgs::QuaternionStamped current_atti_stamped = *attitude_msg;
    std::cout << "  Current_atti: w_ " << current_atti.w << 
                                " x_ " << current_atti.x <<
                                " y_ " << current_atti.y << 
                                " z_ " << current_atti.z << std::endl;
    
    /// define attitude information in euler angle.
    currect_atti_euler = toEulerAngle(current_atti);
    std::cout << "  Current_atti: Roll_inRad_ " << currect_atti_euler.x << 
                                " Pitch_inRad_ " << currect_atti_euler.y << 
                                " Yaw_inRad_ " << currect_atti_euler.z << std::endl;

    std::cout << "  Current_atti: Roll_inDeg_ " << currect_atti_euler.x * rad2deg <<
                              " Pitch_inDeg_ " << currect_atti_euler.y * rad2deg <<
                              " Yaw_inDeg_ " << currect_atti_euler.z * rad2deg << std::endl;

    /// define gps global position 
    current_gps = *gps_msg;
    std::cout << "  Current_gps: Latitude_ " << current_gps.latitude <<
                                " Longitude_ " << current_gps.longitude << 
                                " Altitude_ " << current_gps.altitude << std::endl;

    /// define bounding box
    current_bbox = *bbox_msg;
    std::cout << "  Current_first_bbox: image_x " << (current_bbox.bounding_boxes[0].xmin + current_bbox.bounding_boxes[0].xmax) / 2 << 
                                        " image_y " << (current_bbox.bounding_boxes[0].ymin + current_bbox.bounding_boxes[0].ymax) / 2 << std::endl;


    /// publish DroneState.msg publisher 
    drone_pose_estimation::DroneState this_drone_state;
    //this_drone_state.header = current_gimbal_stamped.header;
    //this_drone_state.header.stamp = ros::Time::now();
    this_drone_state.header.stamp = gimbal_msg->header.stamp;
    this_drone_state.header.frame_id = "drone_state";
    this_drone_state.gps = current_gps;
    this_drone_state.atti = currect_atti_euler;
    this_drone_state.gimbal = current_gimbal;

    /// image position calculator ------------- //
    drone_pose_estimation::ImagePosition image_position;
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

    drone_states_publisher.publish(this_drone_state);
}

/**
 * @description: Quaterion to EulerAngle @Onboard-SDK-ROS-3.8 demo_flight_control.cpp
 * @param {type} 
 * @return {type} 
 */
geometry_msgs::Vector3 singleDroneInfoSync::toEulerAngle(geometry_msgs::Quaternion quat){
  geometry_msgs::Vector3 ans;

  tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
  R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
  return ans;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "my_drone_info_sync_node");
    ros::NodeHandle nh("~");

    singleDroneInfoSync single_drone_info_sync(nh);
    
    ros::spin();
    return 0;
}
