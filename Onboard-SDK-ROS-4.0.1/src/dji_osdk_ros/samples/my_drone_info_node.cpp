# include "my_drone_info_node.h"

#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

bool enableAttitudeOutput = 0;
bool enableGimbalOutput = 1;
bool enableGPSOutput = 0;

const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;

//-----------------  .CPP  ------------------------//

// global variables for subscribed topics
uint8_t flight_status = 255;
uint8_t display_mode  = 255;
sensor_msgs::NavSatFix current_gps;
geometry_msgs::Quaternion current_atti;
geometry_msgs::Vector3 currect_atti_euler;

geometry_msgs::Point current_local_pos;
geometry_msgs::Vector3 current_gimbal;
sensor_msgs::Imu current_imu;


int main(int argc, char** argv){
    ros::init(argc, argv, "my_drone_info_node");
    ros::NodeHandle nh;

    // Subscribe to messages from dji_sdk_node
    // Attitude Subscriber
    ros::Subscriber attitudeSub = nh.subscribe("dji_osdk_ros/attitude", 1, &attitude_callback);
    // Gimbal Subscriber
    ros::Subscriber gimbalSub = nh.subscribe("dji_osdk_ros/gimbal_angle",1, &gimbal_callback);

    ros::Subscriber gpsSub      = nh.subscribe("dji_osdk_ros/gps_position", 1, &gps_callback);
    //ros::Subscriber flightStatusSub = nh.subscribe("dji_osdk_ros/flight_status", 1, &flight_status_callback);
    //ros::Subscriber displayModeSub = nh.subscribe("dji_osdk_ros/display_mode", 1, &display_mode_callback);
    ros::Subscriber localPosition = nh.subscribe("dji_osdk_ros/local_position", 1, &local_position_callback);

    ros::spin();
    return 0;
}

/**
 * @description: Callback function for attitude_publisher @ OSDK Function getQuaternion()
 * @param geometry_msgs::QuaternionStamped msg @ ROS api
 * @return {type} 
 */
void attitude_callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg){
    current_atti = msg->quaternion;
    geometry_msgs::QuaternionStamped current_atti_stamped = *msg;

    currect_atti_euler = toEulerAngle(current_atti);
    if (enableAttitudeOutput) {
      printf("\033[2J");
      printf("\033[1;1H");

      printf("\nCurrent_atti: Roll_inRad_ %f\n", currect_atti_euler.x);
      printf("\nCurrent_atti: Pitch_inRad_ %f\n", currect_atti_euler.y);
      printf("\nCurrent_atti: Yaw_inRad_ %f\n", currect_atti_euler.z);

      printf("\nCurrent_atti: Roll_inDeg_ %f\n", currect_atti_euler.x * rad2deg);
      printf("\nCurrent_atti: Pitch_inDeg_ %f\n", currect_atti_euler.y * rad2deg);
      printf("\nCurrent_atti: Yaw_inDeg_ %f\n", currect_atti_euler.z * rad2deg);

  }
}

/**
 * @description: Callback function for gimbal_angle_publisher @ OSDK Function getGimbal()
 *              According to the Doc of OSDK, Data Accuracy: 0.1 deg in all axes
 * @param geometry_msgs::Vector3Stamped msg @ ROS api
 * @return {type} 
 */
void gimbal_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg){
    current_gimbal = msg->vector;
    geometry_msgs::Vector3Stamped currnet_gimbal_stamped = *msg;

    if (enableGimbalOutput) {
      printf("\033[2J");
      printf("\033[1;1H");

      printf("\nCurrent_gimbal: Roll_inRad %f\n", current_gimbal.x);
      printf("\nCurrent_gimbal: Pitch_inRad_ %f\n", current_gimbal.y);
      printf("\nCurrent_gimbal: Yaw_inRad_ %f\n", current_gimbal.z);

      printf("\nCurrent_gimbal: Roll_inDeg_ %f\n", current_gimbal.x * rad2deg);
      printf("\nCurrent_gimbal Pitch_inDeg_ %f\n", current_gimbal.y * rad2deg);
      printf("\nCurrent_gimbal: Yaw_inDeg_ %f\n", current_gimbal.z * rad2deg);

  }
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

  if (enableGPSOutput) {
      printf("\033[2J");
      printf("\033[1;1H");

      printf("\nCurrent_gps: Latitude_ %f\n", current_gps.latitude);
      printf("\nCurrent_gps: Longitude_ %f\n", current_gps.longitude);
      printf("\nCurrent_gps: Altitude_ %f\n", current_gps.altitude);

  }
  //std::cout << "Current_gps:time_ " << current_gps.header.stamp << std::endl;
  // std::cout << "Current_gps: Latitude_ " << current_gps.latitude << std::endl;
  // std::cout << "Current_gps: Longitude_ " << current_gps.longitude << std::endl;
  // std::cout << "Current_gps: Altitude_ " << current_gps.altitude << std::endl;
}

/**
 * @description: Callback function for imu_publisher @ Acceleration OSDK Function getAcceleration() Angular velocity OSDK Function getAngularRate()
 * @param sensor_msgs::Imu @ ROS api 
 * @return {type} 
 */
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  current_imu = *msg;
  std::cout << "Current_acceleration: x_ " << current_imu.linear_acceleration.x << std::endl;
  std::cout << "Current_acceleration: y_ " << current_imu.linear_acceleration.y << std::endl;
  std::cout << "Current_acceleration: z_ " << current_imu.linear_acceleration.z << std::endl;

  std::cout << "Current_angular_velocity: x_ " << current_imu.angular_velocity.x << std::endl;
  std::cout << "Current_angular_velocity: y_ " << current_imu.angular_velocity.y << std::endl;
  std::cout << "Current_angular_velocity: z_ " << current_imu.angular_velocity.z << std::endl;
}

void flight_status_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_status = msg->data;
}

void display_mode_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  display_mode = msg->data;
}

void local_position_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
  current_local_pos = msg->point;
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