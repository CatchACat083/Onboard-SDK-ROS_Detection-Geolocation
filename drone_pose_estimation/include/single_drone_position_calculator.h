/*
 * @Author: your name
 * @Date: 2020-12-02 11:16:40
 * @LastEditTime: 2020-12-18 11:40:49
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/include/single_drone_position_calculator.h
 */
// System includes
#include <iostream>
#include "chrono"
#include <signal.h>
#include <fstream>
#include <algorithm>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Matrix include
#include <Eigen/Dense>
#include <math.h>

#include <sys/time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <drone_pose_estimation/GroundTargetState.h>
#include <drone_pose_estimation/GroundTargetStates.h>
#include <drone_pose_estimation/DroneState.h>



#include <image_target.h>
#include <geodetic_target.h>
#include <gps_kalman_filter_2d.h>




#define pi (double)3.141592653589793


#define X5S_FOCAL_LENGTH    4493.23609 * 0.0173 / 5280.0 //(Camera Matrix F) * (Sensor Size) / (Pixel Size)
#define X5S_PIXEL_WIDTH     1280.0
#define X5S_PIXEL_HEIGHT    720.0
#define X5S_SENSOR_WIDTH    0.0173              /// sensor width [m]
#define X5S_SENSOR_HEIGHT   0.013               /// sensor height [m]
#define X5S_K1              -0.047904824        /// distortion matrix k1
#define X5S_K2              0.0393172992        /// distortion matrix k2
#define X5S_P1              0.0                 /// distortion matrix p1
#define X5S_P2              0.0                 /// distortion matrix p2

#define MAX_TARGETS_NUM         5                   ///< max number of ground targets in this experimation.

using namespace ros;
using namespace Eigen;
using namespace cv;
using namespace std;

const float deg2rad = pi/180.0;
const float rad2deg = 180.0/pi;

/**
 * @description: 
 *      Local NED (World) coordinate frame: local NED coordinate frame, whose orgin is the first position of UAV subscribed from GPS topic
 *      Drone coordinate frame:             X-axis towards the front, Z-axis down and Y-axis right accordingly. Origin is the center of the airborne GPS antenna.
 *      Drone NED coordinatre frame:        X-axis towards North, Y-axis East and Z-axis down. Origin is the center of the airborne GPS antenna.
 *      Camera coordinate frame:            standardized camera coodrinate frame.
 *      Camera NED coordinate frame:        X-axis towards North, Y-axis East and Z-axis down. Origin is on the optical center of camera.
 *      Car coordinate frame:               X-axis points the frone of car, Z-axis to the geocentrc. Origin is the center of car roof.
 *      Car GPS coordinate frame:           X-axis points the frone of car, Z-axis to the geocentrc. Origin id the center of car-GPS antenna.
 */
class singleDronePositionCalculator{
    public:
        singleDronePositionCalculator(ros::NodeHandle nh);
        ~singleDronePositionCalculator();

    public:
        #define ROTATE_XYZ  1   ///< rotation order:X, Y, Z
        #define ROTATE_ZYX  0   ///< rotation order:Z, Y, X

        #define METHOD_EKF  2   ///< EKF based pose estimation method
        #define METHOD_UV   1   ///< uv based pose estimation method
        #define METHOD_PNP  0   ///< PNP based pose estimation method
    
    private:

        /// @brief init single_drone_position_calcualtor 
        void init();

        /// @brief main function of single drone target position calculation
        void targetPositionCalculator();

        /// @brief callback function of drone state ros_msg.
        void droneStateCallback(const drone_pose_estimation::DroneState::ConstPtr &drone_msg_const);
        
        /// @brief function of camera distortion correction of image points vector.
        void distortionCorrection(vector<cv::Point2f> img_positions, 
                                  vector<cv::Point2f>& dist_img_positions,
                                  cv::Mat camera_matrix,
                                  vector<double> camera_dist_coeffs);

        /// @brief function of camera distortion correction of one single image position.
        void distortionCorrection(cv::Point2f img_position, 
                                  cv::Point2f& dist_img_position,
                                  cv::Mat camera_matrix,
                                  vector<double> camera_dist_coeffs);
        
        /// @brief transform euler angle to rotation matrix.
        void eulerToRotation(Eigen::Vector3d euler_angle,
                             Eigen::Matrix<double, 4, 4> & rotation,
                             int rotation_order);

        /// @brief transform euler angle to rotation matrix.
        void eulerToRotation(Eigen::Vector3d euler_angle,
                             Eigen::Matrix<double, 3, 3> & rotation,
                             int rotation_order);

        /// @brief transform from old coordinate frame to new coordinate frame.
        void coordinateFrameTransform(Eigen::Vector3d euler_angle,
                                      Matrix<double, 3, 1> translation,
                                      Matrix<double, 4, 1> src_frame,
                                      Matrix<double, 4, 1>& dest_frame,
                                      Matrix<double, 4, 4>& rotation,
                                      int rotation_order);

        /// @brief transform from old coordinate frame to new coordinate frame.
        void coordinateFrameTransform(Eigen::Vector3d euler_angle,
                                      Matrix<double, 3, 1> translation,
                                      Matrix<double, 3, 1> src_frame,
                                      Matrix<double, 3, 1>& dest_frame,
                                      Matrix<double, 3, 3>& rotation,
                                      int rotation_order);
        
        /// @brief coordinate frame tran
        void geocentricCoordinateFrameTransform(Eigen::Vector3d gps_angle,
                                                Eigen::Matrix<double, 4, 1> src_frame,
                                                Eigen::Matrix<double, 4, 1>& dest_frame);

        /// @brief Transform LLH to NED coordinate frame.
        void LLH2NED(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1> NEDOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInNEDPosition);

        /// @brief Transform NED coordinate frame to LLH.
        void NED2LLH(Eigen::Matrix<double, 3, 1> targetInNEDPosition, 
                     Eigen::Matrix<double, 3, 1> NEDOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform LLH to geocentric coordinate frame.
        void LLH2XYZ(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        /// @brief Transform geocentric coordinate frame to LLH.
        void XYZ2LLH(Eigen::Matrix<double, 3, 1> targetInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform ENU coordinate frame to LLH.
        void ENU2LLH(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform ENU coordinate frame to geocentric coordinate frame.
        void ENU2XYZ(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        /// @brief console output function of calculator results.       
        void consoleOutput(vector<geodetic_target> geodetic_targets);

        /// @brief ros_topic output function of calculator results.
        void rostopicOutput(vector<geodetic_target> geodetic_targets);

        /// @brief distance calculator of two llh positions.
        void llhDistanceCalculator(Eigen::Matrix<double, 2, 1> target_position, 
                                   Eigen::Matrix<double, 2, 1> base_position,
                                   double& distance,
                                   double& angle);

        // Ros node handle
        ros::NodeHandle nodeHandle_;

        int sub_count_ = 0;                 ///< count of estimation result callback.
        bool gps_first_flag_ = false;       ///< flag of first gps flag.
        int msg_count_ = 0;                 ///< count of synchronizer message, same with count of droneStateCallback

        bool enable_console_output_;        ///< flag if enable console output.
        bool enable_rostopic_output_;        ///< flag if enable ros topic output.
        bool enable_offine_NED_position_;    ///< enable offline NED position, if true, set local NED LLH position in ros_param.

        double calculator_altitude_;     ///< drone altitude starting estimation, only the drone altitude is beyond this varible estimation starts.
        double car_static_altitude_;     ///< car static altitude, calculate according to localNEDOriginInLLHPosition_[2]

        Eigen::Matrix<double, 3, 1> localNEDOriginInLLHPosition_offline_; ///< Origin LLH of the local NED(world) coordinate frame[latitude, longitude, altitude][rad, rad, m]
        Eigen::Matrix<double, 3, 1> localNEDOriginInLLHPosition_;        ///< Origin LLH of the local NED(world) coordinate frame[latitude, longitude, altitude][rad, rad, m]
        Eigen::Matrix<double, 3, 1> localNEDOriginInXYZPosition_;        ///< Origin XYZ of the local NED(world) coordinate frame[x, y, z][m, m, m]

        /// drone position.
        Eigen::Matrix<double, 3, 1> droneInLLHPosition_;        ///< drone position in LLH coordinate frame.
        Eigen::Matrix<double, 3, 1> droneInLocalNEDPosition_;   ///< drone position in local NED(world) coordinate frame.
        Eigen::Matrix<double, 3, 1> localNEDToDroneEuler_;      ///< euler angles:[roll, pitch, yaw] rotated from local NED coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 3, 1> droneToLocalNEDEuler_;      ///< euler angles:[roll, pitch, yaw] rotated from drone coordinate frame to local NED coordinate frame.
        Eigen::Matrix<double, 3, 3> localNEDToDroneRotation_;
        Eigen::Matrix<double, 3, 3> droneToLocalNEDRotation_;

        /// camera to drone euler angles and origin position
        Eigen::Matrix<double, 3, 1> cameraToDroneEuler_;                        ///<! euler angles: (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from camera coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 3, 1> droneToCameraEuler_;                        ///< euler angles: (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from drone coordinate frame to gimbal coordinate frame.
        Eigen::Matrix<double, 3, 3> cameraToDroneRotation_;                     ///<! rotation matrix from camera coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 3, 3> droneToCameraRotation_;                     ///< rotation matrix from drone coordinate frame to camera coordinate frame.
        Eigen::Matrix<double, 3, 1> cameraOriginInDronePosition_;               ///<! origin position of camera coordinate frame in drone coordinate frame.
        
        
        
        Eigen::Matrix<double, 3, 1> cameraTocameraNEDEuler_;                      ///< euler angles:[roll, pitch, yaw] rotated from camera coordinate frame to camera NED coordinate frame.
        Eigen::Matrix<double, 3, 1> cameraNEDTocameraEuler_;
        Eigen::Matrix<double, 3, 3> cameraToCameraNEDRotation_;       ///< rotation matrix rotated from camera coordinate frame to camera NED coordinate frame.
        Eigen::Matrix<double, 3, 3> cameraNEDToCameraRotation_;       ///< rotation matrix rotated from camera NED coordinate frame to camera coordinate frame.
        Eigen::Matrix<double, 3, 1> cameraOriginInCameraNEDPosition_;


        /// target image position
        cv::Point2f target_img_position_;         ///< the image positions of targets whose world position need to be estimated

        /// car-GPS to car euler angles and origion position.
        Eigen::Vector3d carGPSToCarEuler_;                          ///<! euler angles (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from car-GPS coordinate frame to car coordinate frame.
        Eigen::Matrix<double, 3, 1> carGPSOriginInCarPosition_;     ///<! origin position (x, y, z) <m, m, m> of car-GPS coordinate frame in car coordinate frame.

        /// camera parameters
        double f_, u0_, v0_;                    ///< camera intrinsic parameters
        double k1_, k2_, p1_, p2_;              ///< camera distorition parameters
        double u_, v_, sensor_u_, sensor_v_;

        /// image targets vector
        vector<image_target> image_targets_;

        /// target geodetic position 
        vector<geodetic_target> geodetic_targets_;

        /// filters defination
        double time_stamp_;                                     ///< time stamp now from drone_state message.
        GPSKalmanFilter2d kf_filter_[MAX_TARGETS_NUM];          ///< KF filters to estimate target spatial positions


        ros::Publisher ground_target_state_pub_;    ///< publisher of the estimated car state.

        /// subscriber
        string drone_state_string_;
        ros::Subscriber drone_state_sub_;           ///< subscriber of the drone state.

        std::string origin_path_;

};