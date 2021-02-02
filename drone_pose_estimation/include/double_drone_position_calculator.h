/*
 * @Author: your name
 * @Date: 2020-11-12 10:55:33
 * @LastEditTime: 2020-12-23 10:49:37
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/double_drone_position_calculator.h
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
#include <std_msgs/Header.h>

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

#include <drone_pose_estimation/DroneState.h>
#include <drone_pose_estimation/GroundTargetState.h>
#include <drone_pose_estimation/GroundTargetStates.h>


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

using namespace Eigen;
using namespace cv;
using namespace std;

const float deg2rad = pi/180.0;
const float rad2deg = 180.0/pi;


class doubleDronePositionCalculator
{
    public:
        doubleDronePositionCalculator(ros::NodeHandle nh);
        ~doubleDronePositionCalculator();
    public:
        #define ROTATE_XYZ  1   ///< rotation order:X, Y, Z
        #define ROTATE_ZYX  0   ///< rotation order:Z, Y, X

        #define METHOD_EKF  2   ///< EKF based pose estimation method
        #define METHOD_UV   1   ///< uv based pose estimation method
        #define METHOD_PNP  0   ///< PNP based pose estimation method
    
    private:
        /// @brief init double_drone_position_calcualtor
        void init();

        void droneStateCallback(const drone_pose_estimation::DroneState::ConstPtr &slave_msg_const,
                                const drone_pose_estimation::DroneState::ConstPtr &master_msg_const);

        void targetPositionCalculator();

        void distortionCorrection(vector<cv::Point2f> img_positions, 
                                  vector<cv::Point2f>& dist_img_positions,
                                  cv::Mat camera_matrix,
                                  vector<double> camera_dist_coeffs);

        void distortionCorrection(cv::Point2f img_position, 
                                  cv::Point2f& dist_img_position,
                                  cv::Mat camera_matrix,
                                  vector<double> camera_dist_coeffs);

        void targetsMatch(vector<image_target> a_src_targets,
                          vector<image_target> b_src_targets,
                          vector<image_target>& a_des_targets,
                          vector<image_target>& b_des_targets);
        
        /// @brief transform euler angle to rotation matrix.
        void eulerToRotation(Eigen::Vector3d euler_angle,
                             Eigen::Matrix<double, 4, 4> & rotation,
                             int rotation_order);

        /// @brief transform from old coordinate frame to new coordinate frame.
        void coordinateFrameTransform(Eigen::Vector3d euler_angle,
                                      Matrix<double, 3, 1> translation,
                                      Matrix<double, 4, 1> src_frame,
                                      Matrix<double, 4, 1>& dest_frame,
                                      Matrix<double, 4, 4>& rotation,
                                      int rotation_order);
        
        void geocentricCoordinateFrameTransform(Eigen::Vector3d gps_angle,
                                                Eigen::Matrix<double, 4, 1> src_frame,
                                                Eigen::Matrix<double, 4, 1>& dest_frame);
        
        void simpleSynergyRendezvousPosition(Eigen::Matrix<double, 4, 1> a_targetInXYZDirection,
                                             Eigen::Matrix<double, 4, 1> b_targetInXYZDirection,
                                             Eigen::Matrix<double, 3, 1> a_inXYZPosition,
                                             Eigen::Matrix<double, 3, 1> b_inXYZPosition,
                                             Eigen::Matrix<double, 3, 1>& targetInXYZPosition);
        
        void improveSynergyRendezvousPosition(Eigen::Matrix<double, 4, 1> a_targetInXYZDirection,
                                              Eigen::Matrix<double, 4, 1> b_targetInXYZDirection,
                                              Eigen::Matrix<double, 3, 1> a_inXYZPosition,
                                              Eigen::Matrix<double, 3, 1> b_inXYZPosition,
                                              Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        void LLH2NED(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1> NEDOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInNEDPosition);

        void NED2LLH(Eigen::Matrix<double, 3, 1> targetInNEDPosition, 
                     Eigen::Matrix<double, 3, 1> NEDOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        void LLH2XYZ(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        void XYZ2LLH(Eigen::Matrix<double, 3, 1> targetInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        void ENU2LLH(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        void ENU2XYZ(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);
                     
        void consoleOutput(vector<geodetic_target> geodetic_targets);
        void rostopicOutput(vector<geodetic_target> geodetic_targets, std_msgs::Header header);

        void llhDistanceCalculator(Eigen::Matrix<double, 2, 1> target_position, 
                                   Eigen::Matrix<double, 2, 1> base_position,
                                   double& distance,
                                   double& angle);



    private:       
        // Ros node handle
        ros::NodeHandle nodeHandle_;

        int sub_count_ = 0;                 ///< count of estimation result callback.
        bool gps_first_flag_ = false;       ///< flag of first gps flag.
        int msg_count_ = 0;                 ///< count of synchronizer message, same with count of droneStateCallback

        bool enable_console_output_;        ///< flag if enable console output.
        bool enable_rostopic_output_;        ///< flag if enable ros topic output.

        double calculator_altitude_;     ///< drone altitude starting estimation, only the drone altitude is beyond this varible estimation starts.
        double car_static_altitude_;     ///< car static altitude, calculate according to localNEDOriginInLLHPosition_[2]

        Eigen::Matrix<double, 3, 1> localNEDOriginInLLHPosition_;        ///< Origin LLH of the local NED(world) coordinate frame[latitude, longitude, altitude][rad, rad, m]
        Eigen::Matrix<double, 3, 1> localNEDOriginInXYZPosition_;        ///< Origin XYZ of the local NED(world) coordinate frame[x, y, z][m, m, m]

        /// <master> drone position.
        Eigen::Matrix<double, 3, 1> m_droneInLLHPosition_;          ///< drone position in LLH coordinate frame.
        Eigen::Matrix<double, 3, 1> m_droneInLocalNEDPosition_;     ///< drone position in local NED(world) coordinate frame.
        Eigen::Matrix<double, 3, 1> m_localNEDToDroneEuler_;                    ///< euler angles:[roll, pitch, yaw] rotated from local NED coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 3, 1> m_droneToLocalNEDEuler_;                    ///< euler angles:[roll, pitch, yaw] rotated from drone coordinate frame to local NED coordinate frame.
        Eigen::Matrix<double, 4, 4> m_localNEDToDroneRotation_;     ///< rotation matrix rotated from local NED coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 4, 4> m_droneToLocalNEDRotation_;     ///< rotation matrix rotated from drone coordinate frame to local NED coordinate frame.
        /// <slaver> drone position.
        Eigen::Matrix<double, 3, 1> s_droneInLLHPosition_;
        Eigen::Matrix<double, 3, 1> s_droneInLocalNEDPosition_;
        Eigen::Matrix<double, 3, 1> s_localNEDToDroneEuler_;
        Eigen::Matrix<double, 3, 1> s_droneToLocalNEDEuler_;
        Eigen::Matrix<double, 4, 4> s_localNEDToDroneRotation_;
        Eigen::Matrix<double, 4, 4> s_droneToLocalNEDRotation_;


        /// <master> camera to drone euler angles and origin position
        Eigen::Matrix<double, 3, 1> m_cameraToDroneEuler_;                          ///< euler angles: (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from camera coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 3, 1> m_droneToCameraEuler_;                          ///< euler angles: (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from drone coordinate frame to gimbal coordinate frame.
        Eigen::Matrix<double, 4, 4> m_cameraToDroneRotation_;           ///< rotation matrix from camera coordinate frame to drone coordinate frame.
        Eigen::Matrix<double, 4, 4> m_droneToCameraRotation_;           ///< rotation matrix from drone coordinate frame to camera coordinate frame.
        Eigen::Matrix<double, 3, 1> m_cameraOriginInDronePosition_;                 ///< origin position of camera coordinate frame in drone coordinate frame.
        
        /// <slaver> camera to drone euler angles and origin position
        Eigen::Matrix<double, 3, 1> s_cameraToDroneEuler_;
        Eigen::Matrix<double, 3, 1> s_droneToCameraEuler_;
        Eigen::Matrix<double, 4, 4> s_cameraToDroneRotation_;
        Eigen::Matrix<double, 4, 4> s_droneToCameraRotation_;
        Eigen::Matrix<double, 3, 1> s_cameraOriginInDronePosition_;

        /// FOR DJI SYSTEM, which gimbal euler angle related to camera NED coordinate frame.
        /// <master> 
        Eigen::Matrix<double, 3, 1> m_cameraTocameraNEDEuler_;                      ///< euler angles:[roll, pitch, yaw] rotated from camera coordinate frame to camera NED coordinate frame.
        Eigen::Matrix<double, 4, 4> m_cameraToCameraNEDRotation_;       ///< rotation matrix rotated from camera coordinate frame to camera NED coordinate frame.
        Eigen::Matrix<double, 4, 4> m_cameraNEDToCameraRotation_;       ///< rotation matrix rotated from camera NED coordinate frame to camera coordinate frame.
        Eigen::Matrix<double, 3, 1> m_cameraOriginInCameraNEDPosition_;             ///< origin position of camera coordinate frame in camera NED coordinate frame.
        /// <slaver> 
        Eigen::Matrix<double, 3, 1> s_cameraTocameraNEDEuler_;
        Eigen::Matrix<double, 4, 4> s_cameraToCameraNEDRotation_;
        Eigen::Matrix<double, 4, 4> s_cameraNEDToCameraRotation_;
        Eigen::Matrix<double, 3, 1> s_cameraOriginInCameraNEDPosition_;


        /// <master> target image position
        cv::Point2f m_target_img_position_;         ///< the image positions of targets whose world position need to be estimated
        /// <slaver> target image position
        cv::Point2f s_target_img_position_;


        /// car-GPS to car euler angles and origion position.
        Eigen::Vector4d carGPSToCarEuler_;                          ///<! euler angles (order: Z, Y, X) [yaw, pitch, roll] <rad, rad, rad> rotated from car-GPS coordinate frame to car coordinate frame.
        Eigen::Matrix<double, 3, 1> carGPSOriginInCarPosition_;     ///<! origin position (x, y, z) <m, m, m> of car-GPS coordinate frame in car coordinate frame.

        /// <master> camera parameters
        double m_f_, m_u0_, m_v0_;                      ///< camera intrinsic parameters
        double m_k1_, m_k2_, m_p1_, m_p2_;              ///< camera distorition parameters
        double m_u_, m_v_, m_sensor_u_, m_sensor_v_;                               ///< camera pixel width and height

        /// <slaver> camera parameters
        double s_f_, s_u0_, s_v0_;                    ///< camera intrinsic parameters
        double s_k1_, s_k2_, s_p1_, s_p2_;              ///< camera distorition parameters
        double s_u_,  s_v_, s_sensor_u_, s_sensor_v_;                           ///< camera pixel width and height

        /// <master> image targets vector
        vector<image_target> m_image_targets_;
        vector<image_target> s_image_targets_;

        /// target geodetic position 
        vector<geodetic_target> geodetic_targets_;
        

        ///***** filters defination *****///
        double time_stamp_;                                     ///< time stamp now from drone_state message.
        GPSKalmanFilter2d kf_filter_[MAX_TARGETS_NUM];          ///< KF filters to estimate target spatial positions



        ///plane_EKF ekf_filter_[7];                   ///< EKF filters to estimate target spatial positions
        // long ekf_last_time_[7];                     ///< system time of last EKF based estimations
        // double ekf_delta_time_;                     ///< max interval of EKF filter between neighboring estimation. 
        //                                             ///  if interval of neighboring estimation is beyond this variable, this filter need to be re-initialization.
        // Eigen::Matrix<bool, 7, 1> ekf_init_flag_;   ///< flags of EKF filters initialization.
        // Eigen::Matrix<double, 4, 7> ekf_x0_;        ///< inital state x0 of EKF filter

        /// publisher
        ros::Publisher ground_target_state_pub_;        ///< publisher of the estimated target state.
        string ground_target_state_string_;         ///< name string of ground target state ros_topic 

        /// subscriber
        string master_state_string_;                    ///< name string of master drone state ros_topic
        string slaver_state_string_;                    ///< name string of slaver drone state ros_topic

        std::string origin_path_;
};