/*
 * @Author: your name
 * @Date: 2020-12-25 15:19:51
 * @LastEditTime: 2020-12-25 20:10:27
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/include/three_drone_position_estimation.cpp
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

#define MAX_TARGETS_NUM         3                   ///< max number of ground targets in this experimation.

using namespace Eigen;
using namespace cv;
using namespace std;

const float deg2rad = pi/180.0;
const float rad2deg = 180.0/pi;

class threeDronePositionFusion
{
    public:
        threeDronePositionFusion(ros::NodeHandle nh);
        ~threeDronePositionFusion();

    private:
        void init();

        void targetPositionFusion();

        void targetStateCallback(const drone_pose_estimation::GroundTargetStates::ConstPtr &drone_01_msg_const,
                                                   const drone_pose_estimation::GroundTargetStates::ConstPtr &drone_02_msg_const,
                                                   const drone_pose_estimation::GroundTargetStates::ConstPtr &drone_03_msg_const);

        void targetsMatch(vector<geodetic_target> drone_01_src_targets,
                                            vector<geodetic_target> drone_02_src_targets,
                                            vector<geodetic_target> drone_03_src_targets,
                                            vector<geodetic_target>& drone_01_des_targets,
                                            vector<geodetic_target>& drone_02_des_targets,
                                            vector<geodetic_target>& drone_03_des_targets);

        void consoleOutput(vector<geodetic_target> geodetic_targets);
        void rostopicOutput(vector<geodetic_target> geodetic_targets, std_msgs::Header header);


    private:
        /// ros node_handle
        ros::NodeHandle nodeHandle_;

        int sub_count_;                 ///< count of estimation result callback. 
        int max_targets_num_ = MAX_TARGETS_NUM;

        bool enable_console_output_;        ///< flag if enable console output.
        bool enable_rostopic_output_;        ///< flag if enable ros topic output.

        /// publisher
        ros::Publisher ground_target_state_pub_;        ///< publisher of the estimated target state.
        string ground_target_state_string_;         ///< name string of ground target state ros_topic 

        /// subscriber
        string drone_01_state_string_;                    ///< name string of drone_01 state ros_topic
        string drone_02_state_string_;                    ///< name string of drone_02 state ros_topic
        string drone_03_state_string_;                    ///< name string of drone_03 state ros_topic  


        /// ground targets position from each drone.

        vector<geodetic_target> drone_01_targets_;               ///< geodetic target vector from drone_01
        vector<geodetic_target> drone_02_targets_;               ///< geodetic target vector from drone_02
        vector<geodetic_target> drone_03_targets_;               ///< geodetic target vector from drone_03

        /// target geodetic position 
        vector<geodetic_target> geodetic_targets_;

        ///***** filters defination *****///
        double time_stamp_;                                     ///< time stamp now from drone_state message.
        GPSKalmanFilter2d kf_filter_[MAX_TARGETS_NUM];          ///< KF filters to estimate target spatial positions
};




































































