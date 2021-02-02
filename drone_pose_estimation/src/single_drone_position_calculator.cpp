
/*
 * @Author: your name
 * @Date: 2020-08-30 15:23:43
 * @LastEditTime: 2020-12-18 11:38:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: 
 */

#include "single_drone_position_calculator.h"

/**
 * @description: constructed function of the class singleDronePositionCalculator
 * @param [in]  nh  ros::NodeHandle 
 */
singleDronePositionCalculator::singleDronePositionCalculator(ros::NodeHandle nh) 
    : nodeHandle_(nh)
{
    ROS_INFO("single_drone_position_calculator_node start.");

    /// system parameters initilzation.
    nodeHandle_.param("enable_console_output", enable_console_output_, true);
    nodeHandle_.param("enable_rostpoic_output", enable_rostopic_output_, true);
    nodeHandle_.param("enable_offine_NED_position", enable_offine_NED_position_, true);

    /// subscribe topic name initilzation.
    nodeHandle_.param<string>("drone_state_ros_topic", drone_state_string_, "info_node/drone_state");

    /// set offline local NED position in LLH frame.
    double offline_latitude, offline_longitude, offline_altitude;
    nodeHandle_.param("offline_NED_latitude", offline_latitude, 28.2703016385);
    nodeHandle_.param("offline_NED_longitude", offline_longitude, 113.038538401);
    nodeHandle_.param("offline_NED_altitude", offline_altitude, 23.0);

    localNEDOriginInLLHPosition_offline_ << offline_latitude * deg2rad, offline_longitude * deg2rad, offline_altitude;

    /// camera paramaters initilzation.
    nodeHandle_.param("camera_f", f_, X5S_FOCAL_LENGTH);
    nodeHandle_.param("camera_u0", u0_, X5S_PIXEL_WIDTH/2);
    nodeHandle_.param("camera_v0", v0_, X5S_PIXEL_HEIGHT/2);
    nodeHandle_.param("camera_u", u_, X5S_PIXEL_WIDTH);
    nodeHandle_.param("camera_v", v_, X5S_PIXEL_HEIGHT);
    nodeHandle_.param("sensor_u", sensor_u_, X5S_SENSOR_WIDTH);
    nodeHandle_.param("sensor_v", sensor_v_, X5S_SENSOR_HEIGHT);

    nodeHandle_.param("camera_k1", k1_, X5S_K1);
    nodeHandle_.param("camera_k2", k2_, X5S_K2);
    nodeHandle_.param("camera_p1", p1_, X5S_P1);
    nodeHandle_.param("camera_p2", p2_, X5S_P2);

    /// camera to drone origin position.
    nodeHandle_.param("camera_drone_x", cameraOriginInDronePosition_[0], 0.0);
    nodeHandle_.param("camera_drone_y", cameraOriginInDronePosition_[1], 0.0);
    nodeHandle_.param("camera_drone_z", cameraOriginInDronePosition_[2], 0.0);

    /// camera origin in camera NED position.
    nodeHandle_.param("camera_cameraNED_x", cameraOriginInCameraNEDPosition_[0], 0.0);
    nodeHandle_.param("camera_cameraNED_y", cameraOriginInCameraNEDPosition_[1], 0.0);
    nodeHandle_.param("camera_cameraNED_z", cameraOriginInCameraNEDPosition_[2], 0.0);

    /// car-GPS to car euler angles and origion position.
    nodeHandle_.param("carGPS_car_roll", carGPSToCarEuler_[0], 0.0);
    nodeHandle_.param("carGPS_car_pitch", carGPSToCarEuler_[1], 0.0);
    nodeHandle_.param("carGPS_car_yaw", carGPSToCarEuler_[2], 0.0);

    nodeHandle_.param("carGPS_car_x", carGPSOriginInCarPosition_[0], 0.0);
    nodeHandle_.param("carGPS_car_y", carGPSOriginInCarPosition_[1], 0.0);
    nodeHandle_.param("carGPS_car_z", carGPSOriginInCarPosition_[2], 0.0);;



    /// ekf filter initilzation
    // ekf_init_flag_ << 0, 0, 0, 0, 0, 0, 0;      ///< set ekf_init_flag to <false>.
    // ekf_delta_time_ = 20.0;                     ///< set ekf_delta_time_ to 20 seconds.

    gps_first_flag_ = 0;                        ///< flag of first gps set to false, waiting for first gps message.
    calculator_altitude_ = -200.0;                ///< uav altitude starts the estimation. 


    origin_path_ = "origin.txt";

    /// init doubleDronePositionCalculator and start calculation
    init();
}

/**
 * @description: deconstructed function of the class doubleDronePositionCalculator  
 */
singleDronePositionCalculator::~singleDronePositionCalculator()
{
    ROS_INFO("single_drone_position_calculator_node stop.");
}

/**
 * @description: init function of the class doubleDronePositionCalculator. Defination of subscriber, time
 *               sychronizer and publisher.  
 */
void singleDronePositionCalculator::init()
{
    ROS_INFO("single_drone_position_calculator_node init.");

    // define subscriber
    drone_state_sub_ = nodeHandle_.subscribe(drone_state_string_, 1, &singleDronePositionCalculator::droneStateCallback, this);
    ROS_INFO("subscribe topic: %s", drone_state_string_.c_str());
    // define publishier
    ground_target_state_pub_ = nodeHandle_.advertise<drone_pose_estimation::GroundTargetStates>("drone_pose_estimation/ground_target_state", 10);

    
}

/**
 * @description: 
 * @param {*}
 * @return {*}
 */
void singleDronePositionCalculator::droneStateCallback(const drone_pose_estimation::DroneState::ConstPtr &drone_msg_const)
{
    ROS_INFO("drone state callback at frame %d", msg_count_);
    msg_count_ = msg_count_ + 1;

    drone_pose_estimation::DroneState drone_msg = *drone_msg_const;

    /// the first gps data is defined as the origin of world coordinate frame.
    if(!gps_first_flag_){       /// first time entering this callback function.
        ROS_INFO("set origin position to local txt");

        /// create a txt file to save origin LLH of local NED coordinate frame.
        ofstream origin_file(origin_path_.c_str());
        origin_file.setf(ios::fixed, ios::floatfield);
        origin_file.precision(9);
            
        if(!origin_file){       /// create .txt failed.
            ROS_WARN("write origin position to file failed!");
            return;
        }
        else{                   /// create .txt success.
            if(enable_offine_NED_position_){     /// set local NED position to offline position
                ROS_INFO("using offline local NED origion position");
                localNEDOriginInLLHPosition_ << localNEDOriginInLLHPosition_offline_(0),
                                                localNEDOriginInLLHPosition_offline_(1),
                                                localNEDOriginInLLHPosition_offline_(2);
            }
            else{                               /// set local NED position to drone online position
                ROS_INFO("using online local NED origion position");
                localNEDOriginInLLHPosition_<< drone_msg.gps.latitude * deg2rad, 
                                               drone_msg.gps.longitude * deg2rad, 
                                               drone_msg.gps.altitude;
            }
                  
            /// drone message ros time stamp(s).
            double time_stamp = drone_msg.header.stamp.sec + drone_msg.header.stamp.nsec / 1000000000.0;

            /// save origin LLH or local NED coordinate frame in .txt
            origin_file << time_stamp << " ";
            origin_file << drone_msg.gps.latitude << " " << drone_msg.gps.longitude << " " << drone_msg.gps.altitude;
            origin_file.close();
        }

        car_static_altitude_ = localNEDOriginInLLHPosition_[2];
        /// transform local NED origin LLH to geocentric XYZ coordinate frame.
        LLH2XYZ(localNEDOriginInLLHPosition_, localNEDOriginInXYZPosition_);
        gps_first_flag_ = true;
    }
        
    /// refresh the time (for kf/ekf filter)
    time_stamp_ = drone_msg.header.stamp.sec + drone_msg.header.stamp.nsec / 1000000000.0;

    /// refresh the gps data of drone.
    droneInLLHPosition_ <<  drone_msg.gps.latitude * deg2rad, drone_msg.gps.longitude * deg2rad, drone_msg.gps.altitude;    
    LLH2NED(droneInLLHPosition_, localNEDOriginInXYZPosition_, droneInLocalNEDPosition_);


    /// refresh the attitude (euler angle) data of drone.
    /// the attitude is defined as euler angles from world coordinate frame to drone coordinate frame, the rotation order is Z, Y, X.
    localNEDToDroneEuler_ << drone_msg.atti.x, drone_msg.atti.y, drone_msg.atti.z;  ///< euler angles [msg.attitude.roll, msg.attitude.pitch, msg.attitude.yaw] from local NED coordinate frame to drone frame.
    eulerToRotation(localNEDToDroneEuler_, localNEDToDroneRotation_, ROTATE_ZYX);
    droneToLocalNEDRotation_ = localNEDToDroneRotation_.transpose();

    // /// refresh the euler data of gimbal. [roll, pitch, yaw] <rad, rad, rad>
    // cameraToDroneEuler_ << drone_msg.gimbal.x, drone_msg.gimbal.y, drone_msg.gimbal.z;
    // eulerToRotation(cameraToDroneEuler_, cameraToDroneRotation_, ROTATE_ZYX);
    // droneToCameraRotation_ = cameraToDroneRotation_.transpose();

    /// FOR DJI SYSTEM, refresh the euler data of gimbal. [roll, pitch, yaw] <rad, rad, rad>

    cameraTocameraNEDEuler_ << drone_msg.gimbal.y * deg2rad * -1.0, drone_msg.gimbal.x * deg2rad * -1.0, drone_msg.gimbal.z * deg2rad * -1.0;
    eulerToRotation(cameraTocameraNEDEuler_, cameraToCameraNEDRotation_, ROTATE_ZYX);
    // cameraNEDTocameraEuler_ <<drone_msg.gimbal.y * deg2rad, drone_msg.gimbal.x * deg2rad, drone_msg.gimbal.z * deg2rad;
    // eulerToRotation(cameraNEDTocameraEuler_ , cameraNEDToCameraRotation_, ROTATE_ZYX);

    // std::cout << "cameraTocameraNEDEuler_" << cameraTocameraNEDEuler_ << std::endl;
    // std::cout << "cameraToCameraNEDRotation_" << cameraToCameraNEDRotation_ << std::endl;

    /// refresh the image position of target.
    /// <master>
    vector<image_target> image_targets;
    for (int i = 0; i < drone_msg.image_positions.size(); i++)
    {
        image_target this_image_target;           ///< a temp of image_target.

        this_image_target.id = int(drone_msg.image_positions[i].id);
        this_image_target.probabilities = double(drone_msg.image_positions[i].probability);
        this_image_target.Class = drone_msg.image_positions[i].Class;

        float this_x = (drone_msg.image_positions[i].xmin + drone_msg.image_positions[i].xmax) / 2;   ///< image position in x-axis of the bounding box center 
        float this_y = (drone_msg.image_positions[i].ymin + drone_msg.image_positions[i].ymax) / 2;   ///< image position in y-axis of the bounding box center
        this_image_target.img_position = cv::Point2f(this_x, this_y);

        /// distortion correction
        cv::Mat camera_matrix = (Mat_<double>(3,3) << f_, 0, u0_, 0, f_, v0_, 0, 0, 1);      ///< camera intrinsic parameters matrix.
        vector<double> camera_dist_coeffs;                                                        ///< camera distortion parameters vector.
        camera_dist_coeffs.push_back(k1_);
        camera_dist_coeffs.push_back(k2_);
        camera_dist_coeffs.push_back(p1_);
        camera_dist_coeffs.push_back(p2_);

        cv::Point2f this_dist_position = cv::Point2f(0.0, 0.0);         ///< temp image position of target after distortion correction.
        
        distortionCorrection(this_image_target.img_position, this_dist_position, camera_matrix, camera_dist_coeffs);
        // std::cout <<"this_image_target.img_position" << this_image_target.img_position << std::endl;
        
        /// set target image position after distortion correction.
        //this_image_target.dist_img_position = this_dist_position;
        this_image_target.dist_img_position = this_image_target.img_position;
        // std::cout << "this_image_target.dist_img_position"  << this_image_target.dist_img_position << std::endl;
 
        /// set image position of target, related to center of the image. <pixel>
        this_image_target.img_position_center = cv::Point2f( (this_image_target.dist_img_position.x - (u_ / 2)),
                                                             (this_image_target.dist_img_position.y - (v_ / 2)));
        /// set sensor position of target, realted to center of the sensor. <m>
        this_image_target.sensor_position_center = cv::Point2f((this_image_target.dist_img_position.x - (u_ / 2)) * sensor_u_  / u_,
                                                               (this_image_target.dist_img_position.y - (v_ / 2)) * sensor_v_ / v_);

        // std::cout << "this_image_target.img_position_center"  << this_image_target.img_position_center << std::endl;
        image_targets_.push_back(this_image_target);
    }


    /// target position calcualtor
    targetPositionCalculator();

    if (enable_console_output_)
    {
        consoleOutput(geodetic_targets_);
    }

    if (enable_rostopic_output_)
    {
        rostopicOutput(geodetic_targets_);
    }

    /// clear m_image_targets_ and s_image_targets_ vector.
    image_targets_.clear();
    image_targets_.shrink_to_fit();

    geodetic_targets_.clear();
    geodetic_targets_.shrink_to_fit();

}


void singleDronePositionCalculator::targetPositionCalculator()
{
    /// if drone position lower than calculator_altitude OR local NED position haven't initialized.
    if (droneInLLHPosition_(2) < calculator_altitude_ || !gps_first_flag_){
        return;
    }
    
    int targets_num = image_targets_.size();



    for (int i = 0; i < targets_num; i++)
    {
        int this_id =  image_targets_[i].id;

        if(this_id > MAX_TARGETS_NUM)   continue;     ///< illegal object id

        float this_probability = image_targets_[i].probabilities;
        string this_Class = image_targets_[i].Class;

        /// --1-- target's position in image coordinate frame transform to camera NED coordinate frame.
        Eigen::Matrix<double, 3, 3> C;          /// temp rotation matrix
        eulerToRotation(cameraTocameraNEDEuler_, C, ROTATE_ZYX);

        double camera_NED_relative_height;
        camera_NED_relative_height = droneInLLHPosition_[2] - localNEDOriginInLLHPosition_[2];  ///< relative height between camera and ground.

        float this_u = image_targets_[i].img_position.x;
        float this_v = image_targets_[i].img_position.y;

        // Eigen::Matrix<double, 4, 1> targetInCameraNEDPosition;

        // Eigen::Matrix<double, 3, 1> cc;         /// temp translation vector
        // cc[2] = f_ * camera_NED_relative_height / (C(2,0) * (this_u - u0_) + C(2,1) * (this_v - v0_) + C(2,2) * f_);
        
        // targetInCameraNEDPosition <<    cc[2] * (C(0,0) * (this_u - u0_) + C(0,1) * (this_v - v0_) + C(0,2) * f_) / f_,
        //                                 cc[2] * (C(1,0) * (this_u - u0_) + C(1,1) * (this_v - v0_) + C(1,2) * f_) / f_,
        //                                 camera_NED_relative_height,
        //                                 1;
        
        // std::cout << targetInCameraNEDPosition << std::endl;

        // /// --1.5-- FOR DJI system, camera NED frame == drone NED frame.
        // /// --2-- target's position in drone NED frame transform to geocentric (xyz) coordinate frame.
        // Eigen::Matrix<double, 4, 1> targetInXYZPosition;
        // Eigen::Matrix<double, 3, 1> cameraNEDOriginInLLHPosition = droneInLLHPosition_;
        // geocentricCoordinateFrameTransform(cameraNEDOriginInLLHPosition, targetInCameraNEDPosition, targetInXYZPosition);

        // /// --3-- target's location in LLH coordinate frame.
        // Eigen::Matrix<double, 3, 1> temp_targetInXYZPosition;
        // temp_targetInXYZPosition << targetInXYZPosition(0), targetInXYZPosition(1), targetInXYZPosition(2);
        // Eigen::Matrix<double, 3, 1> targetInLLHPosition;
        // XYZ2LLH(temp_targetInXYZPosition, targetInLLHPosition);
        Eigen::Matrix<double, 3, 1> targetInCameraPosition;
        targetInCameraPosition << f_,
                                  this_u - u0_,
                                  this_v - v0_;
        Eigen::Matrix<double, 3, 1> temp_targetInCameraNEDPosition;
        coordinateFrameTransform(cameraTocameraNEDEuler_, cameraOriginInCameraNEDPosition_, targetInCameraPosition,
                                 temp_targetInCameraNEDPosition, cameraToCameraNEDRotation_, ROTATE_XYZ);

        Eigen::Matrix<double, 3, 1> targetInCameraNEDPosition;
        targetInCameraNEDPosition = temp_targetInCameraNEDPosition / temp_targetInCameraNEDPosition(2) * camera_NED_relative_height;

        // Eigen::Matrix<double, 3, 1> cc;         /// temp translation vector
        // cc[2] = f_ * camera_NED_relative_height / (C(2,0) * (this_u - u0_) + C(2,1) * (this_v - v0_) + C(2,2) * f_);
        
        // targetInCameraNEDPosition <<    cc[2] * (C(0,0) * (this_u - u0_) + C(0,1) * (this_v - v0_) + C(0,2) * f_) / f_,
        //                                 cc[2] * (C(1,0) * (this_u - u0_) + C(1,1) * (this_v - v0_) + C(1,2) * f_) / f_,
        //                                 camera_NED_relative_height;

        // std::cout << this_u << this_v << std::endl;

        // std::cout<< "targetInCameraNEDPosition" << targetInCameraNEDPosition << std::endl;

        Eigen::Matrix<double, 3, 1> targetInLocalNEDPosition;
        targetInLocalNEDPosition = targetInCameraNEDPosition + droneInLocalNEDPosition_;

        // std::cout<< "targetInLocalNEDPosition" << targetInLocalNEDPosition << std::endl;

        // std::cout << "droneInLocalNEDPosition_" << droneInLocalNEDPosition_ <<std::endl;

        
        Eigen::Matrix<double, 3, 1> targetInLLHPosition;
        NED2LLH(targetInLocalNEDPosition, localNEDOriginInLLHPosition_, targetInLLHPosition);

        Eigen::Matrix<double, 3, 1> targetInLLHPosition_deg;
        targetInLLHPosition_deg << targetInLLHPosition(0) * rad2deg,
                                   targetInLLHPosition(1) * rad2deg,
                                   targetInLLHPosition(2);
        Eigen::Matrix<double, 3, 1> this_target_attitude;
        this_target_attitude << 0.0, 0.0, 0.0;

        /// --4-- save calculator results to geodetic_targets_ vector
        geodetic_target this_geodetic_target(this_id, this_probability, this_Class,
                                             targetInLLHPosition_deg, this_target_attitude);
        
        geodetic_targets_.push_back(this_geodetic_target);

    }
    
}



/**
 * @description: distortion correction for target image positions
 * @param [in]      img_positions       target image positions vector
 * @param [out]     dist_img_positions  target image positions vector after distortion correction
 * @param [in]      camera_matrix       camera matrix
 * @param [in]      camera_dist_coeffs  camera distortion matrix
 */
void singleDronePositionCalculator::distortionCorrection(vector<cv::Point2f> img_positions, 
                                                         vector<cv::Point2f>& dist_img_positions,
                                                         cv::Mat camera_matrix,
                                                         vector<double> camera_dist_coeffs)
{
    vector<cv::Point2f> distort_points_temp_vector;                              ///< temp variable vector of distortion correction.
    vector<cv::Point2f> undistort_points_temp_vector(img_positions);

    /// computes the ideal point coordinates from the observed point coordinates.
    /// https://docs.opencv.org/3.4.6/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e
    cv::undistortPoints(undistort_points_temp_vector, distort_points_temp_vector, camera_matrix, camera_dist_coeffs);

    /// The transformed point needs to be changed to a homogeneous coordinate system first, 
    /// and then it needs to be multiplied by the camera parameter matrix to be the final 
    /// changed coordinates
    cv::Mat center_points_three_dimensional = Mat::zeros(img_positions.size(), 3, CV_64FC1);
    for (size_t i = 0; i < img_positions.size(); i++){
        center_points_three_dimensional.at<double>(i, 0) = distort_points_temp_vector[i].x;
        center_points_three_dimensional.at<double>(i, 1) = distort_points_temp_vector[i].y;
        center_points_three_dimensional.at<double>(i, 2) = 1;
    }

    cv::Mat undistort_points_three_dimensional = camera_matrix * center_points_three_dimensional.t();

    for (size_t i = 0; i < img_positions.size(); i++){
        Point2f point = Point2f(undistort_points_three_dimensional.at<double>(0,i),
                                undistort_points_three_dimensional.at<double>(1,i));
        dist_img_positions.push_back(point);
    }
}

/**
 * @description: distortion correction for target image positions
 * @param [in]      img_position        one single target image position
 * @param [out]     dist_img_position   one single target image position after distortion correction
 * @param [in]      camera_matrix       camera matrix
 * @param [in]      camera_dist_coeffs  camera distortion matrix
 */
void singleDronePositionCalculator::distortionCorrection(cv::Point2f img_position, 
                                                         cv::Point2f& dist_img_position,
                                                         cv::Mat camera_matrix,
                                                         vector<double> camera_dist_coeffs)
{
    vector<cv::Point2f> distort_points_temp_vector;                              ///< temp variable vector of distortion correction.
    vector<cv::Point2f> undistort_points_temp_vector;

    undistort_points_temp_vector.push_back(img_position);

    /// computes the ideal point coordinates from the observed point coordinates.
    /// https://docs.opencv.org/3.4.6/da/d54/group__imgproc__transform.html#ga55c716492470bfe86b0ee9bf3a1f0f7e
    cv::undistortPoints(undistort_points_temp_vector, distort_points_temp_vector, camera_matrix, camera_dist_coeffs);

    /// The transformed point needs to be changed to a homogeneous coordinate system first, 
    /// and then it needs to be multiplied by the camera parameter matrix to be the final 
    /// changed coordinates
    cv::Mat center_points_three_dimensional = cv::Mat::zeros(1, 3, CV_64FC1);

    center_points_three_dimensional.at<double>(0, 0) = distort_points_temp_vector[0].x;
    center_points_three_dimensional.at<double>(0, 1) = distort_points_temp_vector[0].y;
    center_points_three_dimensional.at<double>(0, 2) = 1;

    cv::Mat undistort_points_three_dimensional = camera_matrix * center_points_three_dimensional.t();

    cv::Point2f point = cv::Point2f(undistort_points_three_dimensional.at<double>(0, 0),
                                    undistort_points_three_dimensional.at<double>(1, 0));

    dist_img_position = point;
    
}


/**
 * @description: Default rotation order: Z(yaw), Y(pitch), X(roll) with new axes respectively.
 * @param [in] eular_angle      eular angles to be transformed: [roll, pitch, yaw][rad, rad, rad]
 * @param [out] rotation        rotation matrix transformed from eular angles
 * @param [in] rotation_order   rotation order flag(ROTATE_ZYX: Z, Y, X, ROTATE_XYZ: X, Y, Z)
 */
void singleDronePositionCalculator::eulerToRotation(Eigen::Vector3d euler_angle,
                                                    Eigen::Matrix<double, 4, 4> & rotation,
                                                    int rotation_order = ROTATE_ZYX)
{
    double c_roll, c_pitch, c_yaw, s_roll, s_pitch, s_yaw;
    Eigen::Matrix<double, 4, 4> rotationMatrixRoll;                     /// rotation matrix around X axis.
    Eigen::Matrix<double, 4, 4> rotationMatrixPitch;                    /// rotation matrix around Y axis.
    Eigen::Matrix<double, 4, 4> rotationMatrixYaw;                      /// rotation matrix around Z axis.

    c_roll = cos(euler_angle[0]);       s_roll = sin(euler_angle[0]);   /// roll eular angle parameter [rad].
    c_pitch = cos(euler_angle[1]);      s_pitch = sin(euler_angle[1]);  /// pitch eular angle parameter [rad].
    c_yaw = cos(euler_angle[2]);        s_yaw = sin(euler_angle[2]);    /// yaw eular angle parameter [rad].

    rotationMatrixRoll  <<   1,      0,       0,    0,
                            0,  c_roll,  s_roll,    0,
                            0, -s_roll,  c_roll,    0,
                            0,       0,       0,    1;
    
    rotationMatrixPitch <<  c_pitch,    0, -s_pitch,    0,
                                  0,    1,        0,    0,
                            s_pitch,    0,  c_pitch,    0,
                                  0,    0,        0,    1;
    
    rotationMatrixYaw  <<    c_yaw, s_yaw,   0,     0,
                            -s_yaw, c_yaw,   0,     0,
                                 0,     0,   1,     0,
                                 0,     0,   0,     1;

    /// rotation order: Z, Y, X
    if(rotation_order == ROTATE_ZYX)
    {
        rotation = rotationMatrixRoll * rotationMatrixPitch * rotationMatrixYaw;
    }
    else if(rotation_order == ROTATE_XYZ){
        rotation = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;
    }
}

/**
 * @description: Default rotation order: Z(yaw), Y(pitch), X(roll) with new axes respectively.
 * @param [in] eular_angle      eular angles to be transformed: [roll, pitch, yaw][rad, rad, rad]
 * @param [out] rotation        rotation matrix transformed from eular angles <3, 3>
 * @param [in] rotation_order   rotation order flag(ROTATE_ZYX: Z, Y, X, ROTATE_XYZ: X, Y, Z)
 */
void singleDronePositionCalculator::eulerToRotation(Eigen::Vector3d euler_angle,
                                                    Eigen::Matrix<double, 3, 3> & rotation,
                                                    int rotation_order = ROTATE_ZYX)
{
    double c_roll, c_pitch, c_yaw, s_roll, s_pitch, s_yaw;
    Eigen::Matrix<double, 3, 3> rotationMatrixRoll;                     /// rotation matrix around X axis.
    Eigen::Matrix<double, 3, 3> rotationMatrixPitch;                    /// rotation matrix around Y axis.
    Eigen::Matrix<double, 3, 3> rotationMatrixYaw;                      /// rotation matrix around Z axis.

    c_roll = cos(euler_angle[0]);       s_roll = sin(euler_angle[0]);   /// roll eular angle parameter [rad].
    c_pitch = cos(euler_angle[1]);      s_pitch = sin(euler_angle[1]);  /// pitch eular angle parameter [rad].
    c_yaw = cos(euler_angle[2]);        s_yaw = sin(euler_angle[2]);    /// yaw eular angle parameter [rad].

    rotationMatrixRoll  <<   1,      0,       0,
                            0,  c_roll,  s_roll,
                            0, -s_roll,  c_roll;
    
    rotationMatrixPitch <<  c_pitch,    0, -s_pitch,
                                  0,    1,        0,
                            s_pitch,    0,  c_pitch;
    
    rotationMatrixYaw  <<    c_yaw, s_yaw,   0,
                            -s_yaw, c_yaw,   0,
                                 0,     0,   1;

    /// rotation order: Z, Y, X
    if(rotation_order == ROTATE_ZYX)
    {
        rotation = rotationMatrixRoll * rotationMatrixPitch * rotationMatrixYaw;
    }
    else if(rotation_order == ROTATE_XYZ){
        rotation = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;
    }
}

/**
 * @description: src_frame position in old coordinate frame to dest_frame position in new coordinate frame.
 *               eular angles(roll, pitch, yaw) are from old coordinate frame to new coordinate frame
 *               translation vector is the position of the old coordinate frame origin in new coordinate frame.
 *               default rotation order: Z（yaw), Y(pitch), X(roll) with new axes respectively.
 * @param [in] euler_angle      euler angles to be transformed
 * @param [in] translation      translation vector of the old frame origin in new coordinate frame.
 * @param [in] src_frame        source spatial position in old coordinate frame.
 * @param [out] dist_frame      destination position in new coordinate frame.
 * @param [out] rotation        rotation matrix transformed from euler angles.
 * @param [in]  rotation_order  rotation order flag(default: ROTATE_ZYX)
 */
void singleDronePositionCalculator::coordinateFrameTransform(Eigen::Vector3d euler_angle,
                                                             Matrix<double, 3, 1> translation,
                                                             Matrix<double, 4, 1> src_frame,
                                                             Matrix<double, 4, 1>& dest_frame,
                                                             Matrix<double, 4, 4>& rotation,
                                                             int rotation_order = ROTATE_ZYX)
{
    /// euler angles to rotation matrix trandformation
    eulerToRotation(euler_angle, rotation, rotation_order);

    Eigen::Matrix<double, 4, 4> translations;
    translations << 1.0,    0.0,    0.0,   -translation[0],
                    0.0,    1.0,    0.0,   -translation[1],
                    0.0,    0.0,    1.0,   -translation[1],
                    0.0,    0.0,    0.0,               1.0;

    /// compute destination position in new coordinate frame.
    dest_frame = translations * rotation * src_frame;
}

void singleDronePositionCalculator::coordinateFrameTransform(Eigen::Vector3d euler_angle,
                                                             Matrix<double, 3, 1> translation,
                                                             Matrix<double, 3, 1> src_frame,
                                                             Matrix<double, 3, 1>& dest_frame,
                                                             Matrix<double, 3, 3>& rotation,
                                                             int rotation_order = ROTATE_ZYX)
{
    /// euler angles to rotation matrix trandformation
    eulerToRotation(euler_angle, rotation, rotation_order);

    /// compute destination position in new coordinate frame.
    dest_frame = rotation * src_frame +  translation;
}


/**
 * @description: src_frame position in source NED coordinate frame to dest_frame position in earth geocetric XYZ coordinate frame.
 *               using gps angles(latitude, longitude, altitude), which are the LLH position of source NED coordinate frame.
 * @param [in] gps_angle        the LLH position of source NED coordinate frame.
 * @param [in] src_frame        source spatial position in NED coordinate frame.
 * @param [out] dist_frame      destination position in earth geocetric XYZ coordinate frame.
 */
void singleDronePositionCalculator::geocentricCoordinateFrameTransform(Eigen::Vector3d gps_angle,
                                                                       Eigen::Matrix<double, 4, 1> src_frame,
                                                                       Eigen::Matrix<double, 4, 1>& dest_frame)
{
    double c_lon, s_lon, c_lat, s_lat;
    c_lon = cos(gps_angle[1]);  s_lon = sin(gps_angle[1]);  /// longitude eular angle parameter [rad].
    c_lat = cos(gps_angle[0]);  s_lat = sin(gps_angle[0]);  /// latitude eular angle parameter [rad].

    Eigen::Matrix<double, 4, 4> rotationMatrixLon;           /// rotation matrix around longitude axis.
    Eigen::Matrix<double, 4, 4> rotationMatrixLat;           /// rotation matrix around latitude axis.

    rotationMatrixLon << c_lon, -s_lon, 0,  0,
                         s_lon, c_lon,  0,  0,
                         0,     0,      1,  0,
                         0,     0,      0,  1;
    
    rotationMatrixLat << c_lat, 0,  -s_lat, 0,
                         0,     1,  0,      0,
                         s_lat, 0,  c_lat,  0,
                         0,     0,  0,      1;
    
    Eigen::Matrix<double, 4, 4> rotation;
    rotation = rotationMatrixLon * rotationMatrixLat;

    Eigen::Matrix<double, 4 ,1> temp_frame;             ///< X-axis to top, Z-axis to N, Y-axis to E.  
    temp_frame << src_frame(2) * -1.0,
                  src_frame(1),
                  src_frame(0),
                  src_frame(3);
    
    dest_frame = rotation * temp_frame;
}


/**
 * @description: Firstly transform llh[] to geocentric coordinate frame by function LLH2XYZ(), then transform to NED coordinate frame.
 * @param[in]   targetInLLHPosition         [latitude, longitude, altitude] to be transformed
 * @param[in]   NEDOriginInXYZPosition      NED origin position in geocentric coordinate frame
 * @param[out]  targetInNEDPosition         spatial positions in local NED(world) coordinate frame
 */
void singleDronePositionCalculator::LLH2NED(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                                            Eigen::Matrix<double, 3, 1> NEDOriginInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInNEDPosition)
{
    Eigen::Matrix<double, 3, 1> targetInXYZPosition;
    Eigen::Matrix<double, 4, 4> initNEDToLocalNEDRotation;  ///< rotation matrix from NED(latitude=0, longitude=0) coordinate frame to local NED coordinate frame
    Eigen::Vector3d initNEDToLocalNEDEuler;                        ///< euler angles rotated from NED(latitude=0, longitude=0) coordinate frame to local NED coordinate frame
    
    /// transform to geocentric coordinate frame
    LLH2XYZ(targetInLLHPosition, targetInXYZPosition);

    initNEDToLocalNEDEuler[0] = targetInLLHPosition[1];
    initNEDToLocalNEDEuler[1] = -targetInLLHPosition[0];
    initNEDToLocalNEDEuler[2] = 0.0;
    eulerToRotation(initNEDToLocalNEDEuler, initNEDToLocalNEDRotation, ROTATE_XYZ);

    Eigen::Vector3d geocentricToInitNEDEuler;                   ///< euler angles rotating from ECEF geocentirc coordinate frame to NED coordinate frame
    Eigen::Matrix<double, 4, 4> geocentricToInitNEDRotation;    ///< rotation matrix from ECEF geocentirc coordinate frame to NED coordinate frame
    geocentricToInitNEDEuler << 0.0 * pi, -0.5 * pi, 0.0 * pi;
    eulerToRotation(geocentricToInitNEDEuler, geocentricToInitNEDRotation, ROTATE_ZYX);

    Eigen::Vector4d tempXYZPosition;
    tempXYZPosition << targetInXYZPosition[0] - NEDOriginInXYZPosition[0],
                       targetInXYZPosition[1] - NEDOriginInXYZPosition[1],
                       targetInXYZPosition[2] - NEDOriginInXYZPosition[2],
                       1.0;
    Eigen::Vector4d tempNEDPosition;

    //targetInNEDPosition = initNEDToLocalNEDRotation * geocentricToInitNEDRotation * (targetInXYZPosition - NEDOriginInXYZPosition);
    tempNEDPosition = initNEDToLocalNEDRotation * geocentricToInitNEDRotation * tempXYZPosition;
    targetInNEDPosition << tempNEDPosition[0], tempNEDPosition[1], tempNEDPosition[2];
}


/**
 * @description: Firstly transform NEDPosition to ENU coordinate frame, then transform to LLH by function ENU2LLH().
 * @param[in]   targetInNEDPosition        spatial positions in local NED(world) coordinate frame
 * @param[in]   NEDOriginInLLHPosition     NED origin position in LLH [latitude, longitude, altitude] coordinate frame
 * @param[out]  targetInLLHPosition        [latitude, longitude, altitude][rad, rad, m] after transformation
 * @see ENU2LLH
 */
void singleDronePositionCalculator::NED2LLH(Eigen::Matrix<double, 3, 1> targetInNEDPosition, 
                                            Eigen::Matrix<double, 3, 1> NEDOriginInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{
    Eigen::Matrix<double, 3, 1> targetInLocalENUPosition;

    Eigen::Vector3d NEDToENUEuler;                  ///< euler angles <rad, rad, rad> rotationg from NED coordinate frame to ENU coordinate frame.
    Eigen::Matrix<double, 4, 4> NEDToENURotation;   ///< rotation matrix rotationg from NED coordinate frame to ENU coordinate frame.
    /// NED to ENU euler angles and rotarion matrix.
    NEDToENUEuler << 1.0 * pi, 0.0 * pi, 0.5 * pi;
    eulerToRotation(NEDToENUEuler, NEDToENURotation, ROTATE_ZYX);

    /// transform to ENU fisrt
    // targetInLocalENUPosition = NEDToENURotation * targetInNEDPosition;

    Eigen::Vector4d tempNEDPosition;
    tempNEDPosition << targetInNEDPosition[0],
                       targetInNEDPosition[1],
                       targetInNEDPosition[2],
                       1.0;
    Eigen::Vector4d tempENUPosition;
    tempENUPosition = NEDToENURotation * tempNEDPosition;
    targetInLocalENUPosition << tempENUPosition[0], tempENUPosition[1], tempENUPosition[2];
    
    /// transform to LLH
    ENU2LLH(targetInLocalENUPosition, NEDOriginInLLHPosition, targetInLLHPosition);
    
}


/**
 * @description: llh to geocentric coordinate frame xyz transform.
 * @param [in]   targetInLLHPosition             [latitude, longitude, altitude][rad, rad, m] the gps coordinate.
 * @param [out]  targetInXYZPosition             [x, y, z] position in geocentric coordinate frame after transformation
 */
void singleDronePositionCalculator::LLH2XYZ(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInXYZPosition)
{
    /// --- START --- ///

    // double latitude = targetInLLHPosition(0);
    // double longitude = targetInLLHPosition(1);
    // double altitude = targetInLLHPosition(2);

    // double a = 6378137.0000;            ///< semi-major axis of the earth ellipsoid = 6378.137 km
    // double b = 6356752.3142;            ///< semi-minor axis of the earth ellipsoid = 6356.7523141 km.
    // double e = sqrt(1 - pow((b / a), 2));    ///< first eccentricity of the earth ellipsoid.

    // double sinphi = sin(latitude);
    // double cosphi = cos(latitude);
    // double coslam = cos(longitude);
    // double sinlam = sin(longitude);
    // double tan2phi = tan(latitude) * tan(latitude);
    // double tmp = 1 - e * e;
    // double tmpden = sqrt(1 + tmp * tan2phi);

    // double X = (a * coslam) / tmpden + altitude * coslam * cosphi;
    // double Y = (a * sinlam) / tmpden + altitude * sinlam * cosphi;

    // double tmp2 = sqrt(1 - e * e * sinphi * sinphi);
    // double Z = (a * tmp * sinphi) / tmp2 + altitude * sinphi;

    // targetInXYZPosition << X, Y, Z;

    /// --- END --- ///

    double latitude = targetInLLHPosition(0);
    double longitude = targetInLLHPosition(1);
    double altitude = targetInLLHPosition(2);

    double a = 6378137;         // The semi-major axis of the earth ellipsoid = 6378.137 km.
    double b = 6356752.3141;  // The semi-minor axis of the earth ellipsoid = 6356.7523141 km.
    double H = altitude + a;

    double e = sqrt(1 - (pow(b, 2) / pow(a, 2)));  //The first eccentricity of the earth ellipsoid.
    // double e1 = sqrt(pow(a, 2) - pow(b, 2)) / a;
    // double e = sqrt(0.006693421622966);      //Krasovski earth ellipsoid.
    // double e = sqrt(0.006694384999588);      //1975 earth ellipsoid.
    // double e = sqrt(0.0066943799013);        //WGS-84 earth ellipsoid.

    double B = latitude;
    double L = longitude;
    double W = sqrt(1 - pow(e, 2) * pow(sin(B), 2));    //First latitude function. 第一纬度函数
    double N = a / W;                           //radius of curvature in prime vertical. 椭球的卯酉圈曲率半径


    targetInXYZPosition <<    (N + H) * cos(B) * cos(L),
                (N + H) * cos(B) * sin(L),
                (N * (1 - pow(e, 2)) + H) * sin(B);

}



/**
 * @description: geocentric coordinate frame xyz to llh transform.
 * @param [in]  targetInXYZPosition             [x, y, z]  position in geocentric coordinate frame to be transformed
 * @param [out] targetInLLHPosition             [latitude, longitude, altitude][deg, deg, m] the gps coordinate after transformation.
 */
void singleDronePositionCalculator::XYZ2LLH(Eigen::Matrix<double, 3, 1> targetInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{
    /// --- START --- ///

    // double X = targetInXYZPosition(0);
    // double Y = targetInXYZPosition(1);
    // double Z = targetInXYZPosition(2);
    // double x2 = X * X;
    // double y2 = Y * Y;
    // double z2 = Z * Z;

    // double a = 6378137.0;
    // double b = 6356752.3142;
    // double e = sqrt(a * a - b * b) / a;

    // double b2 = b * b;
    // double e2 = e * e;
    // double ep = e * (a / b);
    // double r = sqrt(x2 + y2);
    // double r2 = r * r;
    // double E2 = a * a - b * b;
    // double F = 54 * b2 * z2;
    // double G = r2 + (1 - e2) * z2 - e2 * E2;
    // double c = (e2 * e2 * F * r2) / (G * G * G);
    // double s = pow(1 + c + sqrt(c * c + 2 * c), 1.0 / 3);
    // double P = F / (3 * pow(s + 1 / s + 1, 2) * G * G);
    // double Q = sqrt(1 + 2 * e2 * e2 * P);
    // double ro = -(P * e2 * r) / (1 + Q) + sqrt((a * a / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) - P * r2 / 2);
    // double tmp = pow(r - e2 * ro, 2);
    // double U = sqrt(tmp + z2);
    // double V = sqrt(tmp + (1 - e2) * z2);
    // double zo = (b2 * Z) / (a * V);

    // double height = U * (1 - b2 / (a * V));

    // double latitude = atan((Z + ep * ep * zo) / r);

    // double tmp2 = atan(Y / X);

    // double longitude;

    // if (X >= 0){
    //     longitude = tmp2;
    // }else if (X < 0 && Y >= 0){
    //     longitude = pi + tmp2;
    // }else{
    //     longitude = tmp2 - pi;
    // }
    
    // targetInLLHPosition << latitude, longitude, height;

    /// ---END--- ///

    /// https://www.cnblogs.com/marvelousone/p/11265895.html
    /// https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm?source=post_page
    
    double x = targetInXYZPosition(0);
    double y = targetInXYZPosition(1);
    double z = targetInXYZPosition(2);

    double s = sqrt(pow(x, 2) + pow(y, 2));
    double tanB = z / sqrt(pow(x, 2) + pow(y, 2));

    double a = 6378137;         // The semi-major axis of the earth ellipsoid = 6378.137 km.
    double b = 6356752.3141;  // The semi-minor axis of the earth ellipsoid = 6356.7523141 km.

    double e = sqrt(1 - pow(b, 2) / pow(a, 2));  //The first eccentricity of the earth ellipsoid.
    
    // double e = sqrt(0.006693421622966);      //Krasovski earth ellipsoid.
    // double e = sqrt(0.006694384999588);      //1975 earth ellipsoid.
    //double e = sqrt(0.0066943799013);        //WGS-84 earth ellipsoid.

    double L = fabs(acos(x / s));
    double B1 = z / sqrt(pow(x, 2) + pow(y, 2));

    double B2 = 0.0;
    double N = 0.0;

    while(fabs(B2 - B1) > 1E-6){
        N = a / sqrt(1 - pow(e, 2) * pow(sin(B1), 2));
        B2 = atan((z + N * pow(e,2) * sin(B1)) / s);
        B1 = atan((z + N * pow(e,2) * sin(B2)) / s);
    }

    double H = z / sin(B1) - N * (1 - pow(e, 2));


    double latitude = B1;
    double longitude = L;
    //double longitude = atan(y / x) / deg2rad;
    double altitude = H - a;

    if(latitude < 0){
        latitude = latitude + pi;
    }
    if(longitude < 0){
        longitude = longitude + pi;
    }

    targetInLLHPosition <<  latitude,
                            longitude,
                            altitude;

}


/**
 * @description: ENU coordinate frame to geocentric coordinate frame xyz
 * @param[in]   targetInLocalENUPosition    position in ENU coordinate frame to be transformed
 * @param[in]   ENUOriginInXYZPosition      position of the origin of ENU coordinate frame in geocentric coordinate frame
 * @param[out]  targetInXYZPosition         position in geocentric coordinate frame after transformation
 */
void singleDronePositionCalculator::ENU2XYZ(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                                            Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInXYZPosition)
{
    Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition;
    XYZ2LLH(ENUOriginInXYZPosition, ENUOriginInLLHPosition);

    double latitude_origin = ENUOriginInLLHPosition(0);
    double longitude_origin = ENUOriginInLLHPosition(1);

    double C11 = -sin(longitude_origin);
    double C12 = cos(longitude_origin);
    double C21 = -sin(latitude_origin) * cos(longitude_origin);
    double C22 = -sin(latitude_origin) * sin(longitude_origin);
    double C23 = cos(latitude_origin); 
    double C31 = cos(latitude_origin) * cos(longitude_origin);  
    double C32 = cos(latitude_origin) * sin(longitude_origin);
    double C33 = sin(latitude_origin);

    double x_origin = ENUOriginInXYZPosition(0);
    double y_origin = ENUOriginInXYZPosition(1);
    double z_origin = ENUOriginInXYZPosition(2);

    double x_target = targetInLocalENUPosition(0);
    double y_target = targetInLocalENUPosition(1);
    double z_target = targetInLocalENUPosition(2);

    
    double dX = C11 * x_target + C21 * y_target+ C31 * z_target;
    double dY = C12 * x_target + C22 * y_target + C32 * z_target;
    double dZ = C23 * y_target + C33 * z_target;

    double X = x_origin + dX;
    double Y = y_origin + dY;
    double Z = z_origin + dZ;

    targetInXYZPosition << X, Y, Z;

}


/**
 * @description: Firstly transform the origin llh[] to geocentric coordinate frame XYZ[],
 *               by function LLH2XYZ(), then tansform the xyz to geocentric coordinate
 *               frame XYZGeo[] using the origin position XYZ[] by function ENU2XYZ(),
 *               finally transform the XYZGeo[] to LLH by function XYZ2LLH(). 
 * @param[in]   targetInLocalENUPosition     position in ENU coordinate frame to be transformed
 * @param[in]   ENUOriginInLLHPosition       position of the origin of ENU coordinate frame in LLH coordinate frame
 * @param[out]  targetInLLHPosition          [latitude, longitude, altitude] after transformation
 * @see LLH2XYZ
 * @see ENU2XYZ
 * @see XYZ2LLH
 */
void singleDronePositionCalculator::ENU2LLH(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                                            Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{  
    Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition;     ///< position of the origin of ENU coordinate frame in geocentric coordinate xyz frame after transformation
    /// transform LLH ENU origin position to geocentric xyz position first.
    LLH2XYZ(ENUOriginInLLHPosition, ENUOriginInXYZPosition);

    Eigen::Matrix<double, 3, 1> targetInXYZPosition;        ///< position in geocentric coordinate xyz frame after transformation
    /// transform target ENU position to geocentric xyz position.
    ENU2XYZ(targetInLocalENUPosition, ENUOriginInXYZPosition, targetInXYZPosition);

    /// transform to target xyz position to LLH position final
    XYZ2LLH(targetInXYZPosition, targetInLLHPosition);
}

void singleDronePositionCalculator::consoleOutput(vector<geodetic_target> geodetic_targets)
{
    //printf("\033[2J");
    //printf("\033[1;1H");
    for(int i = 0; i < geodetic_targets.size(); i++)
    {
        
        Eigen::Matrix<double, 2, 1> target_geodetic_position;
        target_geodetic_position << geodetic_targets[i].position[0], geodetic_targets[i].position[1];

        Eigen::Matrix<double, 2, 1> base_geodetic_position;
        base_geodetic_position << 28.2703016285, 113.038538401; ///< 2020.11.24.rosbag

        /// diatence calculator
        double this_distance;
        double this_angle;
        llhDistanceCalculator(target_geodetic_position, base_geodetic_position, this_distance, this_angle);

        printf("id:%d, name:%s, latitude:%10f, longitude:%10f, altitude:%10f\n", 
                        geodetic_targets[i].id,
                        geodetic_targets[i].Class.c_str(),
                        geodetic_targets[i].position[0], 
                        geodetic_targets[i].position[1],
                        geodetic_targets[i].position[2]);
        printf("Distance to base LAT:%10f LON:%10f -> Dis:%f -> Ang:%f\n", 
                        base_geodetic_position(0), 
                        base_geodetic_position(1), 
                        this_distance,
                        this_angle);
    }
}

void singleDronePositionCalculator::rostopicOutput(vector<geodetic_target> geodetic_targets)
{
    drone_pose_estimation::GroundTargetStates this_ground_target_states;           ///< ground targets state msg
    
    for(int i = 0; i < geodetic_targets.size(); i++)
    {
        drone_pose_estimation::GroundTargetState this_ground_target_state;         ///< this ground target's state
        this_ground_target_state.id = geodetic_targets[i].id;
        this_ground_target_state.probability = geodetic_targets[i].probability;
        this_ground_target_state.Class = geodetic_targets[i].Class;

        this_ground_target_state.global_position.latitude = geodetic_targets[i].position[0];
        this_ground_target_state.global_position.longitude = geodetic_targets[i].position[1];
        this_ground_target_state.global_position.altitude = geodetic_targets[i].position[2];

        this_ground_target_states.ground_targets.push_back(this_ground_target_state);
    }

    ground_target_state_pub_.publish(this_ground_target_states);                 ///< rostopic publish
}

void singleDronePositionCalculator::llhDistanceCalculator(Eigen::Matrix<double, 2, 1> target_position, 
                                                            Eigen::Matrix<double, 2, 1> base_position,
                                                            double& distance,
                                                            double& angle)
{
    double target_latitude = target_position(0);
    double target_longitude = target_position(1);
    double base_latitude = base_position(0);
    double base_longitude = base_position(1);

    double a;
    double b;

    a = target_latitude * deg2rad - base_latitude * deg2rad;
    b = target_longitude * deg2rad - base_longitude * deg2rad;

    double Ea = 6378137;        ///< semi-major axis of the earth ellipsoid
    double Eb = 6356752.3141;   ///< The semi-minor axis of the earth ellipsoid = 6356.7523141 km.

    double ec = Eb + (Ea - Eb) * (90.0 - base_latitude) / 90.0;
    double ed = ec * cos(base_latitude * deg2rad);
    
    /// calcuator distance between two llh point
    double this_distance = sqrt(pow(a * ec, 2)+ pow(b * ed, 2));

    /// calculator bearing angle from base_point to target_point
    double this_angle = atan(fabs((b*ed)/(a*ec))) * rad2deg;
    double dlo = target_longitude - base_longitude;
    double dla = target_latitude - base_latitude;

    /// judge quadrant
    if(dlo > 0 && dla < 0){
        this_angle = (90.0 - this_angle) + 90.0;
    }else if(dlo <= 0 && dla < 0){
        this_angle = this_angle + 180.0;
    }else if(dlo < 0 && dla >= 0){
        this_angle = (90.0 - this_angle) + 270.0;
    }

    distance = this_distance;
    angle = this_angle;

}


int main(int argc, char** argv){
    ros::init(argc, argv, "single_drone_position_calculator_node");
    ros::NodeHandle nh("~");

    singleDronePositionCalculator positionCalculator(nh);

    ros::spin();
    return 0;

}
