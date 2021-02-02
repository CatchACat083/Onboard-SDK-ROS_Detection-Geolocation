#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>
#include <math.h>

// #include "mat.h"
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include <drone_pose_estimation/GroundTargetState.h>
#include <drone_pose_estimation/GroundTargetStates.h>

using namespace Eigen;
using namespace std;




/**
 * @description:  calculator distancex ros_node between calculator result and ground truth.
 */
class distanceCalculator
{
public:
    /// @brief construction function of distance calculator.
    distanceCalculator(ros::NodeHandle nh);

    /// @brief unconstruction function of distance calculator.
    ~distanceCalculator();

private:
    #define C_PI (double)3.141592653589793

private:
    const float deg2rad = C_PI/180.0;
    const float rad2deg = 180.0/C_PI;

private:

    /// @brief init function of distance calcualtor
    void init();

    /// @brief callback function of ground truth gps position ros_topic @sensor_msgs::NavSatFix.
    void groundtruthCallback(const sensor_msgs::NavSatFix::ConstPtr &ground_msg);

    /// @brief callback function of calculator result gps position ros_topic @drone_pose_estimation::GroundTargetStates.
    void calculatorresultCallback(const drone_pose_estimation::GroundTargetStates::ConstPtr &calculator_msg);

    /// @brief time sync callback function of ground truth and calculator result gps position ros_topic.
    void distanceCallback(const sensor_msgs::NavSatFix::ConstPtr &ground_msg,
                          const drone_pose_estimation::GroundTargetStates::ConstPtr &calculator_msg);

    /// @brief console output function of distance calculator.
    void consoleOutput(drone_pose_estimation::GroundTargetState geodetic_target,  
                       Eigen::Matrix<double, 2, 1> groundtruth_position,
                       double distance,
                       double angle);

    /// @brief calculator distance between two gps points. 
    void llhDistanceCalculator(Eigen::Matrix<double, 2, 1> target_position,
                               Eigen::Matrix<double, 2, 1> base_position,
                               double& distance,
                               double& angle);
    
    /// @brief rostopic output function of distance calculator.
    void rostopicOutput(double distance);

    /// @brief txt file output function of distance calculator.
    void txtFileOutput(double distance);

private:
    ros::NodeHandle nodeHandle_;            ///< ros nodehandle
    ros::Subscriber groundtruthSub_;        ///< subscriber of ground truth gps positon
    ros::Subscriber calculatorSub_;         ///< subscribet of calculator result

    ros::Publisher distancePub_;            ///< publisher of distance result

    sensor_msgs::NavSatFix current_ground_gps_;      ///< ground truth gps postion.

    /// ros_param
    int target_id_;                                 ///< ros_param: id of target which needs to calculator distance.

    bool enable_console_output_;                    ///< ros_param: flag of enable console output
    bool enable_rostopic_output_;                   ///< ros_param: flag of enable ros_topic output
    bool enable_txt_file_output_;                   ///< ros_param: flag of enable txtfile output

    bool enable_stable_groundtruth_;                ///< ros_param: flag of usage of static ground truth position
    double stable_groundtruth_latitude_;            ///< ros_param: static ground truth position latitude
    double stable_groundtruth_longitude_;           ///< ros_param: static ground truth position longitude

    string calculator_result_string_;               ///< ros_param: ros_topic name of calculator result subscriber
    string ground_truth_string_;                    ///< ros_param: ros_topic name of ground truth subscriber
    string distanve_string_;                        ///< ros_param: ros_topic name of distance publisher
    
    /// txt file output
    std::ofstream output_txt_file_;                 ///< output txt file
    string txt_file_name_;                          ///< ros_param: name string of output txt file

};

/**
 * @description: construction function of disatence calculator node.
 * @param  [in]     ros::NodeHandle nh    
 */
distanceCalculator::distanceCalculator(ros::NodeHandle nh)
    : nodeHandle_(nh)
{
    ROS_INFO("distance_calculator_node start");
    /// system parameters initilzation.
    nodeHandle_.param("enable_console_output", enable_console_output_, true);
    nodeHandle_.param("enable_rostpoic_output", enable_rostopic_output_, true);
    nodeHandle_.param("enable_txt_file_output", enable_txt_file_output_, true);

    nodeHandle_.param("enable_stable_groundtruth", enable_stable_groundtruth_, false);
    nodeHandle_.param("stable_groundtruth_latitude", stable_groundtruth_latitude_, 0.0);
    nodeHandle_.param("stable_groundtruth_longitude", stable_groundtruth_longitude_, 0.0);

    nodeHandle_.param<string>("calculator_result_ros_topic", calculator_result_string_, "/ground_01/dgps_node/dgps_fix_position");
    nodeHandle_.param<string>("ground_truth_ros_topic", ground_truth_string_, "/drone_pose_estimation/ground_target_state");
    nodeHandle_.param<string>("distance_ros_topic", distanve_string_, "distance");
    nodeHandle_.param<string>("txt_file_name", txt_file_name_, "distance_txt.txt");
    nodeHandle_.param("target_id", target_id_, 1);

    output_txt_file_.open(txt_file_name_);              ///< creat a new txt for result record
    output_txt_file_.setf(ios::fixed, ios::floatfield);  
    output_txt_file_.precision(9);                      ///< set the decimal digits of the result record

    init();
}

/**
 * @description: unconstruction function of disatence calculator node.  
 */
distanceCalculator::~distanceCalculator()
{
    ROS_INFO("distance_calculator_node stop");
    output_txt_file_.close();
}

/**
 * @description: init function of disatence calculator node. Defination of ros_subscriber and ros_publisher  
 */
void distanceCalculator::init()
{
    ROS_INFO("distance_calculator_node init");
    // groundtruthSub_ = nodeHandle_.subscribe(ground_truth_string_, 1, &distanceCalculator::groundtruthCallback, this);
    // calculatorSub_ = nodeHandle_.subscribe(calculator_result_string_, 1, &distanceCalculator::calculatorresultCallback, this);

    /// if usage of stable ground truth position
    if(enable_stable_groundtruth_)
    {
        ROS_INFO("using stable ground truth global position: %f, %f", stable_groundtruth_latitude_, stable_groundtruth_longitude_);

        calculatorSub_ = nodeHandle_.subscribe(calculator_result_string_, 1, &distanceCalculator::calculatorresultCallback, this);
    }
    else    /// if usage of ground truth ros_topic, using time_sync function
    {
        message_filters::Subscriber<sensor_msgs::NavSatFix>* groundtruth_sub;
        groundtruth_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix>(nodeHandle_, ground_truth_string_, 1); 
        message_filters::Subscriber<drone_pose_estimation::GroundTargetStates>* calculator_sub;
        calculator_sub = new message_filters::Subscriber<drone_pose_estimation::GroundTargetStates>(nodeHandle_, calculator_result_string_, 1);
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, 
                                                                drone_pose_estimation::GroundTargetStates> topic_sync_policy;
        message_filters::Synchronizer<topic_sync_policy>* topic_synchronizer;
        topic_synchronizer = new message_filters::Synchronizer<topic_sync_policy> (topic_sync_policy(1000), *groundtruth_sub, *calculator_sub);
        topic_synchronizer->registerCallback(boost::bind(&distanceCalculator::distanceCallback, this, _1, _2));

    }

    /// define distance ros_publisher
    distancePub_ = nodeHandle_.advertise<std_msgs::Float64>(distanve_string_, 10);
}


/**
 * @description: callback function of target's position ground truth.
 * @param  [in] &ground_msg    LLH position (sensor_msgs::NavSatFix) of the ground target
 */
void distanceCalculator::groundtruthCallback(const sensor_msgs::NavSatFix::ConstPtr &ground_msg)
{
    current_ground_gps_ = *ground_msg;
}

/**
 * @description: callback function of calculate ground target result, print the distance between ground 
 *               truth and calculator result.
 * @param  [in] &calculator_msg     drone_pose_estimation::GroundTargetStates message
 */
void distanceCalculator::calculatorresultCallback(const drone_pose_estimation::GroundTargetStates::ConstPtr &calculator_msg)
{
    drone_pose_estimation::GroundTargetStates this_calculator_msg;      ///< calculator result of ground targets.
    this_calculator_msg = *calculator_msg;

    // for(int i = 0; i < this_calculator_msg.ground_targets.size(); i++)
    // {
    //     sensor_msgs::NavSatFix current_calculator_gps = this_calculator_msg.ground_targets[i].global_position;
    //     /// diatence calculator
    //     double this_distance;                                           ///< distance between real ground target and calculation result.
    //     double this_angle;                                              ///< angle between real ground target and calculation result.

    //     Eigen::Matrix<double, 2, 1> groundInLLHPosition;                ///< real RTK-GPS LL matrix of ground target.
    //     Eigen::Matrix<double, 2, 1> calcuatorInLLHPosition;             ///< calculator RTK-GPS LL matrix of ground target.

    //     groundInLLHPosition << current_ground_gps_.latitude, current_ground_gps_.longitude;
    //     calcuatorInLLHPosition << current_calculator_gps.latitude, current_calculator_gps.longitude;

    //     llhDistanceCalculator(calcuatorInLLHPosition, groundInLLHPosition, this_distance, this_angle);

    //     if(enable_console_output_){
    //         consoleOutput(this_calculator_msg.ground_targets[i], groundInLLHPosition, this_distance, this_angle);
    //     }
    //     if(enable_rostopic_output_){
    //         rostopicOutput(this_distance);
    //     }
    //     if(enable_txt_file_output_){
    //         txtFileOutput(this_distance);
    //     }

    // }


    sensor_msgs::NavSatFix current_calculator_gps = this_calculator_msg.ground_targets[0].global_position;

    double this_distance;                                           ///< distance between real ground target and calculation result.
    double this_angle;                                              ///< angle between real ground target and calculation result.

    Eigen::Matrix<double, 2, 1> groundInLLHPosition;                ///< real RTK-GPS LL matrix of ground target.
    Eigen::Matrix<double, 2, 1> calcuatorInLLHPosition;             ///< calculator RTK-GPS LL matrix of ground target.

    /// if usage of stable ground truth positon
    if(enable_stable_groundtruth_)       
    {
        groundInLLHPosition << stable_groundtruth_latitude_, stable_groundtruth_longitude_;
    }else
    {
        groundInLLHPosition << current_ground_gps_.latitude, current_ground_gps_.longitude;
    }
    
    calcuatorInLLHPosition << current_calculator_gps.latitude, current_calculator_gps.longitude;

    /// start distance calculator.
    llhDistanceCalculator(calcuatorInLLHPosition, groundInLLHPosition, this_distance, this_angle);

    /// define outputs function.
    if(enable_console_output_){
        consoleOutput(this_calculator_msg.ground_targets[0], groundInLLHPosition, this_distance, this_angle);
    }
    if(enable_rostopic_output_){
        rostopicOutput(this_distance);
    }
    if(enable_txt_file_output_){
        txtFileOutput(this_distance);
    }

}

/**
 * @description: time sync callback function of calculate ground target result and ground truth gps position.
 * @param  [in] &ground_msg         sensor_msgs::NavSatFix message
 * @param  [in] &calculator_msg     drone_pose_estimation::GroundTargetStates message
 */
void distanceCalculator::distanceCallback(const sensor_msgs::NavSatFix::ConstPtr &ground_msg,
                                          const drone_pose_estimation::GroundTargetStates::ConstPtr &calculator_msg)
{
    sensor_msgs::NavSatFix current_ground_gps = *ground_msg;            ///< ground truth gps position.
    drone_pose_estimation::GroundTargetStates this_calculator_msg;      ///< calculator result of ground targets.
    drone_pose_estimation::GroundTargetState this_calculator_target;    ///< calculator result of one single ground target.
    this_calculator_msg = *calculator_msg;

    // for(int i = 0; i < this_calculator_msg.ground_targets.size(); i++)
    // {
    //     sensor_msgs::NavSatFix current_calculator_gps = this_calculator_msg.ground_targets[i].global_position;
    //     /// diatence calculator
    //     double this_distance;                                           ///< distance between real ground target and calculation result.
    //     double this_angle;                                              ///< angle between real ground target and calculation result.

    //     Eigen::Matrix<double, 2, 1> groundInLLHPosition;                ///< real RTK-GPS LL matrix of ground target.
    //     Eigen::Matrix<double, 2, 1> calcuatorInLLHPosition;             ///< calculator RTK-GPS LL matrix of ground target.

    //     groundInLLHPosition << current_ground_gps_.latitude, current_ground_gps_.longitude;
    //     calcuatorInLLHPosition << current_calculator_gps.latitude, current_calculator_gps.longitude;

    //     llhDistanceCalculator(calcuatorInLLHPosition, groundInLLHPosition, this_distance, this_angle);

    //     if(enable_console_output_){
    //         consoleOutput(this_calculator_msg.ground_targets[i], groundInLLHPosition, this_distance, this_angle);
    //     }
    //     if(enable_rostopic_output_){
    //         rostopicOutput(this_distance);
    //     }
    //     if(enable_txt_file_output_){
    //         txtFileOutput(this_distance);
    //     }

    // }
    sensor_msgs::NavSatFix current_calculator_gps;      ///< current positon of calculator result.
    current_calculator_gps.latitude =  -1.0;

    /// if calculator result is NULL.
    if (this_calculator_msg.ground_targets.size() < 1) return;

    /// find calculator result related to target_id_.
    for(int i = 0; i < this_calculator_msg.ground_targets.size(); i++)
    {
        if (this_calculator_msg.ground_targets[i].id == target_id_){
            current_calculator_gps = this_calculator_msg.ground_targets[i].global_position;
            this_calculator_target = this_calculator_msg.ground_targets[i];
        }
    }

    /// if failed to find target in need.
    if(current_calculator_gps.latitude == -1.0) return;
    
    /// define distance calculator.
    double this_distance;                                           ///< distance between real ground target and calculation result.
    double this_angle;                                              ///< angle between real ground target and calculation result.

    Eigen::Matrix<double, 2, 1> groundInLLHPosition;                ///< real RTK-GPS LL matrix of ground target.
    Eigen::Matrix<double, 2, 1> calcuatorInLLHPosition;             ///< calculator RTK-GPS LL matrix of ground target.

    groundInLLHPosition << current_ground_gps.latitude, current_ground_gps.longitude;
    calcuatorInLLHPosition << current_calculator_gps.latitude, current_calculator_gps.longitude;

    /// start distance calculator.
    llhDistanceCalculator(calcuatorInLLHPosition, groundInLLHPosition, this_distance, this_angle);

    /// define outputs.
    if(enable_console_output_){
        consoleOutput(this_calculator_target, groundInLLHPosition, this_distance, this_angle);
    }
    if(enable_rostopic_output_){
        rostopicOutput(this_distance);
    }
    if(enable_txt_file_output_){
        txtFileOutput(this_distance);
    }

    
}


/**
 * @description: distance calculator function of two gps position.
 * @param  [in]     target_position         target gps position.
 * @param  [in]     base_position           base gps position.
 * @param  [out]    &distance               calculator result of distance between target and base.
 * @param  [out]    &angle                  calculator result of bearing angle from base to target.
 */
void distanceCalculator::llhDistanceCalculator(Eigen::Matrix<double, 2, 1> target_position,
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


/**
 * @description: console output function of calculator result.
 * @param  [in]     geodetic_target         ground target state.
 * @param  [in]     groundtruth_position    ground truth gps position.
 * @param  [in]     distance                calculator result of distance between ground target and ground truth.
 * @param  [in]     angle                   calculator result of bearing angle from ground truth to  ground target.
 */
void distanceCalculator::consoleOutput(drone_pose_estimation::GroundTargetState geodetic_target,  
                                       Eigen::Matrix<double, 2, 1> groundtruth_position,
                                       double distance,
                                       double angle)
{
    /// define consule output format.
    printf("\033[2J");
    printf("\033[1;1H");

    /// ground target states
    printf("id:%d, name:%s, latitude:%10f, longitude:%10f, altitude:%10f\n", 
                geodetic_target.id,
                geodetic_target.Class.c_str(),
                geodetic_target.global_position.latitude, 
                geodetic_target.global_position.longitude,
                geodetic_target.global_position.altitude);
    /// ground truth state and distance and bearing angle
    printf("Distance to base LAT:%10f LON:%10f -> Dis:%f -> Ang:%f\n", 
            groundtruth_position(0), 
            groundtruth_position(1), 
            distance,
            angle);
}

/**
 * @description: rostopic output function of calculator distance result.
 * @param  [in]     distance                calculator result of distance between ground target and ground truth.
 */
void distanceCalculator::rostopicOutput(double distance)
{
    printf("publish one distance rostopic success");

    std_msgs::Float64 this_pub_distance;    ///< distance message data 
    this_pub_distance.data = distance;

    distancePub_.publish(this_pub_distance);
}


/**
 * @description: txt output function of calculator distance result.
 * @param  [in]     distance                calculator result of distance between ground target and ground truth.
 */
void distanceCalculator::txtFileOutput(double distance)
{
    output_txt_file_ << distance << endl;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "distance_calculator_node");
    ros::NodeHandle nh("~");

    distanceCalculator calculator(nh);

    ros::spin();
    return 0;
}