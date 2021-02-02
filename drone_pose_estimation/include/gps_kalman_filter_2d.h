/*
 * @Author: your name
 * @Date: 2020-11-23 10:36:24
 * @LastEditTime: 2020-11-27 16:13:12
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/include/gps_kalman_filter_2d.h
 */

#define STATE_DIM 4       // 状态维度 state_dimension
#define OBSERVE_DIM 2     // 观测维度 observe_dimension
#define CONTROL_DIM 0     // 控制维度 control_dimension

// #define C_PI (double)3.141592653589793
// const float deg2rad = C_PI/180.0;
// const float rad2deg = 180.0/C_PI;

#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>

/// @brief Create a GPS filter that only tracks two dimensions of position and velocity.
/// The inherent assumption is that changes in velocity are randomly distributed around 0.
/// The state model has four dimensions: x, y, x', y'
/// Each time step we can only observe position, not velocity, so the observation vector has only two dimensions.
class GPSKalmanFilter2d
{
public:
    /// @brief constructed function    
    GPSKalmanFilter2d();

    /// @brief desturcted function
    ~GPSKalmanFilter2d();

    /// @brief define measurement module
    double KalmanMeasure(const double measure);

    /// @brief initialize the filter
    void initKalmanFilter(const double latitude, const double longitude, const double time_stamp_now);

    /// @brief define process module
    void processKalmanFilter(const double latitude, double longitude, const double time_stamp_now);

    /// @brief define process module, with control parameter
    void processKalmanFilter(const double latitude, const double longitude, const double time_stamp_now, const double control);
    
    /// @brief get latitude
    double getLatitude();

    /// @brief get longitude
    double getLongitude();

    /// @brief get velocity in latitude axis
    double getLatVelocity();

    /// @brief get velocity in longitude axis
    double getLonVelocity();

    /// @brief get target bearing
    double getBearing();

    /// @brief get speed
    double getSpeed(double altitude);

public:
    /// This group of matrices must be specified by the user.
    Eigen::MatrixXd A_{STATE_DIM,STATE_DIM};        ///< 状态转移矩阵 state_transition 'A_k' [n*n].
    Eigen::MatrixXd B_{STATE_DIM,CONTROL_DIM};      ///< 控制矩阵 control_matrix 'B_k'.
    Eigen::MatrixXd H_{OBSERVE_DIM,STATE_DIM};      ///< 状态与观测的关系 observation_model 'H_k' [m*n].
    Eigen::MatrixXd Q_{STATE_DIM,STATE_DIM};        ///< 预测值噪声，是叠加在协方差矩阵上的 process_noise_covariance 'Q_k' [n*n].
    Eigen::MatrixXd R_{OBSERVE_DIM,OBSERVE_DIM};    ///< 观测值噪声 observation_noise_covariance 'R_k' [m*m].

    /// The observation is modified by the user before every time step.
    Eigen::MatrixXd z_{OBSERVE_DIM, 1};             ///< 观测值 observation 'z_k' [m*1].

    /// This group of matrices are updated every time step by the filter.
    Eigen::MatrixXd x_raw_{STATE_DIM, 1};           ///< 状态的预测值 predicted_state 'x-hat_k|k-1' [n*1].
                                                    ///< [latitude, longitude, delta_lat, delta_lon].
    Eigen::MatrixXd p_raw_{STATE_DIM, STATE_DIM};   ///< 状态预测的协方差 predicted_estimate_covariance 'p_k|k-1' [n*n].
    Eigen::MatrixXd u_{CONTROL_DIM, 1};             ///< 控制量 control.
    Eigen::MatrixXd K_{STATE_DIM, OBSERVE_DIM};     ///< 卡尔曼增益 optimal_gain 'K_k' [n*m].
    Eigen::MatrixXd x_{STATE_DIM, 1};               ///< 状态的估计值 state_estimate 'x-hat_k|k' [n*1].
    Eigen::MatrixXd p_{STATE_DIM, STATE_DIM};       ///< 状态估计的协方差 estimate_covariance 'p_k|k' [n*n].


    double time_stamp_;         ///< time stamp now (in secs).

    double delta_t_;            ///< delta t

    double filter_invid_time_;   ///< max interval time of filter between neighboring estimation. 
                                ///< if interval of neighboring estimation is beyond this variable, this filter need to be re-initialization.
};
