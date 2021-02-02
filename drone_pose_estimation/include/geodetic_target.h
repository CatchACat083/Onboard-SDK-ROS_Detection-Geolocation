/*
 * @Author: your name
 * @Date: 2020-11-23 14:49:49
 * @LastEditTime: 2020-11-23 14:50:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/include/geodetic_target.h
 */
#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <queue>

//Matrix include
#include <Eigen/Dense>
#include <math.h>
#include <opencv2/opencv.hpp>

class geodetic_target
{
public:
    /// @brief construct function
    geodetic_target();

    /// @brief construct function
    geodetic_target(int new_id, double new_probabilities, std::string new_class,
                    Eigen::Matrix<double, 3, 1> new_position,
                    Eigen::Matrix<double, 3, 1> new_attitude);

    /// @brief destruct function
    ~geodetic_target();

public:
    int id;                 ///< target id from darknet classification.
    double probability;   ///< target probabilities.
    std::string Class;      ///< target's class name.

    Eigen::Matrix<double, 3, 1> position;     ///< target geodetic position [latitude, longitude, altitude] [rad, rad, m]
    Eigen::Matrix<double, 3, 1> attitude;     ///< target euler angle according to NED coordinate frame [roll, pitch, yaw] [rad, rad, rad]
};
