/*
 * @Author: your name
 * @Date: 2020-11-23 11:20:53
 * @LastEditTime: 2020-11-25 11:52:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/geodetic_target.cpp
 */

#include "geodetic_target.h"
/**
 * @description: construct function
 * @param none
 */
geodetic_target::geodetic_target()
{
    id = 0;                 ///< target id from darknet classification.
    probability = 0.0;    ///< target probabilities.
    Class = "";             ///< target's class name.

    position << 0.0, 0.0, 0.0;      ///< target geodetic position [latitude, longitude, altitude] [rad, rad, m]
    attitude << 0.0, 0.0, 0.0;      ///< target euler angle according to NED coordinate frame [roll, pitch, yaw] [rad, rad, rad]
}

/**
 * @description: construct function
 * @param [in]  new_id              target new id
 * @param [in]  new_probabilities   target new probabilities
 * @param [in]  new_position        target new position [latitude, longitude, altitude] [rad, rad, m]
 * @param [in]  new_attitude        target euler angle according to NED coordinate frame [roll, pitch, yaw] [rad, rad, rad]
 */
geodetic_target::geodetic_target(int new_id, double new_probability, std::string new_class,
                                 Eigen::Matrix<double, 3, 1> new_position,
                                 Eigen::Matrix<double, 3, 1> new_attitude)
{
    id = new_id;                            ///< target id from darknet classification.
    probability = new_probability;      ///< target probabilities.
    Class = new_class;                      ///< target's class name.

    position << new_position[0], new_position[1], new_position[2];
    attitude << new_attitude[0], new_attitude[1], new_attitude[2];
}   

/**
 * @description: desturct function
 */
geodetic_target::~geodetic_target()
{
    return;
}
