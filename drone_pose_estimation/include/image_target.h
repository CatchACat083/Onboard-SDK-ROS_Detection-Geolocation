/*
 * @Author: your name
 * @Date: 2020-11-13 20:06:14
 * @LastEditTime: 2020-11-23 14:22:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/image_target.h
 */

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <vector>
#include <string>
#include <queue>


#include <opencv2/opencv.hpp>

class image_target
{
    public:
        int id;                         ///< target id from darknet classification.
        double probabilities;           ///< target probabilities.
        std::string Class;                   ///< target's class name.

        cv::Point2f img_position;       ///< image position of target, related to top-left of the image. <pixel>
        cv::Point2f dist_img_position;  ///< image position of target after distortion correction. <pixel>

        cv::Point2f img_position_center;        ///< image position of target, related to center of the image. <pixel>
        cv::Point2f sensor_position_center;     ///< sensor position of target, realted to center of the sensor. <m>

    public:
        image_target();
        image_target(int new_id, double new_probabilities, std::string new_class, cv::Point2f new_img_position);

        ~image_target();
};

