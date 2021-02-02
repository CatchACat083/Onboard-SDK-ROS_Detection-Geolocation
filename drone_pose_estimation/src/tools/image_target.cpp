/*
 * @Author: your name
 * @Date: 2020-11-13 20:06:03
 * @LastEditTime: 2020-11-16 21:01:10
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/image_target.cpp
 */

#include "image_target.h"


image_target::image_target()
{
    id = 0;
    probabilities = 0.0;
    Class = "";

    img_position = cv::Point2f(0.0,0.0);
    dist_img_position = cv::Point2f(0.0,0.0);

    img_position_center = cv::Point2f(0.0,0.0);
    sensor_position_center = cv::Point2f(0.0,0.0);
}

image_target::image_target(int new_id, double new_probabilities, std::string new_class, cv::Point2f new_img_position)
{
    id = new_id;
    probabilities = new_probabilities;
    Class = new_class;
    img_position = new_img_position;

    dist_img_position = cv::Point2f(0.0,0.0);

    img_position_center = cv::Point2f(0.0,0.0);
    sensor_position_center = cv::Point2f(0.0,0.0);
}

image_target::~image_target()
{

}


