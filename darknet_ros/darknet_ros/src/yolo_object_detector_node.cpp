/*
 * @Author: your name
 * @Date: 2020-09-08 11:21:01
 * @LastEditTime: 2020-09-18 11:07:07
 * @LastEditors: your name
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/darknet_ros/darknet_ros/src/yolo_object_detector_node.cpp
 */
/*
 * yolo_obstacle_detector_node.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: Marko Bjelonic
 *   Institute: ETH Zurich, Robotic Systems Lab
 */

#include <ros/ros.h>
#include <darknet_ros/YoloObjectDetector.hpp>

//-------------- DJI OSDK ROS Camera -----------------//

int main(int argc, char** argv) {
  ros::init(argc, argv, "darknet_ros");
  ros::NodeHandle nodeHandle("~");

  darknet_ros::YoloObjectDetector yoloObjectDetector(nodeHandle);

  ros::spin();
  return 0;
}
