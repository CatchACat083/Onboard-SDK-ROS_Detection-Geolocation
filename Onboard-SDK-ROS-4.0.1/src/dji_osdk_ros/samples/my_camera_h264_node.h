/*
 * @Author: your name
 * @Date: 2020-08-25 11:06:45
 * @LastEditTime: 2020-08-25 21:38:48
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\include\my_camera_stream_node.h
 */
//INCLUDE
#include <ros/ros.h>
#include <iostream>
#include "chrono"
#include <signal.h>

#include <dji_osdk_ros/SetupCameraH264.h>
#include <sensor_msgs/Image.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}

#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"



bool ffmpeg_init();
void show_rgb(uint8_t *rawData, int height, int width);
void decodeToDisplay(uint8_t *buf, int bufLen);
void cameraH264CallBack(const sensor_msgs::Image& msg);
void shutDownHandler(int signum);
// bool cameraSubscriptionHelper(dji_osdk_ros::SetupCameraH264 &service);