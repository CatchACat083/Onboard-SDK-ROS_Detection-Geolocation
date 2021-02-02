/*
 * @Author: your name
 * @Date: 2020-08-26 10:35:01
 * @LastEditTime: 2020-08-26 11:14:38
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\samples\my_camera_stream_node.h
 */
// System includes
#include <ros/ros.h>
#include <iostream>
#include "chrono"
#include <signal.h>

// DJI SDK includes
#include "dji_osdk_ros/SetupCameraStream.h"

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

extern "C"{
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}


//Opencv includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//Image encoding includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void cameraStreamCallBack(const sensor_msgs::Image& msg);
void shutDownHandler(int signum);
bool ffmpeg_init();