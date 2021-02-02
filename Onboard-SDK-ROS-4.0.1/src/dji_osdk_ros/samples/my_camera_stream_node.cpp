/*
 * @Author: your name
 * @Date: 2020-08-26 10:34:48
 * @LastEditTime: 2020-09-24 19:08:25
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \Onboard-SDK-ROS-4.0.1\src\dji_osdk_ros\samples\my_camera_stream_node.cpp
 */

#include "my_camera_stream_node.h"

using namespace dji_osdk_ros;

AVCodecContext*       pCodecCtx;
AVCodec*              pCodec;
AVCodecParserContext* pCodecParserCtx;
SwsContext*           pSwsCtx;
AVFrame* pFrameYUV;
AVFrame* pFrameRGB;
uint8_t* rgbBuf;
size_t   bufSize;

bool camera_imgs_subscribed = false; //flag whether camera images subscribed
dji_osdk_ros::SetupCameraStream setupCameraStream_; //subscription to main camera stream
static const std::string OPENCV_WINDOW = "Image window";

int main(int argc, char** argv){
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nh;

    auto setup_camera_stream_client = nh.serviceClient<dji_osdk_ros::SetupCameraStream>("vehicle_node/setup_camera_stream");
    //auto camera_stream_sub = nh.subscribe("vehicle_node/dji_osdk_ros/main_camera_images", 1, cameraStreamCallBack);

    //ffmpeg_init();

    struct timeval tv;
    struct tm tm;
    gettimeofday(&tv, NULL);
    localtime_r(&tv.tv_sec, &tm);

    
    ///! For signal handling, e.g. if user terminate the program with Ctrl+C
    ///! this program will unsubscribe the image stream if it's subscribed
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = shutDownHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    /// Setup the subscription to main camera stream
    setupCameraStream_.request.cameraType = setupCameraStream_.request.MAIN_CAM;
    setupCameraStream_.request.start        = 1;
    /// Start the subscription the main camera stream
    std::string action;
    if(setupCameraStream_.request.cameraType == setupCameraStream_.request.MAIN_CAM){
        action = "Main_Camera";
    }
    //Call main camera stream service
    if(setup_camera_stream_client.call(setupCameraStream_)){
        if(setupCameraStream_.request.start == 1){
            camera_imgs_subscribed = true;
            ROS_INFO("@@@camera_node:Successfully subscribed to %s images", action.c_str());
        }else{
            camera_imgs_subscribed = false;
            ROS_INFO("@@@camera_node:Successfully unsubscribed to %s images", action.c_str());
        }
    }else{
        ROS_ERROR("@@@camera_node:Failed to %s images", action.c_str());
    }


    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    // ROS_INFO("Wait 10 second to record stream");
    // ros::Duration(10).sleep();


    // ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
    ros::waitForShutdown();

    return 0;
}

void cameraStreamCallBack(const sensor_msgs::Image& msg)
{
    //ROS_INFO("Successfully unsubscribed to camera images");
    // cv_bridge::CvImagePtr cv_ptr;
    // cv::Mat this_img;

    // try{
    //     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    // }

    // catch (cv_bridge::Exception& e){
    //   ROS_ERROR("cv_bridge exception: %s", e.what());
    //   return;
    // }

    // cv_ptr->image.copyTo(this_img);
    
    // std::cout << this_img.size().height << "," <<this_img.size().width <<std::endl;
    // // Update GUI Window
    // cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    // cv::waitKey(1);
}


/**
 * @description: Ctrl+C shutdown Handler, call ros::shutdown();
 * @param  int signum :caught signal
 * @return void 
 */
void shutDownHandler(int signum){
    ROS_INFO("Caught signal %d, Terminating the node...", signum);
    ros::shutdown();
    exit(signum);
}


bool ffmpeg_init()
{
    avcodec_register_all();
    pCodecCtx = avcodec_alloc_context3(NULL);
    if (!pCodecCtx){
        return false;
    }

    pCodecCtx->thread_count = 4;
    pCodec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!pCodec || avcodec_open2(pCodecCtx, pCodec, NULL) < 0){
        return false;
    }

    pCodecParserCtx = av_parser_init(AV_CODEC_ID_H264);
    if (!pCodecParserCtx){
        return false;
    }

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV){
        return false;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB){
        return false;
    }

    pSwsCtx = NULL;

    pCodecCtx->flags2 |= AV_CODEC_FLAG2_SHOW_ALL;
}
