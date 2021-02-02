#include "my_camera_h264_node.h"


//CODE
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
dji_osdk_ros::SetupCameraH264 setupCameraH264_; //subscription to main camera stream


int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_camera_h264_node");
    ros::NodeHandle nh;

    auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
    auto camera_h264_sub = nh.subscribe("dji_osdk_ros/camera_h264_stream", 10, cameraH264CallBack);

    ffmpeg_init();

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
    setupCameraH264_.request.request_view = setupCameraH264_.request.MAIN_CAMERA;
    setupCameraH264_.request.start        = 1;
    /// Start the subscription the main camera stream
    std::string action;
    if(setupCameraH264_.request.request_view == setupCameraH264_.request.MAIN_CAMERA){
        action = "Main_Camera";
    }
    //Call main camera stream service
    if(setup_camera_h264_client.call(setupCameraH264_)){
        if(setupCameraH264_.request.start == 1){
            camera_imgs_subscribed = true;
            ROS_INFO("Successfully subscribed to %s images", action.c_str());
        }else{
            camera_imgs_subscribed = false;
            ROS_INFO("Successfully unsubscribed to %s images", action.c_str());
        }
    }else{
        ROS_ERROR("Failed to %s images", action.c_str());
    }


    ros::AsyncSpinner spinner(1);
    spinner.start();
    // ROS_INFO("Wait 10 second to record stream");
    // ros::Duration(10).sleep();


    // ROS_INFO_STREAM("Finished. Press CTRL-C to terminate the node");
    ros::waitForShutdown();

    return 0;
}

/**
 * @description: Call @DJI_OSDK 'dji_osdk_ros::SetupCameraH264' serviceClient for camera image sunscription
 * @param dji_osdk_ros::SetupCameraH264 &service
 * @return bool 
 */
// bool cameraSubscriptionHelper(dji_osdk_ros::SetupCameraH264 &service, ros::NodeHandle &nh){
//     std::string action;
//     if(service.request.request_view == setupCameraH264_.request.MAIN_CAMERA){
//         action = "Main_Camera";
//     }

//     //Call main camera stream service
//     auto setup_camera_h264_client = nh.serviceClient<dji_osdk_ros::SetupCameraH264>("setup_camera_h264");
//     if(setup_camera_h264_client.call(setupCameraH264_)){
//         if(service.request.start == 1){
//             camera_imgs_subscribed = true;
//             ROS_INFO("Successfully subscribed to %s images", action.c_str());
//         }else{
//             camera_imgs_subscribed = false;
//             ROS_INFO("Successfully unsubscribed to %s images", action.c_str());
//         }
//     }else{
//         ROS_ERROR("Failed to %s images", action.c_str());
//         return false;
//     }

//     return true;

// }

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

void show_rgb(uint8_t *rawData, int height, int width)
{
    cv::Mat mat(height, width, CV_8UC3, rawData, width*3);
    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
    cv::imshow("camera_stream_node", mat);
    cv::waitKey(1);
}

void decodeToDisplay(uint8_t *buf, int bufLen)
{
    uint8_t* pData   = buf;
    int remainingLen = bufLen;
    int processedLen = 0;

    AVPacket pkt;
    av_init_packet(&pkt);
    while (remainingLen > 0)
    {
        processedLen = av_parser_parse2(pCodecParserCtx, pCodecCtx,
                                        &pkt.data, &pkt.size,
                                        pData, remainingLen,
                                        AV_NOPTS_VALUE, AV_NOPTS_VALUE, AV_NOPTS_VALUE);
        remainingLen -= processedLen;
        pData        += processedLen;

        if (pkt.size > 0)
        {
            int gotPicture = 0;
            avcodec_decode_video2(pCodecCtx, pFrameYUV, &gotPicture, &pkt);

            if (!gotPicture)
            {
                //DSTATUS_PRIVATE("Got Frame, but no picture\n");
                continue;
            }
            else
            {
                int w = pFrameYUV->width;
                int h = pFrameYUV->height;
                //DSTATUS_PRIVATE("Got picture! size=%dx%d\n", w, h);

                if(NULL == pSwsCtx)
                {
                    pSwsCtx = sws_getContext(w, h, pCodecCtx->pix_fmt,
                                             w, h, AV_PIX_FMT_RGB24,
                                             4, NULL, NULL, NULL);
                }

                if(NULL == rgbBuf)
                {
                    bufSize = avpicture_get_size(AV_PIX_FMT_RGB24, w, h);
                    rgbBuf = (uint8_t*) av_malloc(bufSize);
                    avpicture_fill((AVPicture*)pFrameRGB, rgbBuf, AV_PIX_FMT_RGB24, w, h);
                }

                if(NULL != pSwsCtx && NULL != rgbBuf)
                {
                    sws_scale(pSwsCtx,
                              (uint8_t const *const *) pFrameYUV->data, pFrameYUV->linesize, 0, pFrameYUV->height,
                              pFrameRGB->data, pFrameRGB->linesize);

                    pFrameRGB->height = h;
                    pFrameRGB->width = w;


                    cv::Mat mat(pFrameRGB->height, pFrameRGB->width, CV_8UC3, pFrameRGB->data[0], pFrameRGB->width * 3);
                    cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
                    cv::imshow("camera_stream_node", mat);
                    cv::waitKey(1);

                }
            }
        }
    }
    av_free_packet(&pkt);
}


void cameraH264CallBack(const sensor_msgs::Image& msg)
{
  decodeToDisplay((uint8_t *)&msg.data[0], msg.data.size());
}

