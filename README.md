# OSDK-ROS_targetGeolocation
## 1. 这是什么？ 

One Demo Targets Detection and Geolocation With DJI@OSDK-ROS and darknet-ROS 

一个地面目标检测+定位的Demo程序，用到了大疆的OSDK-ROS和darknet-ROS的yolov4-tiny 

## 2. 如何使用？ 

### 2.1. 硬件描述 

（1）无人机与云台相机：大疆DJI 经纬Matrice 200 V2 + 禅思Zenmuse X5S + DJI MFT 15mm/1.7 ASPH 

（2）上位机：NVIDIA Jetson TX2 

（3）GNSS：Emlid Reach M+ 

### 2.2. 系统与软件描述 

（1）系统描述（具体详见Jetson_TX2说明文档） 
- tx2_L4t_2821 
- Ubuntu16.04 
- Jetpack3.3 

（2）环境描述（具体详见DJI_OSDK说明文档） 
- GCC 5.4.0 
- Cmake 3.5.1 
- OpenCV 3.3.1 
- OpenCV Contrib 3.3.1 
- CUDA 9.0 
- ffmpeg  2.8.15 
- libusb-1.0-0-dev 
- ROS-Kientic + catkin 
- Nmea-commons 

### 2.3. 代码的编译与运行 
（1）将代码移入`~/catkin_ws/src `

 ` $ cd catkin_ws `
 
  `$ catkin_make` 
  
（2）将darknet目标检测的.weights文件和.cfg文件分别放入

  `~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/weights`
  
  `~/catkin_ws/src/darknet_ros/darknet_ros/yolo_network_config/cfg `
  
同时修改`~/catkin_ws/src/darknet_ros/darknet_ros/config/yolov4-tiny.yaml`中的cfg、weights、threshold、classname。 

（3）正确连接并开启无人机，启动dji_osdk_ros节点 

`  $ roslaunch dji_osdk_ros my_vehicle_node.launch `

启动定位运算节点 

  `$ roslaunch drone_pose_estimation single_drone_info_sync_node.launch `
  
`  $ roslaunch drone_pose_estimation single_drone_position_calculator.launch `

## 3.感谢 

https://github.com/dji-sdk/Onboard-SDK-ROS 

https://github.com/leggedrobotics/darknet_ros 
