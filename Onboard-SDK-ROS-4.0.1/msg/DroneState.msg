# 2020.11.12 @CatchACat_Lin
# This is the M210 drone state message

# the header of getting this data in the ROS system
Header header

# the messages useful for target position 
sensor_msgs/NavSatFix gps               ### GPS position [Latitude, Longitude, Altitude][degree degree m]
geometry_msgs/Vector3 atti              ### eular Angle  [Roll, Pitch, Yaw][rad]
geometry_msgs/Vector3 gimbal            ### eular Angle  [Roll, Pitch, Yaw][rad]

#geometry_msgs/Pose2D image_position

ImagePosition[] image_positions         ### bounding boxes @reference darknet_ros_msgs