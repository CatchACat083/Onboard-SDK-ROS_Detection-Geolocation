# 2020.08.31 @CatchACat_Lin
# This is the M210 drone state message

# the header of getting this data in the ROS system
Header header

# the messages useful for target position 
sensor_msgs/NavSatFix gps               # GPS position [Latitude, Longitude, Altitude][degree degree m]
geometry_msgs/Vector3 atti              # Eular Angle  [Roll, Pitch, Yaw][rad]
geometry_msgs/Vector3 gimbal            # Eular Angle  [Roll, Pitch, Yaw][rad]

#geometry_msgs/Pose2D image_position

geometry_msgs/Point image_position    # Image position [x, y][m]
float64 focal_length                    # Focal length of camera [m]


