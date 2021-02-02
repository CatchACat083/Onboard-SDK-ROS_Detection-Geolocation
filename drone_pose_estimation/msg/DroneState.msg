# 2020.08.31 @CatchACat_Lin
# This is the M210 drone state message

# the header of getting this data in the ROS system
Header header

# the messages useful for target position 
sensor_msgs/NavSatFix gps               ### GPS position [Latitude, Longitude, Altitude][degree degree m]
geometry_msgs/Vector3 atti              ### eular Angle  [Roll, Pitch, Yaw][rad]
geometry_msgs/Vector3 gimbal            ### eular Angle  [Roll, Pitch, Yaw][rad]

#geometry_msgs/Pose2D image_position

ImagePosition[] image_positions         ### bounding boxes @reference darknet_ros_msgs

# # the header of getting this data in the ROS system
# Header header

# # the messages useful for target position calculate
# sensor_msgs/NavSatFix flt_gps               # GPS position [Latitude, Longitude, Altitude][degree degree m]
# geometry_msgs/Vector3 flt_attitude          # eular angle of drone [Roll, Pitch, Yaw][rad]
# geometry_msgs/Vector3 flt_gimbal            # eular angle of gimbal [Roll, Pitch, Yaw][rad]

# float64 cam_focal_length                    # focal length of camera        [mm]
# float64 cam_sensor_width                    # camera sensor width           [*1000 mm]
# float64 cam_sensor_height                   # camera sensor height          
# int cam_horizontal_resolution               # camera horizontal resolution  [pixel]   
# int cam_vertical_resulution                 # camera vertical resolution    [pixel]

# geometry_msgs/Point dec_image_position      # Image position (x, y) of one single target         [m]


