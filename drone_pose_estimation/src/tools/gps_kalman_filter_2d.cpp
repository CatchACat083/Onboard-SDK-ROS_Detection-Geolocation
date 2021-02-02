/*
 * @Author: your name
 * @Date: 2020-11-23 10:35:02
 * @LastEditTime: 2020-11-27 16:17:41
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /catkin_ros/src/drone_pose_estimation/src/gps_kalman_filter.cpp
 */

#include "gps_kalman_filter_2d.h"


#define C_PI (double)3.141592653589793
const float deg2rad = C_PI/180.0;
const float rad2deg = 180.0/C_PI;
/**
 * @description: Create a GPS filter that only tracks two dimensions of position and
 *               velocity. The inherent assumption is that changes in velocity are
 *               randomly distributed around 0.
 * @param none
 */
GPSKalmanFilter2d::GPSKalmanFilter2d(){
    /// Noise is a parameter you can use to alter the expected noise.
    /// 1.0 is the original, and the higher it is, the more a  path will be "smoothed".
    double noise = 1.0;

    /// Assuming the axes are rectilinear does not work well at the poles,
    /// but it has the bonus that we don't need to convert between lat/long
    /// and more rectangular coordinates. The slight inaccuracy of our
    /// physics model is not too important.
    A_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);

    B_ = Eigen::MatrixXd::Constant(STATE_DIM, CONTROL_DIM, 0);

    /// We observe (x, y) in each time step
    H_ = Eigen::MatrixXd::Identity(OBSERVE_DIM, STATE_DIM);

    /// Noise in the world.
    double pos = 0.000001;
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
    Q_(0,0) = Q_(0,0) * pos;
    Q_(1,1) = Q_(1,1) * pos;

    /// Noise in our observation
    R_ = Eigen::MatrixXd::Identity(OBSERVE_DIM, OBSERVE_DIM) * pos * noise;

    /// Control.
    u_ = Eigen::MatrixXd::Constant(CONTROL_DIM, 1, 0);

    /// The start position is totally unknown, so give a high variance
    x_ = Eigen::MatrixXd::Constant(STATE_DIM, 1, 0);
    p_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 1000.0 * 1000.0;

    /// Set time stamp to negitive value
    time_stamp_ = -1.0;
    delta_t_ = -1.0;

    /// set max interval time of filter between neighboring estimation. 
    filter_invid_time_ = 20.0;

}

/**
 * @description: Destruct function
 * @param none
 */
GPSKalmanFilter2d::~GPSKalmanFilter2d() {

}

/**
 * @description: Initialise kalman filter with current values
 * @param [in]  latitude        latitude in degree
 * @param [in]  longitude       longitude in degree
 * @param [in]  time_stamp_now  time stamp now in secs
 */
void GPSKalmanFilter2d::initKalmanFilter(const double latitude, const double longitude, const double time_stamp_now){
    z_(0,0) = latitude * 1000.0;
    z_(1,0) = longitude * 1000.0;

    x_<< latitude * 1000.0,
            longitude * 1000.0,
            0.0,
            0.0;

    time_stamp_ = time_stamp_now;
}

/**
 * @description:    Runs one timestep of prediction + estimation.
 *                  Before each time step of running this, set f.observation to be the nexttime step's observation.
 *                  Before the first step, define the model by setting: f.state_transition f.observation_model f.process_noise_covariance f.observation_noise_covariance
 *                  It is also advisable to initialize with reasonable guesses for f.state_estimate f.estimate_covariance
 * @param [in]  latitude        latitude in degree
 * @param [in]  longitude       longitude in degree
 * @param [in]  time_stamp_now  time stamp now in secs
 */
void GPSKalmanFilter2d::processKalmanFilter(const double latitude, const double longitude, const double time_stamp_now) {
    if (time_stamp_ < 0)
    {
        /// if time_stamp_ < 0, object is unitialised, so initialise with current values
        initKalmanFilter(latitude, longitude, time_stamp_now);
    }
    else if (time_stamp_now - time_stamp_ > filter_invid_time_)
    {
        /// if delta_time > filter_invid_time_, filter becomes invalid, reinitialise the filter.
        initKalmanFilter(latitude, longitude, time_stamp_now);
    }
    else
    {
        /// else apply kalman filter.
        z_(0,0) = latitude * 1000.0;
        z_(1,0) = longitude * 1000.0;

        ///  set the seconds per timestep in the velocity2d model.
        ///  The position units are in thousandths of latitude and longitude. The velocity units are in thousandths of position units per second.
        ///  So if there is one second per timestep, a velocity of 1 will change the lat or long by 1 after a million timesteps.
        ///  Thus a typical position is hundreds of thousands of units. A typical velocity is maybe ten.
        
        delta_t_ = time_stamp_now - time_stamp_;
        //std::cout <<"delta_t" << delta_t_ << std::endl;
        //std::cout <<"time_stamp_now"<< time_stamp_now << std::endl;
        //std::cout <<"time_Stamp"<< time_stamp_<< std::endl;

        if(delta_t_ > 0.0){
            /// time has moved on, so the uncertainty in the current position increases
            double unit_scaler = 0.001;                         /// unit_scaler accounts for the relation between position and velocity units.
            A_(0,2) = unit_scaler * delta_t_;
            A_(1,3) = unit_scaler * delta_t_;

            time_stamp_ = time_stamp_now;
        }
        //--------- Predictive ----------//
        /// predict the state
        x_raw_ = A_ * x_;
        /// predict the state estimate covariance
        p_raw_ = A_ * p_ * A_.transpose() + Q_;

        // -------- Estimation ----------//
        /// calculate the optimal Kalman gain.
        K_ = p_raw_ * H_.transpose() * (H_ * p_raw_ * H_.transpose() + R_).inverse();
        /// estimate the state
        x_ = x_raw_ + K_ * (z_ - H_ * x_raw_);
        /// estimate the state covariance
        p_ = (Eigen::MatrixXd::Identity(STATE_DIM,STATE_DIM) - K_ * H_) * p_raw_;
    }
}

/**
 * @description:    Runs one timestep of prediction + estimation. [INCLUDING CONTROL]
 *                  Before each time step of running this, set f.observation to be the nexttime step's observation.
 *                  Before the first step, define the model by setting: f.state_transition f.observation_model f.process_noise_covariance f.observation_noise_covariance
 *                  It is also advisable to initialize with reasonable guesses for f.state_estimate f.estimate_covariance
 * @param [in]  latitude        latitude in degree
 * @param [in]  longitude       longitude in degree
 * @param [in]  time_stamp_now  time stamp now in secs
 * @param [in]  control         control
 */
void GPSKalmanFilter2d::processKalmanFilter(const double latitude, const double longitude, const double time_stamp_now, const double control = -1.0) {
    if (time_stamp_ < 0){
        /// if time_stamp_ < 0, object is unitialised, so initialise with current values
        initKalmanFilter(latitude, longitude, time_stamp_now);
    }else{
        /// else apply kalman filter.
        z_(0,0) = latitude * 1000.0;
        z_(1,0) = longitude * 1000.0;

        /// set control.
        u_(0,0) = control;

        /// Set the seconds per timestep in the velocity2d model.
        ///  The position units are in thousandths of latitude and longitude. The velocity units are in thousandths of position units per second.
        ///  So if there is one second per timestep, a velocity of 1 will change the lat or long by 1 after a million timesteps.
        ///  Thus a typical position is hundreds of thousands of units. A typical velocity is maybe ten.

        delta_t_ = time_stamp_now - time_stamp_;
        //std::cout <<"delta_t" << delta_t_ << std::endl;
        //std::cout <<"time_stamp_now"<< time_stamp_now << std::endl;
        //std::cout <<"time_Stamp"<< time_stamp_<< std::endl;

        if(delta_t_ > 0.0){
            /// time has moved on, so the uncertainty in the current position increases
            double unit_scaler = 0.001;                         /// unit_scaler accounts for the relation between position and velocity units.
            A_(0,2) = unit_scaler * delta_t_;
            A_(1,3) = unit_scaler * delta_t_;

            time_stamp_ = time_stamp_now;
        }
        ///--------- Predictive ----------///
        /// Predict the state
        x_raw_ = A_ * x_ + B_ * u_;
        /// Predict the state estimate covariance
        p_raw_ = A_ * p_ * A_.transpose() + Q_;
        /// -------- Estimation ----------///
        /// Calculate the optimal Kalman gain.
        K_ = p_raw_ * H_.transpose() * (H_ * p_raw_ * H_.transpose() + R_).inverse();
        /// Estimate the state
        x_ = x_raw_ + K_ * (z_ - H_ * x_raw_);
        /// Estimate the state covariance
        p_ = (Eigen::MatrixXd::Identity(STATE_DIM,STATE_DIM) - K_ * H_) * p_raw_;
    }
}


/**
 * @description: Extract a latitude from a velocity2d Kalman filter.
 * @param None
 * @return double Latitude
 */
double GPSKalmanFilter2d::getLatitude(){
    return x_(0,0) / 1000.0;
}

/**
 * @description: Extract a longitude from a velocity2d Kalman filter.
 * @param None
 * @return double Longitude
 */
double GPSKalmanFilter2d::getLongitude(){
    return x_(1,0) / 1000.0;
}

/**
 * @description: Extract velocity with lat-long-per-second units from a velocity2d Kalman filter.
 * @param None
 * @return double delta_lat -> Lat speed in lat-long-per-second.
 */
double GPSKalmanFilter2d::getLatVelocity(){
    double delta_lat;
    delta_lat = x_(2,0) / (1000.0 * 1000.0);
    return delta_lat;
}

/**
 * @description: Extract velocity with lat-long-per-second units from a velocity2d Kalman filter.
 * @param None
 * @return double[] delta_lon -> Lon speed in lat-long-per-second.
 */
double GPSKalmanFilter2d::getLonVelocity(){
    double delta_lon;
    delta_lon = x_(3,0) / (1000.0 * 1000.0);
    return delta_lon;
}

/**
 * @description: extract a bearing from a velocity2d Kalman filter. 0 = north, 90 = east, 180 = south, 270 = west
 * @see http://www.movable-type.co.uk/scripts/latlong.html for formulas
 * @param None
 * @return double bearing -> target bearing in degrees.
 */
double GPSKalmanFilter2d::getBearing(){
    double x, y;

    /// Convert to radians.
    double latitude = getLatitude() * deg2rad;
    double longitude = getLongitude() * deg2rad;
    double delta_latitude = getLatVelocity() * deg2rad;
    double delta_longitude = getLonVelocity() * deg2rad;

    /// Calculate bearing.
    y= sin(delta_longitude) * cos(latitude);
    x = cos(latitude - delta_latitude) * sin(latitude)
        - sin(latitude - delta_latitude) * cos(latitude) * cos(delta_longitude);
    double bearing = atan2(y,x);

    /// Convert to degress.
    bearing = bearing * rad2deg;
    if(bearing >= 360.0) bearing = bearing - 360.0;
    if(bearing < 0.0) bearing = bearing + 360.0;

    return bearing;
}

/**
 * @description: Extract speed in meters per second from a velocity2d Kalman filter.
 * @param double altitude -> target altitude above sea level.
 * @return double meters_per_second -> target speed in m/s.
 */
double GPSKalmanFilter2d::getSpeed(double altitude){
    /// FIXME Radius should be calculated depending on latitude instead
    /// http://en.wikipedia.org/wiki/Earth_radius#Radius_at_a_given_geodetic_latitude
    double EARTH_RADIUS_IN_METERS = 6371009;

    /// Convert to radians.
    double latitude = getLatitude() * deg2rad;
    double longitude = getLongitude() * deg2rad;
    double delta_latitude = getLatVelocity() * deg2rad;
    double delta_longitude = getLonVelocity() * deg2rad;

    /// First, let's calculate a unit-independent measurement -
    /// the radii of the earth traveled in each second.
    /// (Presumably this will be a very small number.)

    /// Haversine formula.
    double lat1 = latitude - delta_latitude;
    double sin_half_dlat = sin(delta_latitude / 2.0);
    double sin_half_dlon = sin(delta_longitude / 2.0);

    double a = sin_half_dlat * sin_half_dlat + cos(lat1) * cos(latitude) * sin_half_dlon * sin_half_dlon;
    double radians_per_second = 2 * atan2(1000.0 * sqrt(a), 1000.0 * sqrt(1.0 - a));

    /// Convert units.
    double meters_per_second = radians_per_second * (EARTH_RADIUS_IN_METERS + altitude);

    return meters_per_second;
}


