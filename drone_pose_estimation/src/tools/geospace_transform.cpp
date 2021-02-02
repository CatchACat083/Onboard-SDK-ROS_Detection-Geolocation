 
//Matrix include
#include <Eigen/Dense>
#include <math.h>

#define pi (double)3.141592653589793

class geospaceTransform
{
    public:
        #define ROTATE_XYZ  1   ///< rotation order:X, Y, Z
        #define ROTATE_ZYX  0   ///< rotation order:Z, Y, X

    public:
        /// @brief Transform LLH to NED coordinate frame.
        void LLH2NED(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1> NEDOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInNEDPosition);

        /// @brief Transform NED coordinate frame to LLH.
        void NED2LLH(Eigen::Matrix<double, 3, 1> targetInNEDPosition, 
                     Eigen::Matrix<double, 3, 1> NEDOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform LLH to geocentric coordinate frame.
        void LLH2XYZ(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        /// @brief Transform geocentric coordinate frame to LLH.
        void XYZ2LLH(Eigen::Matrix<double, 3, 1> targetInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform ENU coordinate frame to LLH.
        void ENU2LLH(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition,
                     Eigen::Matrix<double, 3, 1>& targetInLLHPosition);

        /// @brief Transform ENU coordinate frame to geocentric coordinate frame.
        void ENU2XYZ(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                     Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition,
                     Eigen::Matrix<double, 3, 1>& targetInXYZPosition);

        /// @brief transform euler angle to rotation matrix.
        void eulerToRotation(Eigen::Vector3d euler_angle,
                             Eigen::Matrix<double, 4, 4> & rotation,
                             int rotation_order);

        /// @brief transform euler angle to rotation matrix.
        void eulerToRotation(Eigen::Vector3d euler_angle,
                             Eigen::Matrix<double, 3, 3> & rotation,
                             int rotation_order);
            
};







/**
 * @description: Firstly transform llh[] to geocentric coordinate frame by function LLH2XYZ(), then transform to NED coordinate frame.
 * @param[in]   targetInLLHPosition         [latitude, longitude, altitude] to be transformed
 * @param[in]   NEDOriginInXYZPosition      NED origin position in geocentric coordinate frame
 * @param[out]  targetInNEDPosition         spatial positions in local NED(world) coordinate frame
 */
void geospaceTransform::LLH2NED(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                                            Eigen::Matrix<double, 3, 1> NEDOriginInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInNEDPosition)
{
    Eigen::Matrix<double, 3, 1> targetInXYZPosition;
    Eigen::Matrix<double, 4, 4> initNEDToLocalNEDRotation;  ///< rotation matrix from NED(latitude=0, longitude=0) coordinate frame to local NED coordinate frame
    Eigen::Vector3d initNEDToLocalNEDEuler;                        ///< euler angles rotated from NED(latitude=0, longitude=0) coordinate frame to local NED coordinate frame
    
    /// transform to geocentric coordinate frame
    LLH2XYZ(targetInLLHPosition, targetInXYZPosition);

    initNEDToLocalNEDEuler[0] = targetInLLHPosition[1];
    initNEDToLocalNEDEuler[1] = -targetInLLHPosition[0];
    initNEDToLocalNEDEuler[2] = 0.0;
    eulerToRotation(initNEDToLocalNEDEuler, initNEDToLocalNEDRotation, ROTATE_XYZ);

    Eigen::Vector3d geocentricToInitNEDEuler;                   ///< euler angles rotating from ECEF geocentirc coordinate frame to NED coordinate frame
    Eigen::Matrix<double, 4, 4> geocentricToInitNEDRotation;    ///< rotation matrix from ECEF geocentirc coordinate frame to NED coordinate frame
    geocentricToInitNEDEuler << 0.0 * pi, -0.5 * pi, 0.0 * pi;
    eulerToRotation(geocentricToInitNEDEuler, geocentricToInitNEDRotation, ROTATE_ZYX);

    Eigen::Vector4d tempXYZPosition;
    tempXYZPosition << targetInXYZPosition[0] - NEDOriginInXYZPosition[0],
                       targetInXYZPosition[1] - NEDOriginInXYZPosition[1],
                       targetInXYZPosition[2] - NEDOriginInXYZPosition[2],
                       1.0;
    Eigen::Vector4d tempNEDPosition;

    //targetInNEDPosition = initNEDToLocalNEDRotation * geocentricToInitNEDRotation * (targetInXYZPosition - NEDOriginInXYZPosition);
    tempNEDPosition = initNEDToLocalNEDRotation * geocentricToInitNEDRotation * tempXYZPosition;
    targetInNEDPosition << tempNEDPosition[0], tempNEDPosition[1], tempNEDPosition[2];
}


/**
 * @description: Firstly transform NEDPosition to ENU coordinate frame, then transform to LLH by function ENU2LLH().
 * @param[in]   targetInNEDPosition        spatial positions in local NED(world) coordinate frame
 * @param[in]   NEDOriginInLLHPosition     NED origin position in LLH [latitude, longitude, altitude] coordinate frame
 * @param[out]  targetInLLHPosition        [latitude, longitude, altitude][rad, rad, m] after transformation
 * @see ENU2LLH
 */
void geospaceTransform::NED2LLH(Eigen::Matrix<double, 3, 1> targetInNEDPosition, 
                                            Eigen::Matrix<double, 3, 1> NEDOriginInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{
    Eigen::Matrix<double, 3, 1> targetInLocalENUPosition;

    Eigen::Vector3d NEDToENUEuler;                  ///< euler angles <rad, rad, rad> rotationg from NED coordinate frame to ENU coordinate frame.
    Eigen::Matrix<double, 4, 4> NEDToENURotation;   ///< rotation matrix rotationg from NED coordinate frame to ENU coordinate frame.
    /// NED to ENU euler angles and rotarion matrix.
    NEDToENUEuler << 1.0 * pi, 0.0 * pi, 0.5 * pi;
    eulerToRotation(NEDToENUEuler, NEDToENURotation, ROTATE_ZYX);

    /// transform to ENU fisrt
    // targetInLocalENUPosition = NEDToENURotation * targetInNEDPosition;

    Eigen::Vector4d tempNEDPosition;
    tempNEDPosition << targetInNEDPosition[0],
                       targetInNEDPosition[1],
                       targetInNEDPosition[2],
                       1.0;
    Eigen::Vector4d tempENUPosition;
    tempENUPosition = NEDToENURotation * tempNEDPosition;
    targetInLocalENUPosition << tempENUPosition[0], tempENUPosition[1], tempENUPosition[2];
    
    /// transform to LLH
    ENU2LLH(targetInLocalENUPosition, NEDOriginInLLHPosition, targetInLLHPosition);
    
}


/**
 * @description: Geodetic of specified ellipsoid (llh) (default WGS-84) to ecef geocentric coordinate frame (xyz) transform.
 * @param [in]   targetInLLHPosition             [latitude, longitude, altitude][rad, rad, m] the gps coordinate.
 * @param [out]  targetInXYZPosition             [x, y, z] position in geocentric coordinate frame after transformation
 */
void geospaceTransform::LLH2XYZ(Eigen::Matrix<double, 3, 1> targetInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInXYZPosition)
{
    /// --- START --- ///

    // double latitude = targetInLLHPosition(0);
    // double longitude = targetInLLHPosition(1);
    // double altitude = targetInLLHPosition(2);

    // double a = 6378137.0000;            ///< semi-major axis of the earth ellipsoid = 6378.137 km
    // double b = 6356752.3142;            ///< semi-minor axis of the earth ellipsoid = 6356.7523141 km.
    // double e = sqrt(1 - pow((b / a), 2));    ///< first eccentricity of the earth ellipsoid.

    // double sinphi = sin(latitude);
    // double cosphi = cos(latitude);
    // double coslam = cos(longitude);
    // double sinlam = sin(longitude);
    // double tan2phi = tan(latitude) * tan(latitude);
    // double tmp = 1 - e * e;
    // double tmpden = sqrt(1 + tmp * tan2phi);

    // double X = (a * coslam) / tmpden + altitude * coslam * cosphi;
    // double Y = (a * sinlam) / tmpden + altitude * sinlam * cosphi;

    // double tmp2 = sqrt(1 - e * e * sinphi * sinphi);
    // double Z = (a * tmp * sinphi) / tmp2 + altitude * sinphi;

    // targetInXYZPosition << X, Y, Z;

    /// --- END --- ///

    double latitude = targetInLLHPosition(0);
    double longitude = targetInLLHPosition(1);
    double altitude = targetInLLHPosition(2);

    double a = 6378137;         // The semi-major axis of the earth ellipsoid = 6378.137 km.
    double b = 6356752.3141;  // The semi-minor axis of the earth ellipsoid = 6356.7523141 km.
    double H = altitude + a;

    double e = sqrt(1 - (pow(b, 2) / pow(a, 2)));  //The first eccentricity of the earth ellipsoid.
    // double e1 = sqrt(pow(a, 2) - pow(b, 2)) / a;
    // double e = sqrt(0.006693421622966);      //Krasovski earth ellipsoid.
    // double e = sqrt(0.006694384999588);      //1975 earth ellipsoid.
    // double e = sqrt(0.0066943799013);        //WGS-84 earth ellipsoid.

    double B = latitude;
    double L = longitude;
    double W = sqrt(1 - pow(e, 2) * pow(sin(B), 2));    //First latitude function. 第一纬度函数
    double N = a / W;                           //radius of curvature in prime vertical. 椭球的卯酉圈曲率半径


    targetInXYZPosition <<    (N + H) * cos(B) * cos(L),
                (N + H) * cos(B) * sin(L),
                (N * (1 - pow(e, 2)) + H) * sin(B);

}



/**
 * @description: geocentric coordinate frame xyz to llh transform.
 * @param [in]  targetInXYZPosition             [x, y, z]  position in geocentric coordinate frame to be transformed
 * @param [out] targetInLLHPosition             [latitude, longitude, altitude][deg, deg, m] the gps coordinate after transformation.
 */
void geospaceTransform::XYZ2LLH(Eigen::Matrix<double, 3, 1> targetInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{
    /// --- START --- ///

    // double X = targetInXYZPosition(0);
    // double Y = targetInXYZPosition(1);
    // double Z = targetInXYZPosition(2);
    // double x2 = X * X;
    // double y2 = Y * Y;
    // double z2 = Z * Z;

    // double a = 6378137.0;
    // double b = 6356752.3142;
    // double e = sqrt(a * a - b * b) / a;

    // double b2 = b * b;
    // double e2 = e * e;
    // double ep = e * (a / b);
    // double r = sqrt(x2 + y2);
    // double r2 = r * r;
    // double E2 = a * a - b * b;
    // double F = 54 * b2 * z2;
    // double G = r2 + (1 - e2) * z2 - e2 * E2;
    // double c = (e2 * e2 * F * r2) / (G * G * G);
    // double s = pow(1 + c + sqrt(c * c + 2 * c), 1.0 / 3);
    // double P = F / (3 * pow(s + 1 / s + 1, 2) * G * G);
    // double Q = sqrt(1 + 2 * e2 * e2 * P);
    // double ro = -(P * e2 * r) / (1 + Q) + sqrt((a * a / 2) * (1 + 1 / Q) - (P * (1 - e2) * z2) / (Q * (1 + Q)) - P * r2 / 2);
    // double tmp = pow(r - e2 * ro, 2);
    // double U = sqrt(tmp + z2);
    // double V = sqrt(tmp + (1 - e2) * z2);
    // double zo = (b2 * Z) / (a * V);

    // double height = U * (1 - b2 / (a * V));

    // double latitude = atan((Z + ep * ep * zo) / r);

    // double tmp2 = atan(Y / X);

    // double longitude;

    // if (X >= 0){
    //     longitude = tmp2;
    // }else if (X < 0 && Y >= 0){
    //     longitude = pi + tmp2;
    // }else{
    //     longitude = tmp2 - pi;
    // }
    
    // targetInLLHPosition << latitude, longitude, height;

    /// ---END--- ///

    /// https://www.cnblogs.com/marvelousone/p/11265895.html
    /// https://www.oc.nps.edu/oc2902w/coord/llhxyz.htm?source=post_page
    
    double x = targetInXYZPosition(0);
    double y = targetInXYZPosition(1);
    double z = targetInXYZPosition(2);

    double s = sqrt(pow(x, 2) + pow(y, 2));
    double tanB = z / sqrt(pow(x, 2) + pow(y, 2));

    double a = 6378137;         // The semi-major axis of the earth ellipsoid = 6378.137 km.
    double b = 6356752.3141;  // The semi-minor axis of the earth ellipsoid = 6356.7523141 km.

    double e = sqrt(1 - pow(b, 2) / pow(a, 2));  //The first eccentricity of the earth ellipsoid.
    
    // double e = sqrt(0.006693421622966);      //Krasovski earth ellipsoid.
    // double e = sqrt(0.006694384999588);      //1975 earth ellipsoid.
    //double e = sqrt(0.0066943799013);        //WGS-84 earth ellipsoid.

    double L = fabs(acos(x / s));
    double B1 = z / sqrt(pow(x, 2) + pow(y, 2));

    double B2 = 0.0;
    double N = 0.0;

    while(fabs(B2 - B1) > 1E-6){
        N = a / sqrt(1 - pow(e, 2) * pow(sin(B1), 2));
        B2 = atan((z + N * pow(e,2) * sin(B1)) / s);
        B1 = atan((z + N * pow(e,2) * sin(B2)) / s);
    }

    double H = z / sin(B1) - N * (1 - pow(e, 2));


    double latitude = B1;
    double longitude = L;
    //double longitude = atan(y / x) / deg2rad;
    double altitude = H - a;

    if(latitude < 0){
        latitude = latitude + pi;
    }
    if(longitude < 0){
        longitude = longitude + pi;
    }

    targetInLLHPosition <<  latitude,
                            longitude,
                            altitude;

}


/**
 * @description: ENU coordinate frame to geocentric coordinate frame xyz
 * @param[in]   targetInLocalENUPosition    position in ENU coordinate frame to be transformed
 * @param[in]   ENUOriginInXYZPosition      position of the origin of ENU coordinate frame in geocentric coordinate frame
 * @param[out]  targetInXYZPosition         position in geocentric coordinate frame after transformation
 */
void geospaceTransform::ENU2XYZ(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                                            Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInXYZPosition)
{
    Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition;
    XYZ2LLH(ENUOriginInXYZPosition, ENUOriginInLLHPosition);

    double latitude_origin = ENUOriginInLLHPosition(0);
    double longitude_origin = ENUOriginInLLHPosition(1);

    double C11 = -sin(longitude_origin);
    double C12 = cos(longitude_origin);
    double C21 = -sin(latitude_origin) * cos(longitude_origin);
    double C22 = -sin(latitude_origin) * sin(longitude_origin);
    double C23 = cos(latitude_origin); 
    double C31 = cos(latitude_origin) * cos(longitude_origin);  
    double C32 = cos(latitude_origin) * sin(longitude_origin);
    double C33 = sin(latitude_origin);

    double x_origin = ENUOriginInXYZPosition(0);
    double y_origin = ENUOriginInXYZPosition(1);
    double z_origin = ENUOriginInXYZPosition(2);

    double x_target = targetInLocalENUPosition(0);
    double y_target = targetInLocalENUPosition(1);
    double z_target = targetInLocalENUPosition(2);

    
    double dX = C11 * x_target + C21 * y_target+ C31 * z_target;
    double dY = C12 * x_target + C22 * y_target + C32 * z_target;
    double dZ = C23 * y_target + C33 * z_target;

    double X = x_origin + dX;
    double Y = y_origin + dY;
    double Z = z_origin + dZ;

    targetInXYZPosition << X, Y, Z;

}


/**
 * @description: Firstly transform the origin llh[] to geocentric coordinate frame XYZ[],
 *               by function LLH2XYZ(), then tansform the xyz to geocentric coordinate
 *               frame XYZGeo[] using the origin position XYZ[] by function ENU2XYZ(),
 *               finally transform the XYZGeo[] to LLH by function XYZ2LLH(). 
 * @param[in]   targetInLocalENUPosition     position in ENU coordinate frame to be transformed
 * @param[in]   ENUOriginInLLHPosition       position of the origin of ENU coordinate frame in LLH coordinate frame
 * @param[out]  targetInLLHPosition          [latitude, longitude, altitude] after transformation
 * @see LLH2XYZ
 * @see ENU2XYZ
 * @see XYZ2LLH
 */
void geospaceTransform::ENU2LLH(Eigen::Matrix<double, 3, 1> targetInLocalENUPosition,
                                            Eigen::Matrix<double, 3, 1> ENUOriginInLLHPosition,
                                            Eigen::Matrix<double, 3, 1>& targetInLLHPosition)
{  
    Eigen::Matrix<double, 3, 1> ENUOriginInXYZPosition;     ///< position of the origin of ENU coordinate frame in geocentric coordinate xyz frame after transformation
    /// transform LLH ENU origin position to geocentric xyz position first.
    LLH2XYZ(ENUOriginInLLHPosition, ENUOriginInXYZPosition);

    Eigen::Matrix<double, 3, 1> targetInXYZPosition;        ///< position in geocentric coordinate xyz frame after transformation
    /// transform target ENU position to geocentric xyz position.
    ENU2XYZ(targetInLocalENUPosition, ENUOriginInXYZPosition, targetInXYZPosition);

    /// transform to target xyz position to LLH position final
    XYZ2LLH(targetInXYZPosition, targetInLLHPosition);
}


/**
 * @description: Default rotation order: Z(yaw), Y(pitch), X(roll) with new axes respectively.
 * @param [in] eular_angle      eular angles to be transformed: [roll, pitch, yaw][rad, rad, rad]
 * @param [out] rotation        rotation matrix transformed from eular angles
 * @param [in] rotation_order   rotation order flag(ROTATE_ZYX: Z, Y, X, ROTATE_XYZ: X, Y, Z)
 */
void geospaceTransform::eulerToRotation(Eigen::Vector3d euler_angle,
                                                    Eigen::Matrix<double, 4, 4> & rotation,
                                                    int rotation_order = ROTATE_ZYX)
{
    double c_roll, c_pitch, c_yaw, s_roll, s_pitch, s_yaw;
    Eigen::Matrix<double, 4, 4> rotationMatrixRoll;                     /// rotation matrix around X axis.
    Eigen::Matrix<double, 4, 4> rotationMatrixPitch;                    /// rotation matrix around Y axis.
    Eigen::Matrix<double, 4, 4> rotationMatrixYaw;                      /// rotation matrix around Z axis.

    c_roll = cos(euler_angle[0]);       s_roll = sin(euler_angle[0]);   /// roll eular angle parameter [rad].
    c_pitch = cos(euler_angle[1]);      s_pitch = sin(euler_angle[1]);  /// pitch eular angle parameter [rad].
    c_yaw = cos(euler_angle[2]);        s_yaw = sin(euler_angle[2]);    /// yaw eular angle parameter [rad].

    rotationMatrixRoll  <<   1,      0,       0,    0,
                            0,  c_roll,  s_roll,    0,
                            0, -s_roll,  c_roll,    0,
                            0,       0,       0,    1;
    
    rotationMatrixPitch <<  c_pitch,    0, -s_pitch,    0,
                                  0,    1,        0,    0,
                            s_pitch,    0,  c_pitch,    0,
                                  0,    0,        0,    1;
    
    rotationMatrixYaw  <<    c_yaw, s_yaw,   0,     0,
                            -s_yaw, c_yaw,   0,     0,
                                 0,     0,   1,     0,
                                 0,     0,   0,     1;

    /// rotation order: Z, Y, X
    if(rotation_order == ROTATE_ZYX)
    {
        rotation = rotationMatrixRoll * rotationMatrixPitch * rotationMatrixYaw;
    }
    else if(rotation_order == ROTATE_XYZ){
        rotation = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;
    }
}

/**
 * @description: Default rotation order: Z(yaw), Y(pitch), X(roll) with new axes respectively.
 * @param [in] eular_angle      eular angles to be transformed: [roll, pitch, yaw][rad, rad, rad]
 * @param [out] rotation        rotation matrix transformed from eular angles <3, 3>
 * @param [in] rotation_order   rotation order flag(ROTATE_ZYX: Z, Y, X, ROTATE_XYZ: X, Y, Z)
 */
void geospaceTransform::eulerToRotation(Eigen::Vector3d euler_angle,
                                                    Eigen::Matrix<double, 3, 3> & rotation,
                                                    int rotation_order = ROTATE_ZYX)
{
    double c_roll, c_pitch, c_yaw, s_roll, s_pitch, s_yaw;
    Eigen::Matrix<double, 3, 3> rotationMatrixRoll;                     /// rotation matrix around X axis.
    Eigen::Matrix<double, 3, 3> rotationMatrixPitch;                    /// rotation matrix around Y axis.
    Eigen::Matrix<double, 3, 3> rotationMatrixYaw;                      /// rotation matrix around Z axis.

    c_roll = cos(euler_angle[0]);       s_roll = sin(euler_angle[0]);   /// roll eular angle parameter [rad].
    c_pitch = cos(euler_angle[1]);      s_pitch = sin(euler_angle[1]);  /// pitch eular angle parameter [rad].
    c_yaw = cos(euler_angle[2]);        s_yaw = sin(euler_angle[2]);    /// yaw eular angle parameter [rad].

    rotationMatrixRoll  <<   1,      0,       0,
                            0,  c_roll,  s_roll,
                            0, -s_roll,  c_roll;
    
    rotationMatrixPitch <<  c_pitch,    0, -s_pitch,
                                  0,    1,        0,
                            s_pitch,    0,  c_pitch;
    
    rotationMatrixYaw  <<    c_yaw, s_yaw,   0,
                            -s_yaw, c_yaw,   0,
                                 0,     0,   1;

    /// rotation order: Z, Y, X
    if(rotation_order == ROTATE_ZYX)
    {
        rotation = rotationMatrixRoll * rotationMatrixPitch * rotationMatrixYaw;
    }
    else if(rotation_order == ROTATE_XYZ){
        rotation = rotationMatrixYaw * rotationMatrixPitch * rotationMatrixRoll;
    }
}
