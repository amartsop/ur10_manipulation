#include "../include/angle_conversion.h"



/** Euler angles to quaternions. Euler angles follow the post multiply
    sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
    and "phi" around x (roll) **/
AngleConversion::Quaternions AngleConversion::euler_to_quaternions(double phi,
    double theta, double psi)
{
    AngleConversion::Quaternions quatern;

    double cy = cos(psi * 0.5); double sy = sin(psi * 0.5);
    double cp = cos(theta * 0.5); double sp = sin(theta * 0.5);
    double cr = cos(phi * 0.5); double sr = sin(phi * 0.5);

    quatern.w = cr * cp * cy + sr * sp * sy;
    quatern.x = sr * cp * cy - cr * sp * sy;
    quatern.y = cr * sp * cy + sr * cp * sy;
    quatern.z = cr * cp * sy - sr * sp * cy;

    // Return Quaternions
    return quatern;
}


AngleConversion::Euler AngleConversion::quaternions_to_euler(double w, double x, double y, 
double z)
{
    AngleConversion::Euler euler_angles;

    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    euler_angles.phi = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0)
    {
        euler_angles.theta = std::copysign(M_PI / 2.0, sinp); // use 90 degrees if out of range
    }
    else
    {
        euler_angles.theta = std::asin(sinp);
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    euler_angles.psi = std::atan2(siny_cosp, cosy_cosp);

    // Return Euler angles
    return euler_angles;
}