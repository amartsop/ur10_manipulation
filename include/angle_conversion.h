#pragma once 
#include <iostream>
#include <vector>
#include <math.h>


class AngleConversion
{
    public:
        AngleConversion();

        // Quaternions struct
        struct Quaternions { double w, x, y, z; };

        // Eulear angles struct
        struct Euler{ double phi, theta, psi; };

        /** Euler angles to quaternions. Euler angles follow the post multiply
            sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
            and "phi" around x (roll) **/
        static Quaternions euler_to_quaternions(double phi, double
            theta, double psi);

        /** Quaternions to Euler Angles. Euler angles follow the post multiply
            sequence zyx. Rotate "psi" around Z (yaw), "theta" around y (roll)
            and "phi" around x (roll) **/
        static Euler quaternions_to_euler(double w, double x, double y, 
            double z);
};


