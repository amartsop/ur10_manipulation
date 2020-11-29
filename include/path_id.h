#pragma once 

#include <iostream>
#include <vector> 


class PathID
{
    public:

        PathID(/* args */);

        // Cartesian space path struct
        struct Cartesian {
            double time = 0.0;
            std::vector<double> position = {0.0, 0.0, 0.0};
            std::vector<double> velocity = {0.0, 0.0, 0.0};
            std::vector<double> acceleration = {0.0, 0.0, 0.0};
            std::vector<double> euler_position = {0.0, 0.0, 0.0};
            std::vector<double> euler_velocity = {0.0, 0.0, 0.0};
            std::vector<double> euler_acceleration = {0.0, 0.0, 0.0};
        };


        // Joint space path struct
        struct Joint {
            double time;
            std::vector<double> position;
            std::vector<double> velocity;
            std::vector<double> acceleration;
        };

    private:
        /* data */

};



