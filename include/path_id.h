#pragma once 

#include <iostream>
#include <vector> 


class PathID
{
    public:

        PathID(/* args */);

        // Cartesian space path struct
        struct Cartesian {
            double time;
            double position[3];
            double velocity[3];
            double acceleration[3];
            double euler_position[3];
            double euler_velocity[3];
            double euler_acceleration[3];
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



