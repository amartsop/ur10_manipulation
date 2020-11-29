#pragma once 

#include <iostream>
#include <vector>
#include "../include/robot_control.h"


typedef std::vector<double> vecd;


class PathPlanning
{
    public:
        PathPlanning(const vecd& cur_pos, const vecd& cur_eul);
        
        // 
        struct Quaternions { double w, x, y, z; };

        // Eulear angles struct
        struct Euler{ double phi, theta, psi; };

        // Get time vector
        vecd get_time_vector(void) { return m_time; }

        // Get position vector
        std::vector<vecd> get_position_vector(void) { return m_pos; }

        // Get euler vector
        std::vector<vecd> get_euler_vector(void) { return m_eul; }

    private:

        // Trajectory initial and final time 
        double m_t0 = 0.0; // Initial time (s) 
        double m_tf = 10.0; // Final time (s)

        // Trajectory sampling frequency
        double m_fs = 100.0;  // Sampling frequency (Hz)
        double m_ts = 1.0 / m_fs; // Sampling period (s)
        unsigned int m_pnum; // Number of sampling points

        // Time
        vecd m_time;

        // Position and orientation
        std::vector<vecd> m_pos, m_eul;

        // Generate trajectory
        void generate_trajectory(const vecd& cur_pos, const vecd& cur_eul);
};


