#pragma once 

#include <iostream>
#include <vector>
#include <math.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "angle_conversion.h"



class InputTrajectory
{
    public:
        InputTrajectory(const geometry_msgs::PoseStamped& current_pose);

        // Pose getter
        std::vector<geometry_msgs::Pose> get_pose(void) { return m_pose; }

    private:

        // Trajectory initial and final time 
        double m_t0 = 0.0; // Initial time (s) 
        double m_tf = 10.0; // Final time (s)

        // Trajectory sampling frequency
        double m_fs = 10.0;  // Sampling frequency (Hz)
        double m_ts = 1.0 / m_fs; // Sampling period (s)
        unsigned int m_pnum; // Number of sampling points


        /****************** Trajectory variables *******************/
        // Time
        std::vector<double> m_time;

        // Pose vector 
        std::vector<geometry_msgs::Pose> m_pose;

        // Generate trajectory
        void generate_trajectory(const geometry_msgs::PoseStamped& current_pose);
};


