#include "../include/path_planning.h"

PathPlanning::PathPlanning()
{
    // Calculate number of trajectory points
    m_pnum = (unsigned int)((m_tf - m_t0) * m_fs);
}



std::vector<PathID::Cartesian> PathPlanning::generate_trajectory(const
    PathID::Cartesian& cur_pose)
{
    // Initiate pose target 
    std::vector<PathID::Cartesian> pose(m_pnum);

    // /******************* Position *******************/
    // Position amplitude (m)
    double a_p[3] = {0.2, 0.2, 0.2};

    // Position frequency (Hz)
    double f_p[3] = {1.0, 1.0, 1.0};

    // Position phase (rad)
    double phi_p[3] = {0.0, 0.0, 0.0};

    /******************* Orientation *******************/
    // Euler angles amplitude (rad)
    double a_o[3] = {0.5, 0.5, 0.5};

    // Euler angles frequency (Hz)
    double f_o[3] = {1.0, 1.0, 1.0};

    // Euler angles phase (rad)
    double phi_o[3] = {0.0, 0.0, 0.0};

    // Current time
    double cur_time = m_t0;

    for (unsigned int j = 0; j < m_pnum; j++)
    {
        // Current time
        pose.at(j).time = cur_time;

        for (int i = 0; i < 3; i++)
        {
            // Rigid body translational trajectory
            double a_dot = 2.0 * M_PI * f_p[i];
            double a = a_dot * cur_time + phi_p[i];

            pose.at(j).position[i] = a_p[i] * sin(a) + cur_pose.position[i];

            pose.at(j).velocity[i] = a_p[i] * a_dot * cos(a) +
                cur_pose.velocity[i];

            pose.at(j).acceleration[i] = - a_p[i] * powf(a_dot, 2.0) * sin(a) 
                + cur_pose.velocity[i];

            // Rigid body rotational trajectory
            double b_dot = 2.0 * M_PI * f_o[i];
            double b = b_dot * cur_time + phi_o[i];
            
            pose.at(j).euler_position[i] = a_o[i] * sin(b) +
                cur_pose.euler_position[i];

            pose.at(j).euler_velocity[i] = a_o[i] * b_dot * cos(b) + 
                cur_pose.euler_velocity[i];

            pose.at(j).euler_acceleration[i] = - a_o[i] * powf(b_dot, 2.0) *
                sin(b) + cur_pose.euler_acceleration[i];
        }

        // Update time
        cur_time += m_ts;
    }
    
    return pose;
}