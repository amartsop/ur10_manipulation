#include "../include/path_planning.h"

PathPlanning::PathPlanning(const vecd& pos, const vecd& eul)
{
    // Calculate number of trajectory points
    m_pnum = (unsigned int)((m_tf - m_t0) * m_fs);

    /****************** Resize trajectory variables *******************/
    // Resize time
    m_time.resize(m_pnum);

    // Resize pose
    m_pos.resize(m_pnum); m_eul.resize(m_pnum);

    // Generate trajectory
    generate_trajectory(pos, eul);
}


void PathPlanning::generate_trajectory(const vecd& pos, const vecd& eul)
{
    // /******************* Position *******************/
    // Position amplitude (m)
    double a_p[3] = {0.0, 0.0, 0.2};

    // Position frequency (Hz)
   double f_p[3] = {1.0, 1.0, 0.5};

    // Position phase (rad)
    double phi_p[3] = {0.0, 0.0, 0.0};

    /******************* Orientation *******************/
    // Euler angles amplitude (rad)
    double a_o[3] = {0.0, 0.0, 0.6};

    // Euler angles frequency (Hz)
    double f_o[3] = {1.0, 5.0, 1.1};

    // Euler angles phase (rad)
    double phi_o[3] = {0.0, 0.0, 0.0};

    // Position and orientation vector
    double rel_pos[3], rel_eul[3];

    // Current time
    double cur_time = m_t0;

    for (unsigned int j = 0; j < m_pnum; j++)
    {
        // Current time
        m_time.at(j) = cur_time;

        for (int i = 0; i < 3; i++)
        {
            // Rigid body translational trajectory
            double a_dot = 2.0 * M_PI * f_p[i];
            double a = a_dot * cur_time + phi_p[i];
            rel_pos[i] = a_p[i] * sin(a);

            // Rigid body rotational trajectory
            double b_dot = 2.0 * M_PI * f_o[i];
            double b = b_dot * cur_time + phi_o[i];
            rel_eul[i] = a_o[i] * sin(b);
        }

        /***************** Update state vector *****************/
        // Position
        double pos_x = rel_pos[0] + pos.at(0);
        double pos_y = rel_pos[1] + pos.at(1);
        double pos_z = rel_pos[2] + pos.at(2);
        m_pos.at(j) = {pos_x, pos_y, pos_z};

        // Orientation
        double eul_phi = rel_eul[0] + eul.at(0);
        double eul_theta = rel_eul[1] + eul.at(1);
        double eul_psi = rel_eul[2] + eul.at(2);
        m_eul.at(j) = {eul_phi, eul_theta, eul_psi};

        // Update time
        cur_time += m_ts;
    }
}