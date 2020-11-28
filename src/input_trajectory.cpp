#include "../include/input_trajectory.h"

InputTrajectory::InputTrajectory(const geometry_msgs::PoseStamped& current_pose)
{
    // Calculate number of trajectory points
    m_pnum = (unsigned int)((m_tf - m_t0) * m_fs);

    /****************** Resize trajectory variables *******************/
    // Resize time
    m_time.resize(m_pnum);

    // Resize pose
    m_pose.resize(m_pnum);

    // Generate trajectory
    generate_trajectory(current_pose);
}

void InputTrajectory::generate_trajectory(const geometry_msgs::PoseStamped& current_pose)
{
    // Calculate current euler 
    double curr_w = current_pose.pose.orientation.w;
    double curr_x = current_pose.pose.orientation.x;
    double curr_y = current_pose.pose.orientation.y;
    double curr_z = current_pose.pose.orientation.z;

    AngleConversion::Euler curr_eul = AngleConversion::quaternions_to_euler(curr_w,
        curr_x, curr_y, curr_z);

    // /******************* Position *******************/
    // Position amplitude (m)
    double a_p[3] = {0.0, 0.0, 0.5};

    // Position frequency (Hz)
   double f_p[3] = {1.0, 1.0, 0.5};

    // Position phase (rad)
    double phi_p[3] = {0.0, 0.0, 0.0};

    /******************* Orientation *******************/
    // Euler angles amplitude (rad)
    double a_o[3] = {0.0, 0.0, 0.0};

    // Euler angles frequency (Hz)
    double f_o[3] = {1.0, 5.0, 1.1};

    // Euler angles phase (rad)
    double phi_o[3] = {0.0, 0.0, 0.0};

    // Position and orientation vector
    double pos[3], euler[3];

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
            pos[i] = a_p[i] * sin(a);

            // Rigid body rotational trajectory
            double b_dot = 2.0 * M_PI * f_o[i];
            double b = b_dot * cur_time + phi_o[i];
            euler[i] = a_o[i] * sin(b);
        }

        /***************** Update state vector *****************/
        // Position
        m_pose.at(j).position.x = pos[0] + current_pose.pose.position.x;
        m_pose.at(j).position.y = pos[1] + current_pose.pose.position.y;
        m_pose.at(j).position.z = pos[2] + current_pose.pose.position.z;
        
        // Orientation
        AngleConversion::Quaternions quatern =
            AngleConversion::euler_to_quaternions(curr_eul.phi + euler[0],
            curr_eul.theta +  euler[1], curr_eul.psi + euler[2]);

        m_pose.at(j).orientation.w = quatern.w;
        m_pose.at(j).orientation.x = quatern.x;
        m_pose.at(j).orientation.y = quatern.y;
        m_pose.at(j).orientation.z = quatern.z;

        // Update time
        cur_time += m_ts;
    }
}