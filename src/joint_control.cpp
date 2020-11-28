#include "../include/joint_control.h"


JointControl::JointControl()
{
    // tell the action client that we want to spin a thread by default
    m_traj_client = new TrajClient("arm_controller/follow_joint_trajectory", true);

    // Initialize trajectory
    const std::vector<double> params = {0.2, 2.0, 0.0};
    m_traj_design.initialize(TrajectoryDesign::TRAJ_SIN, params);

    // wait for action server to come up
    while(!m_traj_client->waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    // Robot Initialization routine
    m_goal.trajectory.joint_names = m_joint_names;

    m_goal.trajectory.points.resize(1);
    m_goal.trajectory.points[0].positions = m_initial_pos;
    m_goal.trajectory.points[0].velocities = m_initial_vel;
    m_goal.trajectory.points[0].time_from_start = ros::Duration(0.0);
    m_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(3.0);

    m_traj_client->sendGoal(m_goal);
    

    while (!m_traj_client->getState().isDone())
    {
        ros::spinOnce();
    }
}       


void JointControl::trajectory_generation(void)
{
    double t_duration = 10.0; // Final time (s)
    double f_sampling = 1e2;  // Sampling frequency (Hz)

    double t_sampling = 1.0 / f_sampling; // Time step (period) (s)
    int samples_num = t_duration / t_sampling;

    // Initialize sample points
    m_goal.trajectory.points.resize(samples_num);

    // Time now
    double time_now = 0.0;

    for(int i = 0; i < samples_num; i++)
    {
        double pos = m_traj_design.get_position(time_now);
        double vel = m_traj_design.get_velocity(time_now);

        // Set positions
        m_goal.trajectory.points[i].positions.resize(m_joint_num);
        m_goal.trajectory.points[i].positions = m_initial_pos;
        m_goal.trajectory.points[i].positions[0] = m_initial_pos[0] + pos;

        // Set velocities
        m_goal.trajectory.points[i].velocities.resize(m_joint_num);
        m_goal.trajectory.points[i].velocities = m_initial_vel;
        m_goal.trajectory.points[i].velocities[0] = m_initial_vel[0] + vel;

        // Motion duration
        m_goal.trajectory.points[i].time_from_start = ros::Duration(time_now);
            
        // Update time
        time_now += t_sampling;
    }

    m_traj_client->sendGoal(m_goal);
}




JointControl::~JointControl()
{
    delete m_traj_client;
}



// void JointControl::start_trajectory(void)
// {
//     // When to start the trajectory: 1s from now
//     m_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

//     m_traj_client->sendGoal(m_goal);
// }


// void JointControl::simple_trajectory(void)
// {

    // control_msgs::FollowJointTrajectoryGoal goal;

    // // First, the joint names, which apply to all waypoints
    // goal.trajectory.joint_names.push_back("shoulder_pan_joint");
    // goal.trajectory.joint_names.push_back("shoulder_lift_joint");
    // goal.trajectory.joint_names.push_back("elbow_joint");
    // goal.trajectory.joint_names.push_back("wrist_1_joint");
    // goal.trajectory.joint_names.push_back("wrist_2_joint");
    // goal.trajectory.joint_names.push_back("wrist_3_joint");


    // // We will have two waypoints in this goal trajectory
    // m_goal.trajectory.points.resize(2);

    // /************* First trajectory point ************************/
    // // Positions
    // int ind = 0;
    // m_goal.trajectory.points[ind].positions.resize(6);
    // m_goal.trajectory.points[ind].positions[0] = 1.5;
    // m_goal.trajectory.points[ind].positions[1] = -0.2;
    // m_goal.trajectory.points[ind].positions[2] = -1.57;
    // m_goal.trajectory.points[ind].positions[3] = 0.0;
    // m_goal.trajectory.points[ind].positions[4] = 0.0;
    // m_goal.trajectory.points[ind].positions[5] = 0.0;

    // // Velocities
    // m_goal.trajectory.points[ind].velocities.resize(6);
    // for (size_t j = 0; j < 6; ++j)
    // {
    //   goal.trajectory.points[ind].velocities[j] = 0.0;
    // }

    // // To be reached 1 second after starting along the trajectory
    // goal.trajectory.points[ind].time_from_start = ros::Duration(1.0);

    // // Second trajectory point
    // // Positions
    // ind += 1;
    // goal.trajectory.points[ind].positions.resize(6);
    // goal.trajectory.points[ind].positions[0] = 2.5;
    // goal.trajectory.points[ind].positions[1] = 0.0;
    // goal.trajectory.points[ind].positions[2] = -1.57;
    // goal.trajectory.points[ind].positions[3] = 0.0;
    // goal.trajectory.points[ind].positions[4] = 0.0;
    // goal.trajectory.points[ind].positions[5] = 0.0;

    // // Velocities
    // goal.trajectory.points[ind].velocities.resize(6);
    // for (size_t j = 0; j < 6; ++j)
    // {
    //   goal.trajectory.points[ind].velocities[j] = 0.0;
    // }

    // // To be reached 2 seconds after starting along the trajectory
    // goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

    // //we are done; return the goal
    // return goal;
// }


