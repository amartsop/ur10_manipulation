#include "../include/cartesian_control.h"


void CartesianControl::trajectory_execution(mpi& move_group)
{
    // Trajectory
    InputTrajectory traj(move_group.getCurrentPose());

    // Get trajectory
    std::vector<geometry_msgs::Pose> desired_pose = traj.get_pose();

    // move_group.setMaxVelocityScalingFactor(0.1);


    // robot_state::RobotState start_state(*move_group.getCurrentState());


    // moveit_msgs::RobotTrajectory trajectory;
    // const double jump_threshold = 0.0;
    // const double eef_step = 10e-3;
    // double fraction = move_group.computeCartesianPath(desired_pose,
    //     eef_step, jump_threshold, trajectory);

    // std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_points = 
    //  trajectory.joint_trajectory.points;

    // // for (auto point: trajectory_points)
    // // {
    // //     std::cout << point.time_from_start.sec << " " << point.time_from_start.nsec<< std::endl;
    // // }

    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // my_plan.trajectory_ = trajectory;


    // my_plan.start_state_ = 

}


// Initialize robot 
void CartesianControl::robot_initialization(mpi& move_group)
{
    // Get joint names 
    m_joints_names = move_group.getActiveJoints();

    // Get current joint values
    std::vector<double> current_joint_values = move_group.getCurrentJointValues();

    /********* Check whether all values are inside the tolerance limits ******/
    // Initialize safe state to true
    m_safe_state = true;

    // If at at least joint angle is not inside the range then change state
    for (unsigned int i = 0; i < current_joint_values.size(); i++)
    {
        double current_angle = abs(current_joint_values.at(i));
        double nominal_value = abs(m_init_joints_vals.at(i));

        if (abs(current_angle - nominal_value) >= m_init_tol.at(i))
        {
            std::string unsafe_msg = "Joint %s out of safe initialization range. No motion will be performed!";
            ROS_INFO(unsafe_msg.c_str(), m_joints_names.at(i).c_str());
            m_safe_state = false;
        }
    }

    // Sent robot to the initialzation position
    if(m_safe_state)
    {
        move_group.setJointValueTarget(m_init_joints_vals);
        move_group.move();
    }

    // Define cartesian desired position
}


// Print joint values
void CartesianControl::print_current_joint_values(mpi& move_group)
{
    std::vector<double> joint_vec = move_group.getCurrentJointValues();
    for(unsigned int i = 0; i < joint_vec.size(); i++)
    {
        std::cout << m_joints_names.at(i) << ": " << joint_vec.at(i) << " rad"
        << std::endl;
    }
}


// Print current end_effector pose
void CartesianControl::print_current_ee_pose(mpi& move_group)
{
    // Print end-effector position
    std::cout << "x: " << move_group.getCurrentPose().pose.position.x <<
        " m" << std::endl;
    std::cout << "y: " << move_group.getCurrentPose().pose.position.y <<
        " m" <<  std::endl;
    std::cout << "z: " << move_group.getCurrentPose().pose.position.z <<
        " m" << std::endl;

    // Print end-effector orientation
    double w = move_group.getCurrentPose().pose.orientation.w;
    double x = move_group.getCurrentPose().pose.orientation.x;
    double y = move_group.getCurrentPose().pose.orientation.y;
    double z = move_group.getCurrentPose().pose.orientation.z;

    ac::Euler euler = ac::quaternions_to_euler(w, x, y, z);

    std::cout << "phi: " << euler.phi << " rad" << std::endl;
    std::cout << "theta: " << euler.theta << " rad" << std::endl;
    std::cout << "psi: " << euler.psi << " rad" << std::endl;
}

// geometry_msgs::PoseStamped CartesianControl::cartesian_trajectory_parsing(const
//     std::vector<double>& robot_pose)
// {
//     // Current time 
//     double time = robot_pose.at(0);

//     // Current position
//     double pos_x = robot_pose.at(1);
//     double pos_y = robot_pose.at(2);
//     double pos_z = robot_pose.at(2);

//     // Current orientation
//     double eul_phi = robot_pose.at(4);
//     double eul_theta = robot_pose.at(5);
//     double eul_psi = robot_pose.at(6);

//     // Transform orientation to quaternions
//     AngleConversion::Quaternions quatern =
//         AngleConversion::euler_to_quaternions(eul_phi, eul_theta, eul_psi);

//     //Generate geometry msg
//     geometry_msgs::PoseStamped desired_ee_pose;

//     // Set time 
//     desired_ee_pose.header.stamp.sec = time;
    
//     // Set position 
//     desired_ee_pose.pose.position.x = pos_x;
//     desired_ee_pose.pose.position.y = pos_y;
//     desired_ee_pose.pose.position.z = pos_z;

//     // Set orientation
//     desired_ee_pose.pose.orientation.w = quatern.w;
//     desired_ee_pose.pose.orientation.x = quatern.x;
//     desired_ee_pose.pose.orientation.y = quatern.y;
//     desired_ee_pose.pose.orientation.z = quatern.z;

//     // Return 
//     return desired_ee_pose;
// }