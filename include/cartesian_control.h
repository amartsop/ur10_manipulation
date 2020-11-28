#pragma once 

#include <iostream>
#include <vector>
#include <stdlib.h>
#include "ros/ros.h"
#include "ros/console.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "angle_conversion.h"
#include "input_trajectory.h"


typedef moveit::planning_interface::MoveGroupInterface mpi;
typedef AngleConversion ac;


class CartesianControl
{
    public:
        CartesianControl(){};

        // Initialize robot 
        void robot_initialization(mpi& move_group);

        void trajectory_execution(mpi& move_group);

    private:
        /******************* Robot Initialization *********************/
        // Offset tolerance
        // ****** CHANGE TOLERANCE WHEN WORKING WITH THE ROBOT ******
        const std::vector<double> m_init_tol = {100.0, 100.0,  100.0,
            100.0, 100.0, 100.0};

        // Initial joint values
        const std::vector<double> m_init_joints_vals = {0.0, -1.7279, -1.7279,
            0.0, 1.57, 0.0};

        // Safe state mode
        bool m_safe_state = false;

        // Joint variables names 
        std::vector<std::string> m_joints_names;

    private: 
        // Print joint values
        void print_current_joint_values(mpi& move_group);

        // Print current end_effector pose
        void print_current_ee_pose(mpi& move_group);

        // // Cartesian trajectory generation
        // geometry_msgs::PoseStamped cartesian_trajectory_parsing(const
        //     std::vector<double>& robot_pose);
};



    // // Get current pose
    // geometry_msgs::PoseStamped current_ee_pose = move_group.getCurrentPose();

    // // Find current Euler angles 
    // double current_w = current_ee_pose.pose.orientation.w;
    // double current_x = current_ee_pose.pose.orientation.x;
    // double current_y = current_ee_pose.pose.orientation.y;
    // double current_z = current_ee_pose.pose.orientation.z;


    // ac::Euler current_euler = ac::quaternions_to_euler(current_w, current_x, 
    //     current_y, current_z);

    // // Desired position
    // geometry_msgs::PoseStamped desired_ee_pose = current_ee_pose;

    // // Desired euler angles
    // ac::Euler desired_euler = current_euler;
    // desired_euler.psi = 1.0 * M_PI / 2.0;
    // desired_euler.theta = 0.0 * M_PI / 2.0;

    // // Desired quaternions
    // ac::Quaternions desired_quatern = ac::euler_to_quaternions(desired_euler.phi, 
    //     desired_euler.theta, desired_euler.psi);

    // // Change position
    // desired_ee_pose.pose.orientation.w = desired_quatern.w;
    // desired_ee_pose.pose.orientation.x = desired_quatern.x;
    // desired_ee_pose.pose.orientation.y = desired_quatern.y;
    // desired_ee_pose.pose.orientation.z = desired_quatern.z;

    // // Set desired position
    // move_group.setPoseTarget(desired_ee_pose);
    // move_group.move();


    // // Print pose
    // std::cout << "x: " << move_group.getCurrentPose().pose.position.x << std::endl;
    // std::cout << "y: " << move_group.getCurrentPose().pose.position.y << std::endl;
    // std::cout << "z: " << move_group.getCurrentPose().pose.position.z << std::endl;


    // double w = move_group.getCurrentPose().pose.orientation.w;
    // double x = move_group.getCurrentPose().pose.orientation.x;
    // double y = move_group.getCurrentPose().pose.orientation.y;
    // double z = move_group.getCurrentPose().pose.orientation.z;

    // ac::Euler euler = ac::quaternions_to_euler(w, x, y, z);

    // std::cout << "phi_rot: " << euler.phi << std::endl;
    // std::cout << "theta_rot: " << euler.theta << std::endl;
    // std::cout << "psi_rot: " << euler.psi << std::endl;


    // std::vector<double> joint_vec = move_group.getCurrentJointValues();
    // for(unsigned int i = 0; i < joint_vec.size(); i++)
    // {
    //     std::cout << m_joints_names.at(i) << ": " << joint_vec.at(i) << std::endl;
    // }
