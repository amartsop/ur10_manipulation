#pragma once

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"

#include "trajectory_design.h"


typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

class JointControl
{
    public:
        JointControl();
        ~JointControl();

        // Trajectory function 
        void trajectory_generation(void);

    //    //! Sends the command to start a given trajectory
    //    void start_trajectory(void);

    //     // Create a simple trajectory
    //     void simple_trajectory(void);

        // Returns the current state of the action
        actionlib::SimpleClientGoalState get_action_state()
        {
            return m_traj_client->getState();
        }

    private:
        // Trajectory client
        TrajClient* m_traj_client;

        // Joint names
        const std::vector<std::string> m_joint_names = {"shoulder_pan_joint",
            "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", 
            "wrist_2_joint", "wrist_3_joint"};

        // Number of joints 
        const unsigned int m_joint_num = 6;

        // Initialize robot 
        const std::vector<double> m_initial_pos = {1.5, -0.2, -1.57, 0.0, 0.0, 0.0};
        const std::vector<double> m_initial_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Goal point
        control_msgs::FollowJointTrajectoryGoal m_goal;

        // Trajectory design
        TrajectoryDesign m_traj_design;
};