#pragma once

#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include "actionlib/client/simple_action_client.h"
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include "angle_conversion.h"


typedef control_msgs::FollowJointTrajectoryGoal fktg;
typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

typedef std::vector<double> vecd;


class RobotControl
{
    public:
        RobotControl(robot_model::RobotModelPtr kinematic_model,
            TrajClient& traj_client);
        ~RobotControl();

    public:
        // Go to cartesian position
        void goto_cartesian_position(double t, const vecd& pos, const vecd& euler,
        TrajClient& traj_client);





    public:
        // Go to joint position
        void goto_joint_position(double t, const vecd& pos, const vecd& vel,
            const vecd& accel, TrajClient& traj_client);

        // Go to joint position
        void goto_joint_position(vecd t, const std::vector<vecd>& pos,
            const std::vector<vecd>& vel, const std::vector<vecd>& accel,
            TrajClient& traj_client);
        
    public:
        // Get end effector position (m)
        vecd get_ee_position(void);

        // Get end effector orientation, euler angles phi, theta, psi (z, y', x'')
        vecd get_ee_orientation(void);

        // Get joint angles values
        vecd get_joint_angles(void);

    private:
        // Joint names
        std::vector<std::string> m_joint_names;

        // Link names
        std::vector<std::string> m_link_names;

        // Kinematic state ptr handle 
        robot_state::RobotState* m_kinematic_state;

        // Group joint model
        robot_state::JointModelGroup* m_joint_model_group;

    private:
        // Initialize robot 
        const std::vector<double> m_init_pos = {0.0, -1.7279, -1.7279,
            0.0, 1.57, 0.0};
        vecd m_init_vel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        vecd m_init_accel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        double m_t_init = 3.0;
};
