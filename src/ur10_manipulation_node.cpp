#include <iostream>

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "../include/path_id.h"
#include "../include/robot_control.h"
#include "../include/path_planning.h"



typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    TrajClient;

int main(int argc, char** argv)
{
    // Initiate ur10_manipulation node 
    ros::init(argc, argv, "ur10_manipulation"); 

    // Initiate asychronous callback check
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Create follow joint trajectroy action
    std::string traj_client_name = "arm_controller/follow_joint_trajectory";
    TrajClient traj_client(traj_client_name, true);

    while(!traj_client.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the joint_trajectory_action server");
    }

    // Create kinematic robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();

    // Initialize robot
    RobotControl robot_control(kinematic_model, traj_client);

    // Generate cartesian path (wrt current robot's pose)
    PathPlanning path_plan;


    auto cart_path = path_plan.generate_trajectory(robot_control.get_ee_pose());


    robot_control.goto_cartesian_position(cart_path, traj_client);

    ros::shutdown();
    return 0;
}