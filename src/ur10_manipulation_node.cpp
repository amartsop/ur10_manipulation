#include <iostream>

#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "../include/robot_control.h"
#include "../include/path_planning.h"
#include "../include/path_id.h"


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


    // PathPlanning path_planning(robot_control.get_ee_position(),
    //     robot_control.get_ee_orientation());

    // auto pos_vec = path_planning.get_position_vector();
    // auto eul_vec = path_planning.get_euler_vector();


    // robot_control.goto_cartesian_position(0.5, pos_vec.at(150), eul_vec.at(150),
    //     traj_client);


    PathID::Joint path;
    path.time = 0.5;
    path.position = {0.0, -1.7279, -1.9279, 0.0, 1.57, 0.0};
    path.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    path.acceleration = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // robot_control.goto_joint_position(path, traj_client);


    // const std::vector<double> m_init_pos = {0.0, -1.7279, -1.7279,
    //     0.0, 3.14, 0.0};
    // vecd m_init_vel = 
    // vecd m_init_accel = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // robot_control.goto_joint_position(0.5, m_init_pos, m_init_vel, m_init_accel,
    //     traj_client);


    ros::shutdown();
    return 0;
}


    // // Pointer that handles the robot's state
    // robot_state::RobotStatePtr kinematic_state(new
    //     robot_state::RobotState(kinematic_model));

    // // Create joint model group
    // const robot_state::JointModelGroup* joint_model_group =
    //     kinematic_model->getJointModelGroup("manipulator");

    // // Control handle
    // RobotControl robot_control(joint_model_group);



    // std::vector<std::vector<double>> pos(2);
    // std::vector<std::vector<double>> vel(2);
    // std::vector<std::vector<double>> accel(2);

    // pos.at(0) = {1.6, -0.0, -1.57, 0.0, 0.0, 0.0};
    // vel.at(0) = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // accel.at(0) = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // pos.at(1) = {0.0, -0.0, -0.0, 0.0, 0.0, 0.0};
    // vel.at(1) = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // accel.at(1) = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


    // std::vector<double> t = {2.0, 4.0};

    // robot_control.goto_joint_position(t, pos, vel, accel, traj_client,
    //     kinematic_state);


    // // Get the position and orientation of the end-effector
    // const std::vector<std::string> &link_names = 
    //     joint_model_group->getLinkModelNames();

    // const Eigen::Affine3d &end_effector_state =
    //     kinematic_state->getGlobalLinkTransform(link_names.back());

    // auto translation = end_effector_state.translation();

    // std::cout << translation(0) << std::endl;
    // std::cout << translation(1) << std::endl;
    // std::cout << translation(2) << std::endl;



    // const std::vector<std::string> &joint_names =
    //     joint_model_group->getJointModelNames();

    // const std::vector<std::string> &link_names = 
    //     joint_model_group->getLinkModelNames();

    // const std::vector<double> initial_pos = {1.5, -0.2, -1.57, 0.0, 0.0, 0.0};

    // // Set joint state to initial_pos
    // kinematic_state->setJointGroupPositions(joint_model_group, initial_pos);

    // const Eigen::Affine3d &end_effector_state =
    //     kinematic_state->getGlobalLinkTransform(link_names.back());


    // // Get Joint Values
    // // ^^^^^^^^^^^^^^^^
    // // We can retreive the current set of joint values stored in the state for the right arm.
    // std::vector<double> joint_values;
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for(std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);

    // // Joint Limits
    // // ^^^^^^^^^^^^
    // // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
    // /* Set one joint in the right arm outside its joint limit */
    // joint_values[0] = 1.57;
    // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

    // /* Check whether any joint is outside its joint limits */
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // /* Enforce the joint limits for this state and check again*/
    // kinematic_state->enforceBounds();
    // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

    // // Forward Kinematics
    // // ^^^^^^^^^^^^^^^^^^
    // // Now, we can compute forward kinematics for a set of random joint
    // // values. Note that we would like to find the pose of the
    // // "r_wrist_roll_link" which is the most distal link in the
    // // "right_arm" of the robot.
    
    // kinematic_state->setToRandomPositions(joint_model_group);
    // const Eigen::Affine3d &end_effector_state =
    //     kinematic_state->getGlobalLinkTransform(link_names.back());


    // // Inverse Kinematics
    // // ^^^^^^^^^^^^^^^^^^
    // // We can now solve inverse kinematics (IK) for the right arm of the
    // // PR2 robot. To solve IK, we will need the following:
    // // * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain): end_effector_state that we computed in the step above.
    // // * The number of attempts to be made at solving IK: 5
    // // * The timeout for each attempt: 0.1 s

    // bool found_ik = kinematic_state->setFromIK(joint_model_group,
    //     end_effector_state, 10, 0.1);

    // // Now, we can print out the IK solution (if found):
    // if (found_ik)
    // {
    //     kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    //     for(std::size_t i=0; i < joint_names.size(); ++i)
    //     {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    //     }
    // }
    // else
    // {
    //     ROS_INFO("Did not find IK solution");
    // }




    // std::vector<double> joint_values;
    // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    // for(std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    //     ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }




    // JointControl joint_control;

    // joint_control.trajectory_generation();

    // /******************** Cartesian trajectory ***********************/
    // // Planning group name and moveit handle
    // const std::string planning_group_name = "manipulator";
    // moveit::planning_interface::MoveGroupInterface move_group(planning_group_name);

    // // Cartesian control handle
    // CartesianControl cartesian_control;

    // // Robot initialization routine
    // cartesian_control.robot_initialization(move_group);

    // // Generate cartesian trajectory
    // cartesian_control.trajectory_execution(move_group);

    // ros::shutdown();
    // return 0;