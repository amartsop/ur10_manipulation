#include "../include/robot_control.h"


RobotControl::RobotControl(robot_model::RobotModelPtr kinematic_model,
    TrajClient& traj_client)
{
    // Kinematic state handle
    m_kinematic_state = new robot_state::RobotState(kinematic_model);

    // Create joint model group
    m_joint_model_group = kinematic_model->getJointModelGroup("manipulator");

    // Set joint and link names 
    m_joint_names = m_joint_model_group->getActiveJointModelNames();
    m_link_names = m_joint_model_group->getLinkModelNames();

    // Initialize robot
    PathID::Joint init_path;
    init_path.time = m_t_init;
    init_path.position = m_init_pos;
    init_path.velocity = m_init_vel;
    init_path.acceleration = m_init_accel;
    goto_joint_position(init_path, traj_client);

}

void RobotControl::goto_cartesian_position(double t, const vecd& pos,
    const vecd& euler, TrajClient& traj_client)
{
    geometry_msgs::Pose cart_pose;

    // Set position
    cart_pose.position.x = pos.at(0);
    cart_pose.position.y = pos.at(1);
    cart_pose.position.z = pos.at(2);

    // Set orientation
    AngleConversion::Quaternions quatern =
        AngleConversion::euler_to_quaternions(euler.at(0), euler.at(1),
        euler.at(2));

    cart_pose.orientation.w = quatern.w;
    cart_pose.orientation.x = quatern.x;
    cart_pose.orientation.y = quatern.y;
    cart_pose.orientation.z = quatern.z;


    bool found_ik = m_kinematic_state->setFromIK(m_joint_model_group,
        cart_pose, 10, 0.1);

    vecd joint_values;

    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        m_kinematic_state->copyJointGroupPositions(m_joint_model_group, joint_values);

        for(std::size_t i=0; i < m_joint_names.size(); ++i)
        {
        ROS_INFO("Joint %s: %f", m_joint_names[i].c_str(), joint_values[i]);
        }
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }


    // goto_joint_position(t, joint_values, m_init_vel, m_init_accel, traj_client);
}


// Go to joint position (one point)
void RobotControl::goto_joint_position(const PathID::Joint& joint_state,
    TrajClient& traj_client)
{
    // Create goal 
    fktg goal = set_joint_state_goal(joint_state);

    // Send goal to client
    traj_client.sendGoal(goal);

    while (!traj_client.getState().isDone())
    {
        ros::spinOnce();
    }
}


// Go to joint positions (multiple points)
void RobotControl::goto_joint_position(const std::vector<PathID::Joint>&
    joint_state, TrajClient& traj_client)
{
    // Create goal 
    fktg goal = set_joint_state_goal(joint_state);

    // Send goal to client
    traj_client.sendGoal(goal);

    while (!traj_client.getState().isDone())
    {
        ros::spinOnce();
    }
}


// Get end effector position
vecd RobotControl::get_ee_position(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_link_names.back());

    Eigen::Vector3d pos = end_effector_state.translation();

    // Position
    vecd pos_vec(3);
    pos_vec.at(0) = pos(0); pos_vec.at(1) = pos(1); pos_vec.at(2) = pos(2);
    
    // Return position
    return pos_vec;
}


// Get end effector orientation, euler angles phi, theta, psi (z, y', x'')
vecd RobotControl::get_ee_orientation(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_link_names.back());

    Eigen::Matrix3d rot = end_effector_state.rotation();
    Eigen::Quaterniond quatern(rot);

    // Quaternions to euler angles
    AngleConversion::Euler eul =
        AngleConversion::quaternions_to_euler(quatern.w(), quatern.x(),
        quatern.y(), quatern.z());

    vecd eul_vec(3);
    eul_vec.at(0) = eul.phi; eul_vec.at(1) = eul.theta; eul_vec.at(2) = eul.psi;

    // Return orientation
    return eul_vec;
}


// Get joint angles values
vecd RobotControl::get_joint_angles(void)
{
    std::vector<double> joint_values;
    m_kinematic_state->copyJointGroupPositions(m_joint_model_group, joint_values);
    return joint_values;
}


// Set joint state goal (one point)
fktg RobotControl::set_joint_state_goal(const PathID::Joint& joint_state)
{
    // Create goal object 
    fktg goal;

    // Robot Initialization routine
    goal.trajectory.joint_names = m_joint_names;

    goal.trajectory.points.resize(1);

    // Set position
    goal.trajectory.points[0].positions = joint_state.position;
    m_kinematic_state->setJointGroupPositions(m_joint_model_group,
        joint_state.position);

    // Set velocity
    goal.trajectory.points[0].velocities = joint_state.velocity;
    m_kinematic_state->setJointGroupVelocities(m_joint_model_group,
        joint_state.velocity);

    // Set acceleration
    goal.trajectory.points[0].accelerations = joint_state.acceleration;
    m_kinematic_state->setJointGroupAccelerations(m_joint_model_group,
        joint_state.acceleration);

    // Set time from start
    goal.trajectory.points[0].time_from_start = ros::Duration(joint_state.time);

    return goal;
}


// Set joint state goal (multiple points)
fktg RobotControl::set_joint_state_goal(const std::vector<PathID::Joint>&
    joint_state)
{
    // Create goal object 
    fktg goal;

    // Resize goal number
    goal.trajectory.points.resize(joint_state.size());

    // Set joint names
    goal.trajectory.joint_names = m_joint_names;

    for(size_t i = 0; i < joint_state.size(); i++)
    {
        // Set position
        goal.trajectory.points[i].positions = joint_state.at(i).position;
        m_kinematic_state->setJointGroupPositions(m_joint_model_group,
            joint_state.at(i).position);

        // Set velocity
        goal.trajectory.points[i].velocities = joint_state.at(i).velocity;
        m_kinematic_state->setJointGroupVelocities(m_joint_model_group,
            joint_state.at(i).velocity);

        // Set acceleration
        goal.trajectory.points[i].accelerations = joint_state.at(i).acceleration;
        m_kinematic_state->setJointGroupAccelerations(m_joint_model_group,
            joint_state.at(i).acceleration);

        // Set time from start
        goal.trajectory.points[i].time_from_start =
            ros::Duration(joint_state.at(i).time);
    }

    return goal;
}


// Destructor
RobotControl::~RobotControl()
{
    delete m_kinematic_state;
}
