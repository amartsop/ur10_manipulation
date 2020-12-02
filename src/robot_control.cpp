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


// Go to cartesian position (multiple points)
void RobotControl::goto_cartesian_position(const std::vector<PathID::Cartesian>&
    cartesian_state, TrajClient& traj_client)
{
    // Get end effector link id
    auto ee_link = m_kinematic_state->getLinkModel(
        m_joint_model_group->getLinkModelNames().back());

    // Valid solution
    bool valid_solution = true;

    // Create joint object 
    std::vector<PathID::Joint> joint_angles(cartesian_state.size());

    // Store jacobians for acceleration estimation
    Eigen::MatrixXd jac_prev;

    // Generate joint trajectory
    for (size_t i = 0; i < cartesian_state.size(); i++)
    {
        // Geometry message
        geometry_msgs::Pose cart_pose;

        // Set position
        cart_pose.position.x = cartesian_state[i].position[0];
        cart_pose.position.y = cartesian_state[i].position[1];
        cart_pose.position.z = cartesian_state[i].position[2];

        // Set orientation
        double phi = cartesian_state[i].euler_position[0];
        double theta = cartesian_state[i].euler_position[1];
        double psi = cartesian_state[i].euler_position[2];

        EulerRotations::Quaternions quatern =
            EulerRotations::euler_to_quaternions(phi, theta, psi);

        cart_pose.orientation.w = quatern.w; cart_pose.orientation.x = quatern.x;
        cart_pose.orientation.y = quatern.y; cart_pose.orientation.z = quatern.z;

        // Joint values in Joint format
        joint_angles[i].time = cartesian_state[i].time;

        // Solve inerse kinematics
        bool found_ik = m_kinematic_state->setFromIK(m_joint_model_group,
            cart_pose, 10, 0.2);

        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
            /******************* Set joint positions ********************/
            m_kinematic_state->copyJointGroupPositions(m_joint_model_group,
                joint_angles[i].position);

            /******************* Set joint velocities ********************/
            // Resize velocity vector
            joint_angles[i].velocity.resize(joint_angles[i].position.size());

            // Find jacobian
            Eigen::MatrixXd jac;

            Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

            bool found_jac = m_kinematic_state->getJacobian(m_joint_model_group, 
                ee_link, reference_point_position, jac);

            // Linear velocity (cartesian)
            Eigen::Vector3d lin_vel(cartesian_state[i].velocity[0],
                cartesian_state[i].velocity[1], cartesian_state[i].velocity[2]);

            // Derivative of Euler angles
            Eigen::Vector3d theta_dot(cartesian_state[i].euler_velocity[0],
                cartesian_state[i].euler_velocity[1],
                cartesian_state[i].euler_velocity[2]);
        
            // Rotational velocity body frame (rad/sec)
            Eigen::Vector3d omega =
                EulerRotations::G(cartesian_state[i].euler_position) * theta_dot;

            // Concatenate cartesian state velocity vector
            Eigen::VectorXd x_dot(lin_vel.size() + omega.size());
            x_dot << lin_vel, omega;

            // Get desired joint velocity
            Eigen::VectorXd q_dot = jac.householderQr().solve(x_dot);

            joint_angles[i].velocity.resize(q_dot.size());

            for(size_t j = 0; j < q_dot.size(); j++)
            {
            joint_angles[i].velocity.at(j) = q_dot(j);
            }

            /******************* Set joint acceleration ********************/
            // Resize acceleration vector
            joint_angles[i].acceleration.resize(joint_angles[i].position.size());

            // Linear acceleration (cartesian)
            Eigen::Vector3d lin_accel(cartesian_state[i].acceleration[0],
                cartesian_state[i].acceleration[1], cartesian_state[i].acceleration[2]);

            // Second derivative of Euler angles
            Eigen::Vector3d theta_ddot(cartesian_state[i].euler_acceleration[0],
                cartesian_state[i].euler_acceleration[1],
                cartesian_state[i].euler_acceleration[2]);

            // Rotational acceleration body frame (rad/sec^2)
            Eigen::Vector3d omega_dot =
                EulerRotations::G(cartesian_state[i].euler_position) * theta_ddot 
                + EulerRotations::G_dot(cartesian_state[i].euler_position, 
                cartesian_state[i].euler_velocity) * theta_dot;

            // Concatenate cartesian state acceleration vector
            Eigen::VectorXd x_ddot(lin_accel.size() + omega_dot.size());
            x_ddot << lin_accel, omega_dot;

            // Calculate time derivative of jacobian matrix
            Eigen::MatrixXd jac_der;
            if (i == 0 || i == cartesian_state.size() - 1 )
            {
                jac_der.setZero(jac.cols(), jac.rows());
            }
            else
            {
                // Calculate time difference
                double h = joint_angles[i].time - joint_angles[i-1].time;

                // Estimate jacobian derivative (forward euler)
                jac_der = (jac - jac_prev) / h;
            }

            // Update jacobian 
            jac_prev = jac;

            // Calculate b (Spong p. 141)
            Eigen::VectorXd b = x_ddot - jac_der * q_dot;

            // Calculate desired joint acceleration
            Eigen::VectorXd q_ddot = jac.householderQr().solve(b);

            for(size_t j = 0; j < q_dot.size(); j++)
            {
            joint_angles[i].acceleration.at(j) = q_ddot(j);
            }
        }
        else
        {
            valid_solution = false;
            break;
        }

        // Print current state
        ROS_INFO("%d out of: %d", i, cartesian_state.size());
    }

    if(valid_solution)
    {
        ROS_INFO("Path found! Generating position...");
        goto_joint_position(joint_angles, traj_client);
    }
    else
    {
        ROS_INFO("Inverse kinematics failed!");
    }
}



// Go to cartesian position (one point)
void RobotControl::goto_cartesian_position(const PathID::Cartesian&
    cartesian_state, TrajClient& traj_client)
{
    // Get end effector link id
    auto ee_link = m_kinematic_state->getLinkModel(
        m_joint_model_group->getLinkModelNames().back());

    // Geometry message
    geometry_msgs::Pose cart_pose;

    // Set position
    cart_pose.position.x = cartesian_state.position[0];
    cart_pose.position.y = cartesian_state.position[1];
    cart_pose.position.z = cartesian_state.position[2];

    // Set orientation
    EulerRotations::Quaternions quatern =
        EulerRotations::euler_to_quaternions(cartesian_state.euler_position[0], 
            cartesian_state.euler_position[1], cartesian_state.euler_position[2]);

    cart_pose.orientation.w = quatern.w;
    cart_pose.orientation.x = quatern.x;
    cart_pose.orientation.y = quatern.y;
    cart_pose.orientation.z = quatern.z;

    // Joint values in Joint format
    PathID::Joint joints_values;
    joints_values.time = cartesian_state.time;
    
    // Solve inverse kinematics
    bool found_ik = m_kinematic_state->setFromIK(m_joint_model_group,
        cart_pose, 0.2);


    // Now, we can print out the IK solution (if found):
    if (found_ik)
    {
        /******************* Set joint positions ********************/
        m_kinematic_state->copyJointGroupPositions(m_joint_model_group,
            joints_values.position);
        
        /******************* Set joint velocities ********************/
        // Resize velocity vector
        joints_values.velocity.resize(joints_values.position.size());

        // Find jacobian
        Eigen::MatrixXd jac;

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);

        bool found_jac = m_kinematic_state->getJacobian(m_joint_model_group, 
            ee_link, reference_point_position, jac);

        // Linear velocity (cartesian)
        Eigen::Vector3d lin_vel(cartesian_state.velocity[0],
            cartesian_state.velocity[1], cartesian_state.velocity[2]);

        // Derivative of Euler angles
        Eigen::Vector3d theta_dot(cartesian_state.euler_velocity[0],
            cartesian_state.euler_velocity[1], cartesian_state.euler_velocity[2]);
    
        // Rotational velocity body frame (rad/sec)
        Eigen::Vector3d omega =
            EulerRotations::G(cartesian_state.euler_position) * theta_dot;

        // Concatenate cartesian state vector
        Eigen::VectorXd x_dot(lin_vel.size() + omega.size());
        x_dot << lin_vel, omega;

        // Get desired joint velocity
        Eigen::VectorXd q_dot = jac.householderQr().solve(x_dot);

        joints_values.velocity.resize(q_dot.size());

        for(size_t i = 0; i < q_dot.size(); i++)
        {
           joints_values.velocity.at(i) = q_dot(i);
        }

        /******************* Set joint accelerations ********************/
        joints_values.acceleration.resize(joints_values.position.size());
        std::fill(joints_values.acceleration.begin(),
            joints_values.acceleration.end(), 0.0);

        // Go to joint position
        goto_joint_position(joints_values, traj_client);
    }
    else
    {
        ROS_INFO("Did not find IK solution");
    }
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
std::vector<double> RobotControl::get_ee_position(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_link_names.back());

    Eigen::Vector3d pos = end_effector_state.translation();

    // Position
    std::vector<double> pos_vec(3);
    pos_vec.at(0) = pos(0); pos_vec.at(1) = pos(1); pos_vec.at(2) = pos(2);
    
    // Return position
    return pos_vec;
}


// Get end effector orientation, euler angles phi, theta, psi (z, y', x'')
std::vector<double> RobotControl::get_ee_orientation(void)
{
    const Eigen::Affine3d &end_effector_state =
        m_kinematic_state->getGlobalLinkTransform(m_link_names.back());

    Eigen::Matrix3d rot = end_effector_state.rotation();
    Eigen::Quaterniond quatern(rot);

    // Quaternions to euler angles
    EulerRotations::Euler eul =
        EulerRotations::quaternions_to_euler(quatern.w(), quatern.x(),
        quatern.y(), quatern.z());

    std::vector<double> eul_vec(3);
    eul_vec.at(0) = eul.phi; eul_vec.at(1) = eul.theta; eul_vec.at(2) = eul.psi;

    // Return orientation
    return eul_vec;
}


// Get end-effector pose
PathID::Cartesian RobotControl::get_ee_pose(void)
{
    // Get end-effector position
    std::vector<double> lin_pos = get_ee_position();

    // Get end-effector orientation
    std::vector<double> euler = get_ee_orientation();

    // Cartesian pose 
    PathID::Cartesian pose;

    // Set position
    pose.position = lin_pos;

    // Set orientation
    pose.euler_position = euler;

    // DEBUG -> Should find a way of changing velocities and acceleration
    return pose;
}



// Get joint angles values
std::vector<double> RobotControl::get_joint_angles(void)
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
