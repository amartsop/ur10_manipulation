
#include "../include/ft_sensor.h"


void FTSensor::ft_callback(const robotiq_ft_sensor::ft_sensorConstPtr& msg)
{
    // Set forces and torques
    m_force_fx = (double) msg->Fx;
    m_force_fy = (double) msg->Fy;
    m_force_fz = (double) msg->Fz;

    m_torque_mx = (double) msg->Mx;
    m_torque_my = (double) msg->My;
    m_torque_mz = (double) msg->Mz;

    ROS_INFO("test: %f", m_force_fz);
}
