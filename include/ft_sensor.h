#pragma once 

#include <iostream>
#include "ros/ros.h"

#include <robotiq_ft_sensor/ft_sensor.h>


class FTSensor
{

public:
    // Force torque callback function
    void ft_callback(const robotiq_ft_sensor::ft_sensorConstPtr& msg);

private:
    // Forces 
    double m_force_fx, m_force_fy, m_force_fz;

    // Torques 
    double m_torque_mx, m_torque_my, m_torque_mz;
};

