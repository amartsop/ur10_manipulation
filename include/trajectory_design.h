#pragma once 

#include <iostream>
#include <math.h>
#include <vector>

class TrajectoryDesign
{

public:

    TrajectoryDesign(){};

    enum traj_type{TRAJ_SIN, TRAJ_COS, TRAJ_LINE};

    void initialize(TrajectoryDesign::traj_type traj_type,
        const std::vector<double>& params);

    double get_position(double t);
    double get_velocity(double t);
    double get_acceleration(double t);

private:
    // Trajectory type 
    traj_type m_traj_type;

    // Update trajectory 
    void update(double t);

    // Trajectory params
    std::vector<double> m_params;

    // Displacement (m)
    double m_displacement;

    // Displacement (m/s)
    double m_velocity;

    // Displacement (m/s^2)
    double m_acceleration;

    // Trajectory (displacement, velocity, acceleration)
    std::vector<double> m_traj;

    /************** Sinusoid ***************/
    std::vector<double> m_sinusoid_params = {0.0, 0.0, 0.0};
    void sinusoid_calculation(double t);

};