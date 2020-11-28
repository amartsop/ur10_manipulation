#include "../include/trajectory_design.h"

void TrajectoryDesign::initialize(TrajectoryDesign::traj_type traj_type,
        const std::vector<double>& params)
{
    // Set trajectory type
    m_traj_type = traj_type;

    // Initialze trajectory 
    m_traj = {0.0, 0.0, 0.0};

    // Choose trajectory type
    switch (traj_type)
    {
    case TRAJ_SIN:
        m_sinusoid_params = params;
        break;
    default:
        break;
    }
}

double TrajectoryDesign::get_position(double t)
{
    // Update and return
    update(t); return m_traj.at(0);
}


double TrajectoryDesign::get_velocity(double t)
{
    // Update and return
    update(t); return m_traj.at(1);
}

double TrajectoryDesign::get_acceleration(double t)
{
    // Update and return
    update(t); return m_traj.at(2);
}

void TrajectoryDesign::update(double t)
{
    switch (m_traj_type)
    {
    case TRAJ_SIN:
        sinusoid_calculation(t);
        break;

    default:
        break;
    }
}


// Sinusoidal trajectory params(amplitude(m), frequency(hz), phase(rad))
void TrajectoryDesign::sinusoid_calculation(double t)
{
    // Sinusoid argument
    double ampl = m_sinusoid_params.at(0);
    double a_dot = 2.0 * M_PI * m_sinusoid_params.at(1);
    double a = a_dot * t + m_sinusoid_params.at(2);

    m_traj = { ampl * sin(a), ampl  * a_dot * cos(a),
        - ampl * powf(a_dot, 2.0) * sin(a)};
}

