#include "../include/camera_control.h"


CameraControl::CameraControl(ros::NodeHandle *nh)
{
    // Publisher for camera recording state
    m_pub_rec_state = nh->advertise<std_msgs::Bool>(m_rec_topic_name, 1000);

    // Initalize recording state
    m_rec_state.data = false;
}

void CameraControl::publish_rec_state(const ros::TimerEvent& event)
{
    m_pub_rec_state.publish(m_rec_state) ;
}
