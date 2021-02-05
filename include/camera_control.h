#pragma once 

#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sstream>

class CameraControl
{

public:
    // Constructor
    CameraControl(ros::NodeHandle *nh);

    // Publish recording state
    void publish_rec_state(const ros::TimerEvent& event);

    // Return the publishing rate of recording interval
    double get_rec_state_publishing_rate(void) { return m_pub_rec_interval; }

    // Set the recording state
    void set_rec_state(bool state) { m_rec_state.data = state; }

private:

    // Publisher handle
    ros::Publisher m_pub_rec_state;

    // Topic name 
    const std::string m_rec_topic_name = "camera_recording_state";

    // Recording state publishing interval
    const double m_pub_rec_interval = 0.002;

    // Recording state
    std_msgs::Bool m_rec_state;

};

