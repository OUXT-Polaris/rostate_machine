#ifndef ROSTATE_MACHINE_H_INCLUDED
#define ROSTATE_MACHINE_H_INCLUDED

//headers in robotx_packages
#include "state_machine.h"
#include <rostate_machine/State.h>
#include <rostate_machine/StateChanged.h>
#include <rostate_machine/Event.h>

//headers in STL
#include <memory>

//headers in ROS
#include <ros/ros.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

class RostateMachine
{
public:
    RostateMachine(std::string xml_filepath, std::string dot_filepath, std::string state_machine_name);
    ~RostateMachine();
    void run();
private:
    void publish_current_state_();
    void event_callback_(const ros::MessageEvent<rostate_machine::Event const>& event);
    std::shared_ptr<state_machine> state_machine_ptr_;
    ros::NodeHandle nh_;
    ros::Publisher current_state_pub_;
    ros::Publisher state_changed_pub_;
    ros::Subscriber trigger_event_sub_;
    std::string state_machine_name_;
    double publish_rate_;
};

#endif //ROSTATE_MACHINE_H_INCLUDED