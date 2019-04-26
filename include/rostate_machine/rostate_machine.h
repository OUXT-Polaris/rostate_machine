#ifndef ROSTATE_MACHINE_H_INCLUDED
#define ROSTATE_MACHINE_H_INCLUDED

/**
 * @file rostate_machine.h
 * @author masaya Kataoka (ms.kataoka@gmail.com)
 * @brief ROS Wrapper for the State Machine Class
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) OUXT Polaris 2019
 * 
 */

//headers in this package
#include "state_machine.h"
#include <rostate_machine/State.h>
#include <rostate_machine/Event.h>

//headers in STL
#include <memory>

//headers in ROS
#include <ros/ros.h>
#include <std_msgs/String.h>

//headers in Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

/**
 * @brief ROS wrapper of the State Machine Class
 * @sa StateMachine
 */
class RostateMachine
{
public:
    RostateMachine(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~RostateMachine();
    void run();
private:
    void publishCurrentState();
    void eventCallback(const ros::MessageEvent<rostate_machine::Event const>& event);
    std::shared_ptr<StateMachine> state_machine_ptr_;
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    //ros::Publisher dot_string_pub_;
    ros::Publisher current_state_pub_;
    ros::Subscriber trigger_event_sub_;
    std::string state_machine_name_;
    std::string xml_filepath_;
    std::string dot_filepath_;
    double publish_rate_;
};

#endif //ROSTATE_MACHINE_H_INCLUDED