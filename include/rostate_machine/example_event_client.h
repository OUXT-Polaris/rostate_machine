#ifndef ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED
#define ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED

/**
 * @file example_event_client.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief example implementation using EventClient
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) 2019 OUXT Polaris
 * 
 */


// Headers in this package
#include <rostate_machine/event_client.h>

// Headers in ROS
#include <ros/ros.h>

// Headers in Boost
#include <boost/optional.hpp>

class ExampleEventClient
{
public:
    ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~ExampleEventClient();
private:
    boost::optional<rostate_machine::Event> checkMotor();
    boost::optional<rostate_machine::Event> stopMotor();
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    rostate_machine::EventClient client_;
};

#endif  //ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED