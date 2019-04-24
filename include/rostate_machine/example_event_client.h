#ifndef ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED
#define ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED

// Headers in this package
#include <rostate_machine/event_client.h>

// Headers in ROS
#include <ros/ros.h>

class ExampleEventClient
{
public:
    ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh);
    ~ExampleEventClient();
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    rostate_machine::EventClient client_;
};

#endif  //ROSTATE_MACHINE_EXAMPLE_EVENT_CLIENT_H_INCLUDED