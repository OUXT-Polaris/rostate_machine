/**
 * @file example_event_client_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Example node for using event_client
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c)  OUXT Polaris 2019
 * 
 */

// Headers for ros
#include <ros/ros.h>

// Headers in this package
#include <rostate_machine/example_event_client.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rostate_machine_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ExampleEventClient client(nh,pnh);
    ros::spin();
    return 0;
}