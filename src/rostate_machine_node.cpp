/**
 * @file rostate_machine_node.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief state_machine node for ROS
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c)  OUXT Polaris 2019
 * 
 */

// headers for ros
#include <ros/ros.h>

// headers in this package
#include <rostate_machine/rostate_machine.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rostate_machine_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RostateMachine state_machine(nh,pnh);
    state_machine.run();
    ros::spin();
    return 0;
}