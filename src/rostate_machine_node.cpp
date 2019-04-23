// headers for ros
#include <ros/ros.h>

// headers in this package
#include <rostate_machine/rostate_machine.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_navigation_planner_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    RostateMachine state_machine(nh,pnh);
    state_machine.run();
    ros::spin();
    return 0;
}