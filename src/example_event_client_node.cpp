// headers for ros
#include <ros/ros.h>

// headers in this package
#include <rostate_machine/event_client.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rostate_machine_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::spin();
    return 0;
}