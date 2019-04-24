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