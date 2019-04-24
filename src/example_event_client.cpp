// Headers in this package
#include <rostate_machine/example_event_client.h>

ExampleEventClient::ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh) : client_(nh,pnh)
{
    nh_ = nh;
    pnh_ = pnh;
}

ExampleEventClient::~ExampleEventClient()
{

}