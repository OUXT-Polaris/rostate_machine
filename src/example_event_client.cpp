// Headers in this package
#include <rostate_machine/example_event_client.h>

ExampleEventClient::ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh) : client_(nh,pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    client_.registerCallback(std::bind(&ExampleEventClient::stopMotor, this),"stop_motor");
    client_.registerCallback(std::bind(&ExampleEventClient::checkMotor, this),"check_motor");
    client_.run();
}

ExampleEventClient::~ExampleEventClient()
{

}

boost::optional<rostate_machine::Event> ExampleEventClient::checkMotor()
{
    rostate_machine::Event ret;
    ret.header.stamp = ros::Time::now();
    ret.trigger_event_name = "motor_disconnected";
    return ret;
}

boost::optional<rostate_machine::Event> ExampleEventClient::stopMotor()
{
    rostate_machine::Event ret;
    ret.header.stamp = ros::Time::now();
    ret.trigger_event_name = "stop_motor";
    return ret;
}