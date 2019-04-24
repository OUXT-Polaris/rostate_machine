#include <rostate_machine/event_client.h>

namespace rostate_machine
{
    EventClient::EventClient(ros::NodeHandle nh,ros::NodeHandle pnh)
    {
        nh_ = nh;
        pnh_ = pnh;
        state_buf_ = boost::circular_buffer<rostate_machine::State>(2);
        pnh_.param<std::string>("target_state_machine_namespace", target_state_machine_namespace_, "");
        pnh_.param<std::string>(target_state_machine_namespace_+"/xml_filepath", xml_filepath_, "");
        trigger_event_pub_ = nh_.advertise<rostate_machine::Event>(target_state_machine_namespace_+"/trigger_event",1);
        current_state_sub_ = nh_.subscribe(target_state_machine_namespace_+"/current_state",1,&EventClient::stateCallback,this);
    }

    EventClient::~EventClient()
    {

    }

    void EventClient::stateCallback(const rostate_machine::State::ConstPtr msg)
    {
        state_buf_.push_back(*msg);
        return;
    }

    void EventClient::loadXml()
    {

    }
}