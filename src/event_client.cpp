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
        loadXml();
        trigger_event_pub_ = nh_.advertise<rostate_machine::Event>(target_state_machine_namespace_+"/trigger_event",1);
        current_state_sub_ = nh_.subscribe(target_state_machine_namespace_+"/current_state",1,&EventClient::stateCallback,this);
    }

    EventClient::~EventClient()
    {

    }

    void EventClient::stateCallback(const rostate_machine::State::ConstPtr msg)
    {
        state_buf_.push_back(*msg);
        if(state_buf_.size() == 2)
        {
            if(state_buf_[0].current_state != state_buf_[1].current_state)
            {
                onTransition();
            }
        }
        return;
    }

    void EventClient::loadXml()
    {
        boost::property_tree::ptree pt;
        boost::property_tree::xml_parser::read_xml(xml_filepath_, pt);
        return;
    }

    void EventClient::onTransition()
    {
        return;
    }
}