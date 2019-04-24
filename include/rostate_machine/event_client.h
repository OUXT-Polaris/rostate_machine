#ifndef ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED
#define ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED

// Headers in this package
#include <rostate_machine/State.h>
#include <rostate_machine/StateChanged.h>
#include <rostate_machine/Event.h>

// Headers in ROS
#include <ros/ros.h>

// Headers in STL
#include <functional>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace rostate_machine
{
    class EventClient
    {
    public:
        EventClient(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~EventClient();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Publisher trigger_event_pub_;
        ros::Subscriber current_state_sub_;
        void stateCallback(const rostate_machine::State::ConstPtr msg);
        boost::circular_buffer<rostate_machine::State> state_buf_;
        std::string target_state_machine_namespace_;
        std::string xml_filepath_;
        void loadXml();
        void onTransition();
    };
}

#endif  //ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED