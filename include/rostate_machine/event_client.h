#ifndef ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED
#define ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED

// Headers in this package
#include <rostate_machine/callback_func.h>
#include <rostate_machine/State.h>
#include <rostate_machine/StateChanged.h>
#include <rostate_machine/Event.h>

// Headers in ROS
#include <ros/ros.h>

// Headers in Boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

// Headers in STL
#include <vector>
#include <map>

namespace rostate_machine
{
    class EventClient
    {
    public:
        EventClient(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~EventClient();
        void registerCallback(std::function<boost::optional<rostate_machine::Event>(void)> func,std::string tag);
        void run();
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
        std::vector<std::string> onTransition();
        std::vector<CallbackFunc> callbacks_;
        std::map<std::string, std::vector<std::function<boost::optional<rostate_machine::Event>(void)> > > function_lists_;
        std::vector<std::string> split(const std::string &s, char delim);
    };
}

#endif  //ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED