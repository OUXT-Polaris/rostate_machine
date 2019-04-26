#ifndef ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED
#define ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED

/**
 * @file event_client.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Event Client for Rostate Machine
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) OUXT Polaris 2019
 * 
 */

// Headers in this package
#include <rostate_machine/State.h>
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
    constexpr int always = 0;
    constexpr int on_entry = 1;
    constexpr int on_exit = 2;

    struct TagInfo
    {
        const std::string tag;
        const int when;
        const std::vector<std::string> states;
        TagInfo(std::string tag,int when,std::vector<std::string> states) 
            : tag(tag),when(when),states(states) {}
    };

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
        std::vector<std::string> split(const std::string &s, char delim);
        std::map<std::string, std::vector<std::function<boost::optional<rostate_machine::Event>(void)> > > tagged_functions_;
        std::vector<TagInfo> tag_info_;
    };
}

#endif  //ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED