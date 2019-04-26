/**
 * @file event_client.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Event Client for Rostate Machine
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) OUXT Polaris 2019
 * 
 */

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
    }

    void EventClient::run()
    {
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
        std::vector<std::string> active_tags;
        if(state_buf_.size() == 1)
        {
            for(auto tag_itr = tag_info_.begin(); tag_itr != tag_info_.end(); tag_itr++)
            {
                for(auto state_itr = tag_itr->states.begin(); state_itr != tag_itr->states.end(); state_itr++)
                {
                    if(tag_itr->when == always && *state_itr == state_buf_[0].current_state)
                    {
                        active_tags.push_back(tag_itr->tag);
                    }
                }
            }
        }
        if(state_buf_.size() == 2)
        {
            for(auto tag_itr = tag_info_.begin(); tag_itr != tag_info_.end(); tag_itr++)
            {
                for(auto state_itr = tag_itr->states.begin(); state_itr != tag_itr->states.end(); state_itr++)
                {
                    if(tag_itr->when == always && *state_itr == state_buf_[1].current_state)
                    {
                        active_tags.push_back(tag_itr->tag);
                    }
                }
            }
            if(state_buf_[0].current_state != state_buf_[1].current_state)
            {
                for(auto tag_itr = tag_info_.begin(); tag_itr != tag_info_.end(); tag_itr++)
                {
                    for(auto state_itr = tag_itr->states.begin(); state_itr != tag_itr->states.end(); state_itr++)
                    {
                        if(tag_itr->when == on_entry && *state_itr == state_buf_[1].current_state)
                        {
                            active_tags.push_back(tag_itr->tag);
                        }
                        if(tag_itr->when == on_exit && *state_itr == state_buf_[0].current_state)
                        {
                            active_tags.push_back(tag_itr->tag);
                        }
                    }
                }
            }
        }
        for(auto tag_itr = active_tags.begin(); tag_itr != active_tags.end(); tag_itr++)
        {
            std::vector<std::function<boost::optional<rostate_machine::Event>(void)> > execute_funcs = tagged_functions_[*tag_itr];
            for(auto func_itr = execute_funcs.begin(); func_itr != execute_funcs.end(); func_itr++)
            {
                std::function<boost::optional<rostate_machine::Event>(void)> func = *func_itr;
                boost::optional<rostate_machine::Event> ret = func();
                if(ret)
                {
                    trigger_event_pub_.publish(*ret);
                }
            }
        }
        return;
    }

    void EventClient::loadXml()
    {
        using namespace boost::property_tree;
        ptree pt;
        xml_parser::read_xml(xml_filepath_, pt);
        for (const ptree::value_type& state_itr : pt.get_child("state_machine"))
        {
            if(state_itr.first == "callback")
            {
                std::string tag = state_itr.second.get<std::string>("<xmlattr>.tag");
                std::string when = state_itr.second.get<std::string>("<xmlattr>.when");
                std::vector<std::string> states = split(state_itr.second.get<std::string>("<xmlattr>.states"),',');
                bool is_matched = false;
                if(when == "always")
                {
                    TagInfo info = TagInfo(tag,always,states);
                    tag_info_.push_back(info);
                    is_matched = true;
                }
                if(when == "on_entry")
                {
                    TagInfo info = TagInfo(tag,on_entry,states);
                    tag_info_.push_back(info);
                    is_matched = true;
                }
                if(when == "on_exit")
                {
                    TagInfo info = TagInfo(tag,on_exit,states);
                    tag_info_.push_back(info);
                    is_matched = true;
                }
                if(is_matched == false)
                {
                    ROS_WARN_STREAM("unspoorted when attribute!");
                }
            }
        }
        return;
    }

    void EventClient::registerCallback(std::function<boost::optional<rostate_machine::Event>(void)> func,std::string tag)
    {
        tagged_functions_[tag].push_back(func);
        return;
    }

    std::vector<std::string> EventClient::onTransition()
    {
        std::vector<std::string> tags;
        return tags;
    }


    std::vector<std::string> EventClient::split(const std::string &s, char delim)
    {
        std::vector<std::string> elems;
        std::stringstream ss(s);
        std::string item;
        while (getline(ss, item, delim))
        {
            if (!item.empty())
            {
                elems.push_back(item);
            }
        }
        return elems;
    }
}