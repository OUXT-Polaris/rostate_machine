/**
 * @file rostate_machine.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief ROS Wrapper for the State Machine Library
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c)  OUXT Polaris 2019
 * 
 */

#include <rostate_machine/rostate_machine.h>

/**
 * @brief Construct a new Rostate Machine:: Rostate Machine object
 * 
 * @param nh NodeHandle
 * @param pnh Private NodeHandle
 */
RostateMachine::RostateMachine(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("state_machine_name", state_machine_name_, "");
    pnh_.param<std::string>("dot_filepath", dot_filepath_, "");
    pnh_.param<std::string>("xml_filepath", xml_filepath_, "");
}

/**
 * @brief Destroy the Rostate Machine:: Rostate Machine object
 * 
 */
RostateMachine::~RostateMachine()
{

}

/**
 * @brief Callback Function for Trigger Event Topic
 * 
 * @param event Event Message for the rostate_machine package.
 */
void RostateMachine::eventCallback(const ros::MessageEvent<rostate_machine::Event const>& event)
{
    rostate_machine::Event msg = *event.getMessage();
    StateInfo old_info = state_machine_ptr_->getStateInfo();
    bool result = state_machine_ptr_->tryTransition(msg.trigger_event_name);
    StateInfo info = state_machine_ptr_->getStateInfo();
    if(!result)
    {
        std::string publisher_name = event.getPublisherName();
        ROS_INFO_STREAM( "from : " << publisher_name << ", failed to transition, current state : "<< info.current_state << ", event_name : " << msg.trigger_event_name);
    }
    else
    {
        std::string publisher_name = event.getPublisherName();
        ROS_INFO_STREAM( "from : " << publisher_name << ", succeed to transition, current state : "<< info.current_state << ", event_name : " << msg.trigger_event_name);
    }
    return;
}

/**
 * @brief Run State Machine (Enable Transitions)
 * 
 */
void RostateMachine::run()
{
    state_machine_ptr_ = std::make_shared<StateMachine>(xml_filepath_);
    state_machine_ptr_->drawStateMachine(dot_filepath_);
    state_machine_name_ = state_machine_name_;
    nh_.param<double>(ros::this_node::getName()+"/publish_rate", publish_rate_, 10);
    //dot_string_pub_ = nh_.advertise<std_msgs::String>(ros::this_node::getName()+"/dot_string",1);
    current_state_pub_ = nh_.advertise<rostate_machine::State>(ros::this_node::getName()+"/current_state",1);

    boost::thread publish_thread(boost::bind(&RostateMachine::publishCurrentState, this));
    trigger_event_sub_ = nh_.subscribe(ros::this_node::getName()+"/trigger_event", 10, &RostateMachine::eventCallback,this);
    return;
}

/**
 * @brief Publish Current State Topic
 * 
 */
void RostateMachine::publishCurrentState()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
        //std_msgs::String dot_string_msg;
        //dot_string_msg.data = state_machine_ptr_->getDotString();
        //dot_string_pub_.publish(dot_string_msg);
        rostate_machine::State state_msg;
        StateInfo info = state_machine_ptr_->getStateInfo();
        state_msg.current_state = info.current_state;
        state_msg.possible_transitions = info.possibe_transitions;
        state_msg.possible_transition_states = info.possibe_transition_states;
        state_msg.header.stamp = ros::Time::now();
        current_state_pub_.publish(state_msg);
        rate.sleep();
    }
    return;
}