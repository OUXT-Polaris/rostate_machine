
#include <rostate_machine/rostate_machine.h>

RostateMachine::RostateMachine(ros::NodeHandle nh,ros::NodeHandle pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    pnh_.param<std::string>("state_machine_name", state_machine_name_, "");
    //pnh_.param<std::string>("dot_filepath", dot_filepath_, "");
    pnh_.param<std::string>("xml_filepath", xml_filepath_, "");
}

RostateMachine::~RostateMachine()
{

}

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
        rostate_machine::StateChanged state_changed_msg;
        state_changed_msg.current_state = info.current_state;
        state_changed_msg.old_state = old_info.current_state;
        state_changed_msg.triggered_event = msg.trigger_event_name;
        state_changed_pub_.publish(state_changed_msg);
    }
    return;
}

void RostateMachine::run()
{
    state_machine_ptr_ = std::make_shared<StateMachine>(xml_filepath_);
    //state_machine_ptr_->drawStateMachine(dot_filepath_);
    state_machine_name_ = state_machine_name_;
    nh_.param<double>(ros::this_node::getName()+"/publish_rate", publish_rate_, 10);
    current_state_pub_ = nh_.advertise<rostate_machine::State>(ros::this_node::getName()+"/"+state_machine_name_+"/current_state",1);
    state_changed_pub_ = nh_.advertise<rostate_machine::StateChanged>(ros::this_node::getName()+"/"+state_machine_name_+"/state_changed",1);

    boost::thread publish_thread(boost::bind(&RostateMachine::publishCurrentState, this));
    trigger_event_sub_ = nh_.subscribe(ros::this_node::getName()+"/"+state_machine_name_+"/trigger_event", 10, &RostateMachine::eventCallback,this);
    return;
}

void RostateMachine::publishCurrentState()
{
    ros::Rate rate(publish_rate_);
    while(ros::ok())
    {
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