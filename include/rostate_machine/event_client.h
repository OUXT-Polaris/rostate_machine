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

namespace rostate_machine
{
    template <typename T>
    boost::optional<rostate_machine::Event> eventCallbackFunc(T);

    class EventClient
    {
    public:
        EventClient(ros::NodeHandle nh,ros::NodeHandle pnh);
        ~EventClient();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber current_state_sub_;
        void stateCallback(const rostate_machine::State::ConstPtr msg);
        boost::circular_buffer<rostate_machine::State> state_buf_;
    };
}

#endif  //ROSTATE_MACHINE_EVENT_CLIENT_H_INCLUDED