/**
 * @file example_event_client.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief example implementation using EventClient
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) 2019 OUXT Polaris
 * 
 */

// Headers in this package
#include <rostate_machine/example_event_client.h>

/**
 * @brief Construct a new Example Event Client:: Example Event Client object
 * 
 * @param nh Node Handler
 * @param pnh Private Node Handle
 */
ExampleEventClient::ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh) : client_(nh,pnh,"example_state_machine_node")
{
    nh_ = nh;
    pnh_ = pnh;
    /**
     * @brief register callback for the tag in StateMachine .xml file
     * 
     */
    client_.registerCallback(std::bind(&ExampleEventClient::stopMotor, this),"stop_motor");
    client_.registerCallback(std::bind(&ExampleEventClient::checkMotor, this),"check_motor");
    client_.registerCallback(std::bind(&ExampleEventClient::checkRecovery, this),"check_recovery");
    /**
     * @brief start client
     * 
     */
    client_.run();
}

ExampleEventClient::~ExampleEventClient()
{

}

/**
 * @brief example Callback function, callback function must be boost::optional<rostate_machine::Event>(void) type
 * 
 * @return boost::optional<rostate_machine::Event> 
 */
boost::optional<rostate_machine::Event> ExampleEventClient::checkMotor()
{
    if(client_.getCurrentStateDuration().toSec()>1.0)
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "motor_disconnected";
        return ret;
    }
    return boost::none;
}

boost::optional<rostate_machine::Event> ExampleEventClient::checkRecovery()
{
    if(client_.getCurrentStateDuration().toSec()>2.0)
    {
        rostate_machine::Event ret;
        ret.header.stamp = ros::Time::now();
        ret.trigger_event_name = "recovery";
        return ret;
    }
    return boost::none;
}

/**
 * @brief example Callback function, callback function must be boost::optional<rostate_machine::Event>(void) type
 * 
 * @return boost::optional<rostate_machine::Event> 
 */
boost::optional<rostate_machine::Event> ExampleEventClient::stopMotor()
{
    return boost::none;
}