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
ExampleEventClient::ExampleEventClient(ros::NodeHandle nh,ros::NodeHandle pnh) : client_(nh,pnh)
{
    nh_ = nh;
    pnh_ = pnh;
    /**
     * @brief register callback for the tag in StateMachine .xml file
     * 
     */
    client_.registerCallback(std::bind(&ExampleEventClient::stopMotor, this),"stop_motor");
    client_.registerCallback(std::bind(&ExampleEventClient::checkMotor, this),"check_motor");
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
    rostate_machine::Event ret;
    ret.header.stamp = ros::Time::now();
    ret.trigger_event_name = "motor_disconnected";
    return ret;
}

/**
 * @brief example Callback function, callback function must be boost::optional<rostate_machine::Event>(void) type
 * 
 * @return boost::optional<rostate_machine::Event> 
 */
boost::optional<rostate_machine::Event> ExampleEventClient::stopMotor()
{
    rostate_machine::Event ret;
    ret.header.stamp = ros::Time::now();
    ret.trigger_event_name = "stop_motor";
    return ret;
}