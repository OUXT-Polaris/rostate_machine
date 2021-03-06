/**
 * @file test_rostate_machine
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Test code for rostate_machine package
 * @version 0.1
 * @date 2019-04-26
 * 
 * @copyright Copyright (c) OUXT Polaris 2019
 * 
 */

// Headers in this package
#include <rostate_machine/state_machine.h>

// Headers in Gtest
#include <gtest/gtest.h>

// Headers in ROS
#include <ros/ros.h>
#include <ros/package.h>

// Headers in STL
#include <fstream> 
#include <iostream>

// Declare Test
TEST(TestSuite, testCase1)
{
    std::string xml_filepath = ros::package::getPath("rostate_machine") + std::string("/test/data/test_state_machine.xml");
    std::ifstream ifs(xml_filepath);
    std::string description;
    std::string line;

    while (getline(ifs, line))
    {
        description += line + "\n";
    }
    StateMachine sm(description);
    EXPECT_EQ(sm.getCurrentState(),"remote_operated");
    EXPECT_EQ(sm.tryTransition("test"),false);
    EXPECT_EQ(sm.getCurrentState(),"remote_operated");
    EXPECT_EQ(sm.tryTransition("system_bringup"),true);
    EXPECT_EQ(sm.getCurrentState(),"autonomous");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}