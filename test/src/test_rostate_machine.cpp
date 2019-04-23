// Headers in this package
#include <rostate_machine/state_machine.h>

// Headers in Gtest
#include <gtest/gtest.h>

// Declare another test
TEST(TestSuite, testCase2)
{
    
    EXPECT_EQ(true,true);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}