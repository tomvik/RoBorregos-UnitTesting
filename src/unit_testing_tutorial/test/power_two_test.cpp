// Bring in my package's API, which is what I'm testing
// Bring in gtest
#include <gtest/gtest.h>

#include "power_two/power_two.h"
#include "ros/ros.h"

class PowerTwoTest : public ::testing::Test {
 public:
    PowerTwoTest() {}
    ~PowerTwoTest() {}
};

// Declare a test
TEST(PowerTwoTest, isOneArgument_MoreThanTwo) {
    char *argv[] = {"programName", "para1", "para2", "para3"};
    int argc = sizeof(argv)/sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST(PowerTwoTest, isOneArgument_LessThanTwo) {
    char *argv[] = {"programName"};
    int argc = sizeof(argv)/sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST(PowerTwoTest, isOneArgument_ExactlyTwo) {
    char *argv[] = {"programName", "9"};
    int argc = sizeof(argv)/sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), true);
    EXPECT_EQ(string_argument, "9");
}

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
