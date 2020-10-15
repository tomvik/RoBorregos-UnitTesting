#include "power_two/power_two.h"

#include <gtest/gtest.h>

#include "ros/ros.h"

class PowerTwoTest : public ::testing::Test {
 public:
    PowerTwoTest() {}
    ~PowerTwoTest() {}
};

//////////////// isOneArgument Tests ////////////////////////

TEST(PowerTwoTest, isOneArgument_MoreThanTwo) {
    char *argv[] = {"programName", "para1", "para2", "para3"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST(PowerTwoTest, isOneArgument_LessThanTwo) {
    char *argv[] = {"programName"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST(PowerTwoTest, isOneArgument_ExactlyTwo) {
    char *argv[] = {"programName", "9"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), true);
    EXPECT_EQ(string_argument, "9");
}

//////////////// isOneCharacter Tests ////////////////////////

TEST(PowerTwoTest, isOneCharacter_LessThanOne) {
    std::string string_argument = "";
    char input = 0;

    EXPECT_EQ(powerTwo::isOneCharacter(string_argument, input), false);
    EXPECT_EQ(input, 0);
}

TEST(PowerTwoTest, isOneCharacter_MoreThanOne) {
    std::string string_argument = "90123";
    char input = 0;

    EXPECT_EQ(powerTwo::isOneCharacter(string_argument, input), false);
    EXPECT_EQ(input, 0);
}

TEST(PowerTwoTest, isOneCharacter_ExactlyOne) {
    std::string string_argument = "9";
    char input = 0;

    EXPECT_EQ(powerTwo::isOneCharacter(string_argument, input), true);
    EXPECT_EQ(input, '9');
}

//////////////// isDigit Tests ////////////////////////

TEST(PowerTwoTest, isDigit_NotADigit) {
    const char input = 'a';
    int digit = -1;

    EXPECT_EQ(powerTwo::isDigit(input, digit), false);
    EXPECT_EQ(digit, -1);
}

TEST(PowerTwoTest, isDigit_IsADigit) {
    const char input = '5';
    int digit = -1;

    EXPECT_EQ(powerTwo::isDigit(input, digit), true);
    EXPECT_EQ(digit, 5);
}

//////////////// getPowerTwoMessage Tests ////////////////////////

TEST(PowerTwoTest, getPowerTwoMessage_OneResultingDigit) {
    const int digit = 1;

    EXPECT_EQ(powerTwo::getPowerTwoMessage(digit), "Your digit was: 1 and 1^2 = 01");
}

TEST(PowerTwoTest, getPowerTwoMessage_TwoResultingDigits) {
    const int digit = 5;

    EXPECT_EQ(powerTwo::getPowerTwoMessage(digit), "Your digit was: 5 and 5^2 = 25");
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
