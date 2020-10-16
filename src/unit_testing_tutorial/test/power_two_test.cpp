#include "power_two/power_two.h"

#include <gtest/gtest.h>

#include <iostream>

#include "ros/ros.h"

class PowerTwoTest : public ::testing::Test {
 public:
    PowerTwoTest() {}
    ~PowerTwoTest() {}

    void SetUp() {
        // Save cout's buffer here
        sbuf = std::cout.rdbuf();

        // Clear our inner buffer
        buffer.str(std::string());

        // Redirect cout to our stringstream buffer or any other ostream
        std::cout.rdbuf(buffer.rdbuf());
    }

    void TearDown() {
        buffer.str(std::string());
        // When done redirect cout to its old self
        std::cout.rdbuf(sbuf);
    }


    // This can be an ofstream as well or any other ostream
    std::stringstream buffer;
    std::streambuf *sbuf;
};

//////////////// isOneArgument Tests ////////////////////////

TEST_F(PowerTwoTest, isOneArgument_MoreThanTwo) {
    char *argv[] = {"programName", "para1", "para2", "para3"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST_F(PowerTwoTest, isOneArgument_LessThanTwo) {
    char *argv[] = {"programName"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_EQ(powerTwo::isOneArgument(argc, argv, string_argument), false);
    EXPECT_EQ(string_argument, "");
}

TEST_F(PowerTwoTest, isOneArgument_ExactlyTwo) {
    char *argv[] = {"programName", "9"};
    int argc = sizeof(argv) / sizeof(argv[0]);
    std::string string_argument = "";
    EXPECT_TRUE(powerTwo::isOneArgument(argc, argv, string_argument));
    EXPECT_EQ(string_argument, "9");
}

//////////////// isOneCharacter Tests ////////////////////////

TEST_F(PowerTwoTest, isOneCharacter_LessThanOne) {
    std::string string_argument = "";
    char input = 0;

    EXPECT_FALSE(powerTwo::isOneCharacter(string_argument, input));
    EXPECT_EQ(input, 0);
}

TEST_F(PowerTwoTest, isOneCharacter_MoreThanOne) {
    std::string string_argument = "90123";
    char input = 0;

    EXPECT_EQ(powerTwo::isOneCharacter(string_argument, input), false);
    EXPECT_EQ(input, 0);
}

TEST_F(PowerTwoTest, isOneCharacter_ExactlyOne) {
    std::string string_argument = "9";
    char input = 0;

    EXPECT_EQ(powerTwo::isOneCharacter(string_argument, input), true);
    EXPECT_EQ(input, '9');
}

//////////////// isDigit Tests ////////////////////////

TEST_F(PowerTwoTest, isDigit_NotADigit) {
    const char input = 'a';
    int digit = -1;

    EXPECT_EQ(powerTwo::isDigit(input, digit), false);
    EXPECT_EQ(digit, -1);
}

TEST_F(PowerTwoTest, isDigit_IsADigit) {
    const char input = '5';
    int digit = -1;

    EXPECT_EQ(powerTwo::isDigit(input, digit), true);
    EXPECT_EQ(digit, 5);
}

//////////////// getPowerTwoMessage Tests ////////////////////////

TEST_F(PowerTwoTest, getPowerTwoMessage_OneResultingDigit) {
    const int digit = 1;

    EXPECT_EQ(powerTwo::getPowerTwoMessage(digit), "Your digit was: 1 and 1^2 = 01");
}

TEST_F(PowerTwoTest, getPowerTwoMessage_TwoResultingDigits) {
    const int digit = 5;

    EXPECT_EQ(powerTwo::getPowerTwoMessage(digit), "Your digit was: 5 and 5^2 = 25");
}

//////////////// calculateAndPrintPowerTwo Tests ////////////////////////

TEST_F(PowerTwoTest, calculateAndPrintPowerTwo_WrongAmountOfParameters) {
    // Too many
    char *argv_first[] = {"programName", "para1", "para2", "para3"};
    int argc_first = sizeof(argv_first) / sizeof(argv_first[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_first, argv_first), -1);
    EXPECT_EQ(buffer.str(), "There should be only one argument :(\n");

    buffer.str(std::string());

    // Too few
    char *argv_second[] = {"programName"};
    int argc_second = sizeof(argv_second) / sizeof(argv_second[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_second, argv_second), -1);
    EXPECT_EQ(buffer.str(), "There should be only one argument :(\n");
}



TEST_F(PowerTwoTest, calculateAndPrintPowerTwo_WrongAmountOfCharacters) {
    char *argv_first[] = {"programName", "28"};
    int argc_first = sizeof(argv_first) / sizeof(argv_first[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_first, argv_first), -2);
    EXPECT_EQ(buffer.str(), "The input is not of length 1 :(\n");
}



TEST_F(PowerTwoTest, calculateAndPrintPowerTwo_NotADigit) {
    char *argv_first[] = {"programName", "a"};
    int argc_first = sizeof(argv_first) / sizeof(argv_first[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_first, argv_first), -3);
    EXPECT_EQ(buffer.str(), "The input is not a digit :(\n");
}



TEST_F(PowerTwoTest, calculateAndPrintPowerTwo_AllGood) {
    char *argv_first[] = {"programName", "1"};
    int argc_first = sizeof(argv_first) / sizeof(argv_first[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_first, argv_first), -0);
    EXPECT_EQ(buffer.str(), "Your digit was: 1 and 1^2 = 01\n");

    buffer.str(std::string());

    char *argv_second[] = {"programName", "9"};
    int argc_second = sizeof(argv_second) / sizeof(argv_second[0]);

    EXPECT_EQ(powerTwo::calculateAndPrintPowerTwo(argc_second, argv_second), -0);
    EXPECT_EQ(buffer.str(), "Your digit was: 9 and 9^2 = 81\n");
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
