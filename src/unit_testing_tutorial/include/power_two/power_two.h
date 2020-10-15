#ifndef SRC_UNIT_TESTING_TUTORIAL_INCLUDE_POWER_TWO_POWER_TWO_H_
#define SRC_UNIT_TESTING_TUTORIAL_INCLUDE_POWER_TWO_POWER_TWO_H_

#include <string>

namespace powerTwo {

bool isOneArgument(const int argc, char** argv, std::string& string_argument);

bool isOneCharacter(const std::string& string_argument, char& input_digit);

bool isDigit(const char input_digit, int& digit);

std::string getPowerTwoMessage(const int digit);

int calculateAndPrintPowerTwo(const int argc, char** argv);

};  // namespace powerTwo

#endif  // SRC_UNIT_TESTING_TUTORIAL_INCLUDE_POWER_TWO_POWER_TWO_H_
