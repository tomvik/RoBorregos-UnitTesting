#include "power_two/power_two.h"

#include <iostream>
#include <string>

namespace powerTwo {

bool isOneArgument(const int argc, char** argv, std::string& string_argument) {
    if (argc == 2) {
        string_argument = argv[1];
        return true;
    } else {
        return false;
    }
}

bool isOneCharacter(const std::string& string_argument, char& input_digit) {
    if (string_argument.length() == 1) {
        input_digit = string_argument[0];
        return true;
    } else {
        return false;
    }
}

bool isDigit(const char input_digit, int& digit) {
    if (input_digit >= '0' && input_digit <= '9') {
        digit = input_digit - '0';
        return true;
    } else {
        return false;
    }
}

std::string getPowerTwoMessage(const int digit) {
    const int power_two = digit * digit;

    const char char_digit = digit + '0';
    const char power_two_first_digit = power_two / 10 + '0';
    const char power_two_second_digit = power_two % 10 + '0';

    std::string digit_string = "";
    digit_string += char_digit;

    std::string power_two_string = "";

    power_two_string += power_two_first_digit;
    power_two_string += power_two_second_digit;
    return "Your digit was: " + digit_string + " and " + digit_string + "^2 = " + power_two_string;
}

int calculateAndPrintPowerTwo(const int argc, char** argv) {
    std::string string_argument = "";
    if (isOneArgument(argc, argv, string_argument)) {
        char input_digit = 0;
        if (isOneCharacter(string_argument, input_digit)) {
            int digit = -1;
            if (isDigit(input_digit, digit)) {
                std::cout << getPowerTwoMessage(digit) << std::endl;
            } else {
                std::cout << "The input is not a digit :(" << std::endl;
                return -3;
            }
        } else {
            std::cout << "The input is not of length 1 :(" << std::endl;
            return -2;
        }
    } else {
        std::cout << "There should be only one argument :(" << std::endl;
        return -1;
    }
    return 0;
}

};  // namespace powerTwo
