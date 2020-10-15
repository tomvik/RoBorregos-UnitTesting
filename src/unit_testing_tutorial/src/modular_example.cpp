#include <iostream>

////////////////////////////////////////////////////////////////////////////////
/*
int main(int argc, char** argv) {
    if (argc == 2) {
        std::string string_argument(argv[1]);
        if (string_argument.length() == 1) {
            const char input_digit = string_argument[0];
            if (input_digit >= '0' && input_digit <= '9') {
                const int digit = input_digit - '0';
                std::cout << "Your digit was: " << digit << " and " << digit
                          << "^2 = " << digit * digit << std::endl;
            } else {
                return -3;
            }
        } else {
            return -2;
        }
    } else {
        return -1;
    }
    return 0;
}
*/
//////////////////////////////////////////////////////////////////////////////

bool isOneArgument(const int argc, char** argv, std::string& string_argument) {
    if (argc == 2) {
        std::string string_argument(argv[1]);
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
    char* digit_c_string;
    char* power_two_c_string;
    itoa(digit, digit_c_string, 10);
    itoa(digit * digit, power_two_c_string, 10);
    std::string digit_string(digit_c_string);
    std::string power_two_string(power_two_c_string);
    return "Your digit was: " + digit_string + " and " + digit_string + "^2 = " + power_two_string;
}

int main(int argc, char** argv) {
    std::string string_argument = "";
    if (isOneArgument(argc, argv, string_argument)) {
        char input_digit = 0;
        if (isOneCharacter(string_argument, input_digit)) {
            int digit = -1;
            if (isDigit(input_digit, digit)) {
                std::cout << getPowerTwoMessage(digit) << std::endl;
            } else {
                return -3;
            }
        } else {
            return -2;
        }
    } else {
        return -1;
    }
    return 0;
}