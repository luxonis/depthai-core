#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch2/catch_all.hpp>
#include <cstdlib>
#include <string>
#include <vector>

#include "../../../src/utility/Environment.hpp"

#ifdef _WIN32
    #include <windows.h>

    #include <cstdlib>
#endif

using namespace dai::utility;

// Test helper functions
namespace {
void setEnvironmentVariable(const std::string& key, const std::string& value) {
#ifdef _WIN32
    // Use Windows API to set environment variables
    SetEnvironmentVariableA(key.c_str(), value.c_str());
    _putenv_s(key.c_str(), value.c_str());
#else
    constexpr int overwrite = 1;  // Overwrite if variable already exists
    setenv(key.c_str(), value.c_str(), overwrite);
#endif
}

void unsetEnvironmentVariable(const std::string& key) {
#ifdef _WIN32
    // Use Windows API to remove environment variables
    SetEnvironmentVariableA(key.c_str(), nullptr);
    _putenv_s(key.c_str(), "");
#else
    unsetenv(key.c_str());
#endif
}
}  // namespace

TEST_CASE("splitList - Basic functionality", "[splitList]") {
    SECTION("Basic test") {
        std::string input = "apple,banana,cherry,date";
        std::vector<std::string> expected = {"apple", "banana", "cherry", "date"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Comma and space delimiter") {
        std::string input = "apple, banana, cherry, date";
        std::vector<std::string> expected = {"apple", "banana", "cherry", "date"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Single element") {
        std::string input = "apple";
        std::vector<std::string> expected = {"apple"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Empty string") {
        std::string input = "";
        std::vector<std::string> expected = {};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Delimiter at the start") {
        std::string input = ",apple,banana";
        std::vector<std::string> expected = {"", "apple", "banana"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Delimiter at the end") {
        std::string input = "apple,banana,";
        std::vector<std::string> expected = {"apple", "banana", ""};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Consecutive delimiters") {
        std::string input = "apple,,banana";
        std::vector<std::string> expected = {"apple", "", "banana"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Multiple character delimiter") {
        std::string input = "apple--banana--cherry";
        std::vector<std::string> expected = {"apple", "banana", "cherry"};
        REQUIRE(splitList(input, "--") == expected);
    }

    SECTION("Delimiter not found") {
        std::string input = "apple-banana";
        std::vector<std::string> expected = {"apple-banana"};
        REQUIRE(splitList(input, ",") == expected);
    }

    SECTION("Delimiter is a space") {
        std::string input = "apple banana cherry";
        std::vector<std::string> expected = {"apple", "banana", "cherry"};
        REQUIRE(splitList(input, " ") == expected);
    }

    SECTION("Delimiter is a space, multiple spaces") {
        std::string input = "apple  banana   cherry";
        std::vector<std::string> expected = {"apple", "", "banana", "", "", "cherry"};
        REQUIRE(splitList(input, " ") == expected);
    }

    SECTION("Long delimiter") {
        std::string input = "appleXYZbananaXYZcherry";
        std::vector<std::string> expected = {"apple", "banana", "cherry"};
        REQUIRE(splitList(input, "XYZ") == expected);
    }

    SECTION("Complex case with special characters") {
        std::string input = "apple,banana,,,cherry,,date";
        std::vector<std::string> expected = {"apple", "banana", "", "", "cherry", "", "date"};
        REQUIRE(splitList(input, ",") == expected);
    }
}

TEST_CASE("Test getEnvAs", "[getEnvAs]") {
    SECTION("String") {
        // Test existing string variable
        setEnvironmentVariable("stringVariable", "HELLO");
        REQUIRE(getEnvAs<std::string>("stringVariable", "DEFAULT") == "HELLO");

        // Previously existing variable gets cached
        unsetEnvironmentVariable("stringVariable");
        REQUIRE(getEnvAs<std::string>("stringVariable", "DEFAULT") == "HELLO");

        // Disable caching
        REQUIRE(getEnvAs<std::string>("stringVariable", "DEFAULT", false) == "DEFAULT");

        // Test non-existing string variable
        unsetEnvironmentVariable("stringVariable2");
        REQUIRE(getEnvAs<std::string>("stringVariable2", "DEFAULT") == "DEFAULT");
    }

    SECTION("Int") {
        // Test existing int variable
        setEnvironmentVariable("intVariable", "123");
        REQUIRE(getEnvAs<int>("intVariable", 0) == 123);

        // Test non-existing int variable
        unsetEnvironmentVariable("intVariable2");
        REQUIRE(getEnvAs<int>("intVariable2", 0) == 0);

        // Test invalid value
        setEnvironmentVariable("intVariable3", "INVALID");
        REQUIRE(getEnvAs<int>("intVariable3", 0) == 0);
    }

    SECTION("Bool") {
        // Test existing bool variable
        setEnvironmentVariable("boolVariable", "true");
        REQUIRE(getEnvAs<bool>("boolVariable", false) == true);

        setEnvironmentVariable("boolVariable2", "1");
        REQUIRE(getEnvAs<bool>("boolVariable2", false) == true);

        setEnvironmentVariable("boolVariable3", "0");
        REQUIRE(getEnvAs<bool>("boolVariable3", true) == false);

        setEnvironmentVariable("boolVariable4", "INVALID");
        REQUIRE(getEnvAs<bool>("boolVariable4", true) == true);

        setEnvironmentVariable("boolVariable5", "INVALID2");
        REQUIRE(getEnvAs<bool>("boolVariable5", false) == false);
    }

    SECTION("Float") {
        // Test existing float variable
        setEnvironmentVariable("floatVariable", "1.23");
        REQUIRE(getEnvAs<float>("floatVariable", 0.0f) == 1.23f);

        // Test non-existing float variable
        unsetEnvironmentVariable("floatVariable2");
        REQUIRE(getEnvAs<float>("floatVariable2", 0.0f) == 0.0f);

        // Test invalid value
        setEnvironmentVariable("floatVariable3", "INVALID");
        REQUIRE(getEnvAs<float>("floatVariable3", 0.0f) == 0.0f);
    }
}

TEST_CASE("Test getEnvAsChecked", "[getEnvAsChecked]") {
    SECTION("String") {
        // Test existing string variable
        setEnvironmentVariable("stringVariable", "HELLO");
        REQUIRE(getEnvAsChecked<std::string>("stringVariable", "WORLD", {"HELLO", "WORLD"}) == "HELLO");

        // Test non-existing string variable
        unsetEnvironmentVariable("stringVariable2");
        REQUIRE(getEnvAsChecked<std::string>("stringVariable2", "DEFAULT", {"HELLO", "DEFAULT"}) == "DEFAULT");

        // Test that if default value is not valid, we throw an error
        REQUIRE_THROWS_AS(getEnvAsChecked<std::string>("stringVariable3", "WORLD", {"HELLO"}), std::runtime_error);

        // Test invalid value - should use default value
        setEnvironmentVariable("stringVariable4", "INVALID");
        REQUIRE(getEnvAsChecked<std::string>("stringVariable4", "WORLD", {"HELLO", "WORLD"}) == "WORLD");
    }

    SECTION("Double") {
        // Test existing double variable
        setEnvironmentVariable("doubleVariable", "1.23");
        REQUIRE(getEnvAsChecked<double>("doubleVariable", 0.0, {0.0, 1.23}) == 1.23);

        // Test non-existing double variable
        unsetEnvironmentVariable("doubleVariable2");
        REQUIRE(getEnvAsChecked<double>("doubleVariable2", 0.0, {0.0, 1.23}) == 0.0);

        // Test that if default value is not valid, we throw an error
        REQUIRE_THROWS_AS(getEnvAsChecked<double>("doubleVariable3", 1.23, {0.0}), std::runtime_error);

        // Test invalid value - should use default value
        setEnvironmentVariable("doubleVariable4", "3.14");
        REQUIRE(getEnvAsChecked<double>("doubleVariable4", 0.0, {0.0, 1.23}) == 0.0);
    }
}
