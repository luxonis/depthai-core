#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <catch2/catch_all.hpp>
#include <string>
#include <utility/Environment.hpp>
#include <vector>

using namespace dai::utility;

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
        dai::utility::setEnv("stringVariable", "HELLO");
        REQUIRE(dai::utility::getEnvAs<std::string>("stringVariable", "DEFAULT") == "HELLO");

        // Previously existing variable gets cached
        dai::utility::unsetEnv("stringVariable");
        REQUIRE(dai::utility::getEnvAs<std::string>("stringVariable", "DEFAULT") == "HELLO");

        // Disable caching
        REQUIRE(dai::utility::getEnvAs<std::string>("stringVariable", "DEFAULT", false) == "DEFAULT");

        // Test non-existing string variable
        dai::utility::unsetEnv("stringVariable2");
        REQUIRE(dai::utility::getEnvAs<std::string>("stringVariable2", "DEFAULT") == "DEFAULT");
    }

    SECTION("Int") {
        // Test existing int variable
        dai::utility::setEnv("intVariable", "123");
        REQUIRE(dai::utility::getEnvAs<int>("intVariable", 0) == 123);

        // Test non-existing int variable
        dai::utility::unsetEnv("intVariable2");
        REQUIRE(dai::utility::getEnvAs<int>("intVariable2", 0) == 0);

        // Test invalid value
        dai::utility::setEnv("intVariable3", "INVALID");
        REQUIRE(dai::utility::getEnvAs<int>("intVariable3", 0) == 0);
    }

    SECTION("Bool") {
        // Test existing bool variable
        dai::utility::setEnv("boolVariable", "true");
        REQUIRE(dai::utility::getEnvAs<bool>("boolVariable", false) == true);

        dai::utility::setEnv("boolVariable2", "1");
        REQUIRE(dai::utility::getEnvAs<bool>("boolVariable2", false) == true);

        dai::utility::setEnv("boolVariable3", "0");
        REQUIRE(dai::utility::getEnvAs<bool>("boolVariable3", true) == false);

        dai::utility::setEnv("boolVariable4", "INVALID");
        REQUIRE(dai::utility::getEnvAs<bool>("boolVariable4", true) == true);

        dai::utility::setEnv("boolVariable5", "INVALID2");
        REQUIRE(dai::utility::getEnvAs<bool>("boolVariable5", false) == false);
    }

    SECTION("Float") {
        // Test existing float variable
        dai::utility::setEnv("floatVariable", "1.23");
        REQUIRE(dai::utility::getEnvAs<float>("floatVariable", 0.0f) == 1.23f);

        // Test non-existing float variable
        dai::utility::unsetEnv("floatVariable2");
        REQUIRE(dai::utility::getEnvAs<float>("floatVariable2", 0.0f) == 0.0f);

        // Test invalid value
        dai::utility::setEnv("floatVariable3", "INVALID");
        REQUIRE(dai::utility::getEnvAs<float>("floatVariable3", 0.0f) == 0.0f);
    }
}

TEST_CASE("Test getEnvAsChecked", "[getEnvAsChecked]") {
    SECTION("String") {
        // Test existing string variable
        dai::utility::setEnv("stringVariable", "HELLO");
        REQUIRE(dai::utility::getEnvAsChecked<std::string>("stringVariable", "WORLD", {"HELLO", "WORLD"}) == "HELLO");

        // Test non-existing string variable
        dai::utility::unsetEnv("stringVariable2");
        REQUIRE(dai::utility::getEnvAsChecked<std::string>("stringVariable2", "DEFAULT", {"HELLO", "DEFAULT"}) == "DEFAULT");

        // Test that if default value is not valid, we throw an error
        REQUIRE_THROWS_AS(dai::utility::getEnvAsChecked<std::string>("stringVariable3", "WORLD", {"HELLO"}), std::runtime_error);

        // Test invalid value - should use default value
        dai::utility::setEnv("stringVariable4", "INVALID");
        REQUIRE(dai::utility::getEnvAsChecked<std::string>("stringVariable4", "WORLD", {"HELLO", "WORLD"}) == "WORLD");
    }

    SECTION("Double") {
        // Test existing double variable
        dai::utility::setEnv("doubleVariable", "1.23");
        REQUIRE(dai::utility::getEnvAsChecked<double>("doubleVariable", 0.0, {0.0, 1.23}) == 1.23);

        // Test non-existing double variable
        dai::utility::unsetEnv("doubleVariable2");
        REQUIRE(dai::utility::getEnvAsChecked<double>("doubleVariable2", 0.0, {0.0, 1.23}) == 0.0);

        // Test that if default value is not valid, we throw an error
        REQUIRE_THROWS_AS(dai::utility::getEnvAsChecked<double>("doubleVariable3", 1.23, {0.0}), std::runtime_error);

        // Test invalid value - should use default value
        dai::utility::setEnv("doubleVariable4", "3.14");
        REQUIRE(dai::utility::getEnvAsChecked<double>("doubleVariable4", 0.0, {0.0, 1.23}) == 0.0);
    }
}
