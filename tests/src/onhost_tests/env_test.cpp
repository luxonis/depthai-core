#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include <cassert>
#include <catch2/catch_all.hpp>
#include <iostream>
#include <string>
#include <vector>

#include "../../../src/utility/Environment.hpp"
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
