#include <catch2/catch_all.hpp>
#include <depthai/device/Version.hpp>

using Version = dai::Version;

TEST_CASE("Version parsing") {
    // Valid versions
    {
        std::string str = "1.2.3";
        Version v("0.0.0");
        REQUIRE_NOTHROW(v = Version{str});
        REQUIRE(v.toString() == str);
    }
    {
        std::string str = "4.2.0+c137116638a96d2145ff091235de854890ed56ae";
        Version v("0.0.0");
        REQUIRE_NOTHROW(v = Version{str});
        REQUIRE(v.toString() == str);
    }

    // Invalid versions
    REQUIRE_THROWS(Version("1.7+abcdef"));
    REQUIRE_THROWS(Version("1..7+abcdef"));
    REQUIRE_THROWS(Version("1..7"));
    REQUIRE_THROWS(Version("1..7abcd"));
    REQUIRE_THROWS(Version(""));
}

TEST_CASE("Version comparisons") {
    // Less than
    REQUIRE(Version("0.0.15+abcdef") < Version("0.0.16"));
    REQUIRE(Version("0.0.14") < Version("0.0.15"));
    REQUIRE(Version("0.0.14") < Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.7") < Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.15+abcdef") < Version("0.1.0"));

    // Greater than
    REQUIRE(Version("0.0.16") > Version("0.0.15+abcdef"));
    REQUIRE(Version("0.0.15") > Version("0.0.14"));
    REQUIRE(Version("0.0.15+abcdef") > Version("0.0.14"));
    REQUIRE(Version("0.0.15+abcdef") > Version("0.0.7"));
    REQUIRE(Version("0.1.0") > Version("0.0.15+abcdef"));

    // Equality
    REQUIRE(Version("0.0.15") == Version("0.0.15"));
    REQUIRE(Version("22.7.15") == Version(22, 7, 15));
    REQUIRE(Version("0.7.15") == Version(0, 7, 15));
    REQUIRE(Version("0.7.15+abc123") == Version(0, 7, 15, "abc123"));
    REQUIRE(Version("0.7.15+abc123") == Version("0.7.15+somethingElse"));  // To comply with semver, build info is ignored in equality

    // Inequality
    REQUIRE_FALSE(Version("0.0.15") == Version("0.0.14"));
    REQUIRE_FALSE(Version("1.0.15") == Version("0.1.15"));
    REQUIRE_FALSE(Version("1.0.15") == Version("1.0.14"));
    REQUIRE_FALSE(Version("1.0.15") == Version("1.0.14+abc123"));
    REQUIRE_FALSE(Version("1.0.15") == Version("0.0.15+abc123"));
}

TEST_CASE("Version construction and string conversion") {
    // Construction from major, minor, patch, and build info
    Version v1(1, 2, 3, "build123");
    REQUIRE(v1.toString() == "1.2.3+build123");

    // Construction from major, minor, patch, pre-release, and build info
    Version v2(1, 2, 3, Version::PreReleaseType::BETA, 2, "build123");
    REQUIRE(v2.toString() == "1.2.3-beta.2+build123");

    // Construction from major, minor, patch only
    Version v3(1, 2, 3);
    REQUIRE(v3.toString() == "1.2.3");
    REQUIRE(v3.toStringSemver() == "1.2.3");

    // Construction from string
    Version v4("1.2.3-alpha.1+build123");
    REQUIRE(v4.toString() == "1.2.3-alpha.1+build123");
    REQUIRE(v4.toStringSemver() == "1.2.3-alpha.1");
}

TEST_CASE("Version build info") {
    Version v1(1, 2, 3, "build123");
    REQUIRE(v1.getBuildInfo() == "build123");

    Version v2("1.2.3+build123");
    REQUIRE(v2.getBuildInfo() == "build123");

    Version v3("1.2.3");
    REQUIRE(v3.getBuildInfo() == "");
}

TEST_CASE("Version copy and move semantics") {
    // Copy constructor
    Version v1("1.2.3");
    Version v2 = v1;  // Copy construction
    REQUIRE(v2.toString() == "1.2.3");
    REQUIRE(v1 == v2);

    // Copy assignment operator
    Version v3("4.5.6");
    v3 = v1;  // Copy assignment
    REQUIRE(v3.toString() == "1.2.3");
    REQUIRE(v1 == v3);

    // Move constructor
    Version v4 = std::move(v1);  // Move construction
    REQUIRE(v4.toString() == "1.2.3");

    // Move assignment operator
    Version v5("7.8.9");
    v5 = std::move(v2);  // Move assignment
    REQUIRE(v5.toString() == "1.2.3");

    // Destructor is implicitly tested by scope exit
}
