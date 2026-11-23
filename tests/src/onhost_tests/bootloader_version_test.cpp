#include <catch2/catch_all.hpp>
#include <depthai/device/Version.hpp>

#include "utility/DeviceOSVersion.hpp"

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

TEST_CASE("Device OS version parsing preserves release semantics", "[DeviceOSVersion]") {
    for(const auto* version : {"1.46.0", "1.47.0-rc.1", "1.47.0+build"}) {
        const auto parsed = dai::utility::parseDeviceOSVersion(version);
        REQUIRE(parsed.has_value());
        REQUIRE(parsed->toString() == version);
    }
    const auto prefixed = dai::utility::parseDeviceOSVersion("r851-1.47.0");
    REQUIRE(prefixed.has_value());
    REQUIRE(*prefixed == Version(1, 47, 0));
    for(const auto* version : {"", "unknown", "custom", "1.46", "r-1.47.0", "rx-1.47.0", "1.47.0garbage"}) {
        CAPTURE(version);
        REQUIRE_FALSE(dai::utility::parseDeviceOSVersion(version));
    }
}

TEST_CASE("Device OS cache replaces metadata for each connection", "[DeviceOSVersion]") {
    dai::utility::DeviceOSVersionCache cache;
    int calls = 0;
    REQUIRE_FALSE(cache.get()->parsed);
    cache.refresh([&] { ++calls; return "1.45.0"; });
    const auto firstSession = cache.get();
    cache.refresh([&] { ++calls; return "r2-1.47.0"; });
    REQUIRE(cache.get()->parsed == Version(1, 47, 0));
    REQUIRE(cache.get()->raw == "r2-1.47.0");
    REQUIRE(firstSession->parsed == Version(1, 45, 0));
    for(int i = 0; i < 10; ++i) REQUIRE(cache.get()->parsed == Version(1, 47, 0));
    REQUIRE(calls == 2);

    SECTION("Malformed metadata clears the previous parsed version") {
        cache.refresh([&] { ++calls; return "invalid"; });
        REQUIRE(cache.get()->raw == "invalid");
        REQUIRE_FALSE(cache.get()->parsed);
    }
    SECTION("Unavailable RPC clears the previous session") {
        REQUIRE_THROWS(cache.refresh([&]() -> std::string { ++calls; throw std::runtime_error("RPC unavailable"); }));
        REQUIRE(cache.get()->raw.empty());
        REQUIRE_FALSE(cache.get()->parsed);
    }
    SECTION("Unsupported platform clears metadata") {
        cache.refresh([] { return std::string{}; });
        REQUIRE(cache.get()->raw.empty());
        REQUIRE_FALSE(cache.get()->parsed);
        REQUIRE(calls == 2);
    }
}
