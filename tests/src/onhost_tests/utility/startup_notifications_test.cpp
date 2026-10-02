#include <array>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "utility/StartupNotifications.hpp"

using dai::Platform;
using dai::utility::collectMessages;
using nlohmann::json;

namespace {

const json ALL_USERS = {{"message", "All users"},
                        {"depthaiVersions", json::array()},
                        {"platforms", json::array()},
                        {"protocols", json::array()},
                        {"osVersions", json::array()},
                        {"deviceSKUs", json::array()}};
constexpr std::array<const char*, 5> FILTER_KEYS = {"depthaiVersions", "platforms", "protocols", "osVersions", "deviceSKUs"};

}  // namespace

TEST_CASE("empty startup filters match all devices and preserve message order", "[startup_notifications]") {
    auto second = ALL_USERS;
    second["message"] = "Second announcement";
    const json payload = {{"messages", {ALL_USERS, second}}};
    for(const auto platform : {Platform::RVC2, Platform::RVC3, Platform::RVC4}) {
        REQUIRE(collectMessages(payload, "", platform, X_LINK_ANY_PROTOCOL, "", "") == std::vector<std::string>{"All users", "Second announcement"});
        REQUIRE(collectMessages(payload, "3.10.0", platform, X_LINK_USB_VSC, "", "OAK-D-PRO-FF")
                == std::vector<std::string>{"All users", "Second announcement"});
    }
}

TEST_CASE("startup version lists match exact SDK versions on every platform", "[startup_notifications]") {
    auto message = ALL_USERS;
    message["depthaiVersions"] = {"3.9.0", "3.10.0"};
    const json payload = {{"messages", {message}}};
    for(const auto platform : {Platform::RVC2, Platform::RVC3, Platform::RVC4}) {
        for(const auto* version : {"3.9.0", "3.10.0"}) {
            REQUIRE(collectMessages(payload, version, platform, X_LINK_TCP_IP, "", "any SKU") == std::vector<std::string>{"All users"});
        }
    }
    for(const auto* version : {"3.11.0", "3.10.0-rc.1", "3.10.0+build", ""}) {
        INFO(version);
        REQUIRE(collectMessages(payload, version, Platform::RVC2, X_LINK_USB_VSC, "", "OAK-D").empty());
    }
}

TEST_CASE("startup messages require every populated filter to match any listed value", "[startup_notifications]") {
    const json targeted = {{"message", "Targeted news"},
                           {"depthaiVersions", {"3.9.0", "3.10.0"}},
                           {"platforms", {"RVC2", "RVC4"}},
                           {"protocols", {"usb", "tcpip"}},
                           {"osVersions", {"1.43.0", "1.44.0"}},
                           {"deviceSKUs", {"OAK4-D", "OAK4-D-PRO-FF"}}};
    REQUIRE(collectMessages(json{{"messages", {targeted}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D-PRO-FF")
            == std::vector<std::string>{"Targeted news"});
    for(const auto* key : FILTER_KEYS) {
        INFO(key);
        auto message = targeted;
        message[key] = {"different"};
        REQUIRE(collectMessages(json{{"messages", {message}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D-PRO-FF").empty());
        message[key] = json::array();
        REQUIRE(collectMessages(json{{"messages", {message}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D-PRO-FF")
                == std::vector<std::string>{"Targeted news"});
    }
}

TEST_CASE("startup platform filters distinguish RVC2 RVC3 and RVC4", "[startup_notifications]") {
    auto message = ALL_USERS;
    message["platforms"] = {"RVC3"};
    const json payload = {{"messages", {message}}};
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC3, X_LINK_ANY_PROTOCOL, "", "") == std::vector<std::string>{"All users"});
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_ANY_PROTOCOL, "", "").empty());
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_ANY_PROTOCOL, "", "").empty());
}

TEST_CASE("startup protocol filters use the connected transport", "[startup_notifications]") {
    auto usb = ALL_USERS;
    usb["message"] = "USB news";
    usb["protocols"] = {"usb"};
    auto tcpip = ALL_USERS;
    tcpip["message"] = "TCP/IP news";
    tcpip["protocols"] = {"tcpip"};
    const json payload = {{"messages", {ALL_USERS, usb, tcpip}}};
    for(const auto protocol : {X_LINK_USB_VSC, X_LINK_USB_CDC, X_LINK_USB_EP}) {
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, protocol, "", "OAK-D") == std::vector<std::string>{"All users", "USB news"});
    }
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D") == std::vector<std::string>{"All users", "TCP/IP news"});
    for(const auto protocol : {X_LINK_ANY_PROTOCOL, X_LINK_LOCAL_SHDMEM, X_LINK_TCP_IP_OR_LOCAL_SHDMEM, X_LINK_PCIE, X_LINK_IPC}) {
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, protocol, "1.44.0", "OAK4-D") == std::vector<std::string>{"All users"});
    }
}

TEST_CASE("startup OS version filters require an exact available OS version", "[startup_notifications]") {
    auto message = ALL_USERS;
    message["osVersions"] = {"1.44.0"};
    const json payload = {{"messages", {message}}};
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_ANY_PROTOCOL, "1.44.0", "OAK4-D") == std::vector<std::string>{"All users"});
    for(const auto* version : {"", "1.43.0", "1.44.0-rc.1", "1.44.0+build"}) {
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_ANY_PROTOCOL, version, "OAK4-D").empty());
    }
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_ANY_PROTOCOL, "", "OAK-D").empty());
}

TEST_CASE("startup SKU filters support trailing wildcard prefixes", "[startup_notifications]") {
    auto message = ALL_USERS;
    message["deviceSKUs"] = {"OAK*"};
    const json payload = {{"messages", {message}}};
    for(const auto* sku : {"OAK-D-PRO-FF", "OAK-1", "OAK4-D", "OAK"}) {
        INFO(sku);
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_USB_VSC, "", sku) == std::vector<std::string>{"All users"});
    }
    for(const auto* sku : {"OTHER-DEVICE", "NOT-OAK-1", "oak-1", "OA", ""}) {
        INFO(sku);
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_USB_VSC, "", sku).empty());
    }
}

TEST_CASE("startup SKU filters combine exact names and prefixes with other filters", "[startup_notifications]") {
    auto message = ALL_USERS;
    message["depthaiVersions"] = {"3.10.0"};
    message["deviceSKUs"] = {"OAK-1", "OAK4*"};
    const json payload = {{"messages", {message}}};
    for(const auto* sku : {"OAK-1", "OAK4-D-PRO-FF"}) {
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", sku) == std::vector<std::string>{"All users"});
        REQUIRE(collectMessages(payload, "3.11.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", sku).empty());
    }
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_USB_VSC, "", "OAK-10").empty());
    REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC2, X_LINK_USB_VSC, "", "OAK-D-PRO-FF").empty());
}

TEST_CASE("all startup filters support trailing wildcard prefixes", "[startup_notifications]") {
    const json targeted = {{"message", "Targeted news"},
                           {"depthaiVersions", {"3.10.*"}},
                           {"platforms", {"RVC*"}},
                           {"protocols", {"tcp*"}},
                           {"osVersions", {"1.44.*"}},
                           {"deviceSKUs", {"OAK*"}}};
    REQUIRE(collectMessages(json{{"messages", {targeted}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK-1")
            == std::vector<std::string>{"Targeted news"});
    REQUIRE(collectMessages(json{{"messages", {targeted}}}, "3.10.2-rc.1", Platform::RVC3, X_LINK_TCP_IP, "1.44.3", "OAK4-D")
            == std::vector<std::string>{"Targeted news"});
    REQUIRE(collectMessages(json{{"messages", {targeted}}}, "3.100.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK-1").empty());

    for(const auto* key : FILTER_KEYS) {
        INFO(key);
        auto message = targeted;
        message[key] = {"different*"};
        REQUIRE(collectMessages(json{{"messages", {message}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK-1").empty());
        message[key] = {"different*", "*"};
        REQUIRE(collectMessages(json{{"messages", {message}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK-1")
                == std::vector<std::string>{"Targeted news"});
        message[key] = {"*", false};
        REQUIRE(collectMessages(json{{"messages", {message, ALL_USERS}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK-1")
                == std::vector<std::string>{"All users"});
    }
}

TEST_CASE("bare startup wildcards match all values including unavailable device information", "[startup_notifications]") {
    auto message = ALL_USERS;
    for(const auto* key : FILTER_KEYS) {
        message[key] = {"*"};
    }
    REQUIRE(collectMessages(json{{"messages", {message}}}, "", Platform::RVC2, X_LINK_ANY_PROTOCOL, "", "") == std::vector<std::string>{"All users"});
}

TEST_CASE("malformed startup entries do not suppress valid messages", "[startup_notifications]") {
    for(const auto& payload : {json(nullptr), json::array(), json::object(), json{{"messages", nullptr}}, json{{"messages", "Not a list"}}}) {
        REQUIRE(collectMessages(payload, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D").empty());
    }
    for(const auto& malformed : {json(nullptr), json(42), json("Not an object"), json::array(), json::object(), json{{"message", "Missing filters"}}}) {
        REQUIRE(collectMessages(json{{"messages", {malformed, ALL_USERS}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D")
                == std::vector<std::string>{"All users"});
    }
    for(const auto& value : {json(nullptr), json(42), json(""), json::array({"Nested message"})}) {
        auto malformed = ALL_USERS;
        malformed["message"] = value;
        REQUIRE(collectMessages(json{{"messages", {malformed, ALL_USERS}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D")
                == std::vector<std::string>{"All users"});
    }
    for(const auto* key : FILTER_KEYS) {
        INFO(key);
        for(const auto& value : {json(nullptr), json(42), json(""), json::object(), json::array({""}), json::array({"3.10.0", false})}) {
            auto malformed = ALL_USERS;
            malformed[key] = value;
            REQUIRE(collectMessages(json{{"messages", {malformed, ALL_USERS}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D")
                    == std::vector<std::string>{"All users"});
        }
        auto malformed = ALL_USERS;
        malformed.erase(key);
        REQUIRE(collectMessages(json{{"messages", {malformed, ALL_USERS}}}, "3.10.0", Platform::RVC4, X_LINK_TCP_IP, "1.44.0", "OAK4-D")
                == std::vector<std::string>{"All users"});
    }
}
