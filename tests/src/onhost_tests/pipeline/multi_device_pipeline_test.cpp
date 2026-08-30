#include <catch2/catch_all.hpp>

#include "depthai/depthai.hpp"

// Host-only checks for the multi-device pipeline API (no device required)

TEST_CASE("Linking across two different pipelines throws") {
    dai::Pipeline p1(false);
    dai::Pipeline p2(false);

    auto syncA = p1.create<dai::node::Sync>();
    auto syncB = p2.create<dai::node::Sync>();

    REQUIRE_THROWS_AS(syncA->out.link(syncB->inputs["in"]), std::runtime_error);
}

TEST_CASE("Linking within the same pipeline still works") {
    dai::Pipeline p(false);

    auto syncA = p.create<dai::node::Sync>();
    auto syncB = p.create<dai::node::Sync>();

    REQUIRE_NOTHROW(syncA->out.link(syncB->inputs["in"]));
}

TEST_CASE("Host-only pipeline has no devices") {
    dai::Pipeline p(false);
    REQUIRE(p.getDevices().empty());
    REQUIRE(p.getDefaultDevice() == nullptr);
}

TEST_CASE("addDevice rejects a null device") {
    dai::Pipeline p(false);
    REQUIRE_THROWS_AS(p.addDevice(std::shared_ptr<dai::Device>()), std::invalid_argument);
}
