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

TEST_CASE("DeviceInfo from an IP address resolves to TCP_IP") {
    REQUIRE(dai::DeviceInfo("10.12.234.143").protocol == X_LINK_TCP_IP);
    REQUIRE(dai::DeviceInfo("192.168.1.1").protocol == X_LINK_TCP_IP);
}

TEST_CASE("DeviceInfo from a non-IP dotted name keeps ANY protocol") {
    REQUIRE(dai::DeviceInfo("oak-1.local").protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(dai::DeviceInfo("1.2.3.4.5").protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(dai::DeviceInfo("999.1.1.1").protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(dai::DeviceInfo("2.1.usb").protocol == X_LINK_ANY_PROTOCOL);
}

TEST_CASE("DeviceInfo from a device id keeps ANY protocol") {
    auto info = dai::DeviceInfo("14442C10D13EABCE00");
    REQUIRE(info.protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(info.getDeviceId() == "14442C10D13EABCE00");
}

TEST_CASE("DeviceInfo::local targets the booted local shared-memory device") {
    auto info = dai::DeviceInfo::local();
    REQUIRE(info.state == X_LINK_BOOTED);
    REQUIRE(info.protocol == X_LINK_LOCAL_SHDMEM);
}

TEST_CASE("MessageGroup interval uses the timestamp source the group was synced with") {
    using namespace std::chrono;
    using Source = dai::SyncProperties::TimestampSource;

    auto makeBuffer = [](int64_t hostMs, int64_t deviceMs) {
        auto buffer = std::make_shared<dai::Buffer>();
        buffer->setTimestamp(time_point<steady_clock, steady_clock::duration>(milliseconds(hostMs)));
        buffer->setTimestampDevice(time_point<steady_clock, steady_clock::duration>(milliseconds(deviceMs)));
        return buffer;
    };

    dai::MessageGroup group;
    group.add("a", makeBuffer(100, 1000));
    group.add("b", makeBuffer(103, 1020));

    SECTION("DEVICE source (default, matches device-produced groups)") {
        REQUIRE(group.getTimestampSource() == Source::DEVICE);
        REQUIRE(group.getIntervalNs() == duration_cast<nanoseconds>(milliseconds(20)).count());
        REQUIRE(group.isSynced(duration_cast<nanoseconds>(milliseconds(20)).count()));
        REQUIRE(!group.isSynced(duration_cast<nanoseconds>(milliseconds(19)).count()));
    }

    SECTION("HOST source measures host timestamps") {
        group.setTimestampSource(Source::HOST);
        REQUIRE(group.getIntervalNs() == duration_cast<nanoseconds>(milliseconds(3)).count());
        REQUIRE(group.isSynced(duration_cast<nanoseconds>(milliseconds(3)).count()));
    }

    SECTION("SYSTEM source skips entries without a system timestamp") {
        group.setTimestampSource(Source::SYSTEM);
        REQUIRE(group.getIntervalNs() == 0);
    }
}
