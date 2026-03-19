#include <catch2/catch_all.hpp>

#include <chrono>
#include <optional>

#include "depthai/depthai.hpp"

using namespace std::chrono_literals;

namespace {

template <typename Clock>
int64_t toNs(std::chrono::time_point<Clock> tp) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch()).count();
}

}  // namespace

TEST_CASE("Buffer PTP timestamp set/unset", "[PTP][Buffer]") {
    dai::Buffer msg;

    REQUIRE_FALSE(msg.getTimestampPtp().has_value());

    auto tsPtp = std::chrono::time_point<dai::ptp_clock>(1234567890ns);
    msg.setTimestampPtp(tsPtp);
    REQUIRE(msg.getTimestampPtp().has_value());
    REQUIRE(toNs(*msg.getTimestampPtp()) == toNs(tsPtp));

    msg.setTimestampPtp(std::nullopt);
    REQUIRE_FALSE(msg.getTimestampPtp().has_value());
}

TEST_CASE("ImgFrame PTP exposure offsets", "[PTP][ImgFrame]") {
    dai::ImgFrame frame;
    frame.cam.exposureTimeUs = 2000;

    auto tsPtp = std::chrono::time_point<dai::ptp_clock>(2500000000ns);
    frame.setTimestampPtp(tsPtp);

    REQUIRE(frame.getTimestampPtp().has_value());
    REQUIRE(frame.getTimestampPtp(dai::CameraExposureOffset::END).has_value());
    REQUIRE(frame.getTimestampPtp(dai::CameraExposureOffset::START).has_value());
    REQUIRE(frame.getTimestampPtp(dai::CameraExposureOffset::MIDDLE).has_value());

    REQUIRE(toNs(*frame.getTimestampPtp(dai::CameraExposureOffset::END)) == toNs(tsPtp));
    REQUIRE(toNs(*frame.getTimestampPtp(dai::CameraExposureOffset::START) + frame.getExposureTime()) == toNs(tsPtp));
    REQUIRE(toNs(*frame.getTimestampPtp(dai::CameraExposureOffset::MIDDLE) + frame.getExposureTime() / 2) == toNs(tsPtp));

    frame.setTimestampPtp(std::nullopt);
    REQUIRE_FALSE(frame.getTimestampPtp().has_value());
    REQUIRE_FALSE(frame.getTimestampPtp(dai::CameraExposureOffset::START).has_value());
}

TEST_CASE("PTP timestamp serialization roundtrip", "[PTP][Serialization]") {
    SECTION("ImgFrame utility serialize/deserialize") {
        dai::ImgFrame src;
        src.setSequenceNum(77);
        src.setTimestampPtp(std::chrono::time_point<dai::ptp_clock>(987654321ns));
        src.setTimestampSystem(std::chrono::time_point<std::chrono::system_clock>(123456789ns));
        src.setTimestampDevice(std::chrono::steady_clock::time_point(333ns));
        src.setTimestamp(std::chrono::steady_clock::time_point(222ns));

        auto ser = dai::utility::serialize(src);
        dai::ImgFrame dst;
        dai::utility::deserialize(ser, dst);

        REQUIRE(dst.getSequenceNum() == 77);
        REQUIRE(dst.getTimestampPtp().has_value());
        REQUIRE(toNs(*dst.getTimestampPtp()) == 987654321);
        REQUIRE(dst.getTimestampSystem().has_value());
        REQUIRE(toNs(*dst.getTimestampSystem()) == 123456789);
    }

    #ifndef DEPTHAI_MESSAGES_RVC2
    SECTION("IMUData report and aggregate PTP roundtrip") {
        dai::IMUData src;
        src.setSequenceNum(42);
        src.setTimestampPtp(std::chrono::time_point<dai::ptp_clock>(7654321ns));

        dai::IMUPacket packet;
        packet.acceleroMeter.sequence = 11;
        packet.acceleroMeter.tsPtp = dai::Timestamp{123, 456};
        src.packets.push_back(packet);

        auto ser = dai::utility::serialize(src);
        dai::IMUData dst;
        dai::utility::deserialize(ser, dst);

        REQUIRE(dst.getSequenceNum() == 42);
        REQUIRE(dst.getTimestampPtp().has_value());
        REQUIRE(toNs(*dst.getTimestampPtp()) == 7654321);
        REQUIRE_FALSE(dst.packets.empty());
        REQUIRE(dst.packets[0].acceleroMeter.getTimestampPtp().has_value());
        REQUIRE(toNs(*dst.packets[0].acceleroMeter.getTimestampPtp()) == toNs(std::chrono::time_point<dai::ptp_clock>(123s + 456ns)));
    }
    #endif
}
