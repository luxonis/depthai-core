#include <catch2/catch_all.hpp>

// std
#include <atomic>
#include <chrono>
#include <cstddef>
#include <iostream>
#include <memory>

// Include depthai library
#include <depthai/depthai.hpp>

#include "depthai/common/UsbSpeed.hpp"
#include "depthai/depthai.hpp"

TEST_CASE("Usb modes") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p;

    std::unique_ptr<dai::Device> device;
    SECTION("UsbSpeed::HIGH") {
        device.reset(new dai::Device(p, dai::UsbSpeed::HIGH));
    }

    SECTION("UsbSpeed::SUPER") {
        device.reset(new dai::Device(p, dai::UsbSpeed::SUPER));
    }

    SECTION("UsbSpeed::SUPER_PLUS") {
        device.reset(new dai::Device(p, dai::UsbSpeed::SUPER_PLUS));
    }

    auto infoNode = p.create<dai::node::SystemLogger>();
    auto outputQueue = infoNode->out.createOutputQueue();
    p.start();

    const auto timeout = 1s;
    bool timeoutOccurred = false;
    auto infoFrame = outputQueue->get<dai::SystemInformation>(timeout, timeoutOccurred);

    REQUIRE(!timeoutOccurred);
    REQUIRE(infoFrame != nullptr);
    REQUIRE(infoFrame->leonCssCpuUsage.average >= 0.0f);
    REQUIRE(infoFrame->leonCssCpuUsage.average <= 1.0f);
}

TEST_CASE("Usb config modes") {
    using namespace std;
    using namespace std::chrono;
    using namespace std::chrono_literals;

    dai::Pipeline p(false);

    dai::UsbSpeed speed;
    SECTION("UsbSpeed::HIGH") {
        dai::DeviceBase::Config cfg;
        cfg.board.usb.maxSpeed = dai::UsbSpeed::HIGH;
        speed = dai::UsbSpeed::HIGH;
        p.setBoardConfig(cfg.board);
    }

    SECTION("UsbSpeed::SUPER") {
        dai::DeviceBase::Config cfg;
        cfg.board.usb.maxSpeed = dai::UsbSpeed::SUPER;
        speed = dai::UsbSpeed::SUPER;
        p.setBoardConfig(cfg.board);
    }

    SECTION("UsbSpeed::SUPER_PLUS") {
        dai::DeviceBase::Config cfg;
        cfg.board.usb.maxSpeed = dai::UsbSpeed::SUPER_PLUS;
        speed = dai::UsbSpeed::SUPER_PLUS;
        p.setBoardConfig(cfg.board);
    }

    dai::Device d(p);
    REQUIRE(d.getUsbSpeed() == speed);
}