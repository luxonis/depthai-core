#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// std
#include <atomic>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

void makeInfo(dai::Pipeline& p) {
    auto info = p.create<dai::node::SystemLogger>();
    auto xo = p.create<dai::node::XLinkOut>();
    xo->setStreamName("sysinfo");
    info->out.link(xo->input);
}

void verifyInfo(dai::Device& d) {
    auto infoQ = d.getOutputQueue("sysinfo");
    if(infoQ->isClosed()) throw std::runtime_error("queue is not open");
}

TEST_CASE("usb2Mode == true") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, true);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::HIGH") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::HIGH);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::SUPER") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::SUPER);
    verifyInfo(d);
}

TEST_CASE("UsbSpeed::SUPER_PLUS") {
    dai::Pipeline p;
    makeInfo(p);
    dai::Device d(p, dai::UsbSpeed::SUPER_PLUS);
    verifyInfo(d);
}
