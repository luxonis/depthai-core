#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// std
#include <atomic>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("UsbSpeed::HIGH") {
    dai::Pipeline p;
    dai::Device d(p, dai::UsbSpeed::HIGH);
}

TEST_CASE("UsbSpeed::SUPER") {
    dai::Pipeline p;
    dai::Device d(p, dai::UsbSpeed::SUPER);
}

TEST_CASE("UsbSpeed::SUPER_PLUS") {
    dai::Pipeline p;
    dai::Device d(p, dai::UsbSpeed::SUPER_PLUS);
}
