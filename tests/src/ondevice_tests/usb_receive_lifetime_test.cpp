#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <memory>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"

TEST_CASE("USB frames remain valid across RPC, close and reconnect") {
    using namespace std::chrono_literals;
    std::vector<uint8_t> expected(4177920);
    for(size_t index = 0; index < expected.size(); ++index) expected[index] = index % 251;
    std::vector<std::shared_ptr<dai::Buffer>> held;
    std::string identity;
    {
        dai::Pipeline pipeline;
        auto device = pipeline.getDefaultDevice();
        const auto info = device->getDeviceInfo();
        if(info.protocol != X_LINK_USB_EP && info.protocol != X_LINK_USB_VSC) SKIP("Requires a USB device");
        identity = info.getDeviceId();
        auto source = pipeline.create<dai::node::BenchmarkOut>();
        source->setRunOnHost(false);
        source->setFps(10);
        auto input = source->input.createInputQueue();
        auto output = source->out.createOutputQueue(2, false);
        pipeline.start();
        auto seed = std::make_shared<dai::Buffer>();
        seed->setData(expected);
        input->send(seed);
        for(unsigned index = 0; index < 8; ++index) {
            bool timedOut = false;
            auto frame = output->get<dai::Buffer>(5s, timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(frame != nullptr);
            auto data = frame->getData();
            REQUIRE(data.size() == expected.size());
            REQUIRE(data.front() == expected.front());
            REQUIRE(data.back() == expected.back());
            data.front() = index + 1;
            data.back() = 255 - index;
            held.push_back(std::move(frame));
        }
        bool timedOut = false;
        auto following = output->get<dai::Buffer>(5s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(following != nullptr);
        const auto followingData = std::as_const(*following).getData();
        REQUIRE(followingData.size() == expected.size());
        REQUIRE(std::equal(followingData.begin(), followingData.end(), expected.begin()));
        REQUIRE(device->getDeviceId() == device->getDeviceInfo().getDeviceId());
        pipeline.stop();
        pipeline.wait();
        device->close();
        REQUIRE(device->isClosed());
    }
    {
        dai::Device reconnected{dai::DeviceInfo(identity)};
        REQUIRE(reconnected.getDeviceInfo().getDeviceId() == identity);
        REQUIRE(reconnected.getDeviceId() == reconnected.getDeviceInfo().getDeviceId());
        reconnected.close();
    }
    for(size_t index = 0; index < held.size(); ++index) {
        expected.front() = index + 1;
        expected.back() = 255 - index;
        const auto data = std::as_const(*held[index]).getData();
        REQUIRE(data.size() == expected.size());
        REQUIRE(std::equal(data.begin(), data.end(), expected.begin()));
    }
}
