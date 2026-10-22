#include <catch2/catch_all.hpp>
#include <chrono>
#include <future>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "depthai/depthai.hpp"

// Host-only checks for the multi-device pipeline API (no device required)

namespace {
class ShutdownNode : public dai::node::CustomThreadedNode<ShutdownNode> {
   public:
    std::promise<void> stopEntered;
    std::promise<void> releaseStop;
    std::promise<void> requestFinished;
    std::shared_future<void> stopReleased{releaseStop.get_future()};
    std::shared_future<void> requestGate;

    void run() override {
        if(requestGate.valid()) requestGate.wait();
        stopPipeline();
        requestFinished.set_value();
    }

    void onStop() override {
        stopEntered.set_value();
        stopReleased.wait();
    }
};
}  // namespace

TEST_CASE("Host node shutdown completes before wait or pipeline destruction returns") {
    using namespace std::chrono_literals;
    const bool destroyPipeline = GENERATE(false, true);
    CAPTURE(destroyPipeline);
    std::optional<dai::Pipeline> pipeline(std::in_place, false);
    auto node = pipeline->create<ShutdownNode>();
    auto entered = node->stopEntered.get_future();
    auto requested = node->requestFinished.get_future();
    pipeline->start();
    // The request must return on the node thread while shutdown is still blocked.
    const auto enteredStatus = entered.wait_for(5s);
    const auto requestedStatus = requested.wait_for(5s);
    if(enteredStatus != std::future_status::ready || requestedStatus != std::future_status::ready) {
        node->releaseStop.set_value();
        FAIL("Host node could not request asynchronous shutdown");
    }
    auto completion = std::async(std::launch::async, [&] {
        if(destroyPipeline) {
            pipeline.reset();
        } else {
            pipeline->wait();
        }
    });
    CHECK(completion.wait_for(100ms) == std::future_status::timeout);
    node->releaseStop.set_value();
    completion.get();
    // Also join the node when testing destruction; retaining the node is intentional.
    node->wait();
}

TEST_CASE("Pipeline destruction can race with a host shutdown request") {
    using namespace std::chrono_literals;
    for(int attempt = 0; attempt < 32; ++attempt) {
        CAPTURE(attempt);
        std::optional<dai::Pipeline> pipeline(std::in_place, false);
        auto node = pipeline->create<ShutdownNode>();
        std::promise<void> request;
        node->requestGate = request.get_future().share();
        node->releaseStop.set_value();
        auto requested = node->requestFinished.get_future();
        pipeline->start();
        // Release the node and the last pipeline owner together, without waiting for the request.
        request.set_value();
        pipeline.reset();
        CHECK(requested.wait_for(5s) == std::future_status::ready);
        node->wait();
        // A retained node must also be safe to call after its pipeline has gone away.
        CHECK_NOTHROW(node->stopPipeline());
    }
}

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

TEST_CASE("DeviceInfo from a dotted quad with leading zeros keeps ANY protocol") {
    // inet_pton semantics: "01.2.3.4" is not an IPv4 address
    REQUIRE(dai::DeviceInfo("01.2.3.4").protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(dai::DeviceInfo("1.2.3.04").protocol == X_LINK_ANY_PROTOCOL);
    REQUIRE(dai::DeviceInfo("256.1.1.1").protocol == X_LINK_ANY_PROTOCOL);
}

TEST_CASE("DeviceInfo from a USB port path that looks like an IPv4 address is a name-only lookup") {
    // XLink names a USB device on bus 1 behind two hubs "1.2.1.4"; it infers TCP_IP but stays
    // reachable because XLinkConnection::findFirstSuitableDevice retries the name over USB
    auto info = dai::DeviceInfo("1.2.1.4");
    REQUIRE(info.protocol == X_LINK_TCP_IP);
    REQUIRE(info.name == "1.2.1.4");
    REQUIRE(info.getDeviceId().empty());
    REQUIRE(info.state == X_LINK_ANY_STATE);
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

TEST_CASE("Input::getSourceDevice resolves through a copy handed out by getInputs()") {
    dai::Pipeline p(false);
    auto producer = p.create<dai::node::Sync>();
    auto consumer = p.create<dai::node::Sync>();
    producer->out.link(consumer->inputs["a"]);

    // Host producer: nullptr, whether asked on the node's own input or on a copy
    REQUIRE(consumer->inputs["a"].getSourceDevice() == nullptr);
    bool found = false;
    for(auto input : consumer->getInputs()) {
        if(input.getName() == "a") {
            found = true;
            REQUIRE_NOTHROW(input.getSourceDevice());
            REQUIRE(input.getSourceDevice() == nullptr);
        }
    }
    REQUIRE(found);
}

TEST_CASE("InputMap::getSourceDevices keeps grouped inputs apart") {
    dai::Pipeline p(false);
    auto producer = p.create<dai::node::Sync>();
    auto consumer = p.create<dai::node::Sync>();
    producer->out.link(consumer->inputs["x"]);
    producer->out.link(consumer->inputs[std::pair<std::string, std::string>{"grpA", "x"}]);
    producer->out.link(consumer->inputs[std::pair<std::string, std::string>{"grpB", "x"}]);

    auto sources = consumer->inputs.getSourceDevices();
    REQUIRE(sources.size() == 3);
    REQUIRE(sources.count("x") == 1);
    REQUIRE(sources.count("grpA/x") == 1);
    REQUIRE(sources.count("grpB/x") == 1);
    for(const auto& entry : sources) {
        REQUIRE(entry.second == nullptr);
    }
}

namespace {
class IdleHostNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, IdleHostNode> {
   public:
    constexpr static const char* NAME = "IdleHostNode";
    void run() override {
        while(mainLoop()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
};
}  // namespace

TEST_CASE("Host-only pipeline reports running until stop has completed") {
    dai::Pipeline p(false);
    p.create<IdleHostNode>();
    REQUIRE_FALSE(p.isRunning());
    p.start();
    REQUIRE(p.isRunning());
    p.stop();
    REQUIRE_FALSE(p.isRunning());
    p.wait();

    // Can be started again after a stop
    p.start();
    REQUIRE(p.isRunning());
    p.stop();
    p.wait();
    REQUIRE_FALSE(p.isRunning());
}

TEST_CASE("Host-only pipeline run() returns once stopped from another thread") {
    dai::Pipeline p(false);
    p.create<IdleHostNode>();
    std::thread stopper([&p]() {
        while(!p.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        p.stop();
    });
    p.run();
    stopper.join();
    REQUIRE_FALSE(p.isRunning());
}

TEST_CASE("Pipeline stopOnDeviceLoss is off by default and can be toggled") {
    dai::Pipeline p(false);
    REQUIRE_FALSE(p.getStopOnDeviceLoss());
    p.setStopOnDeviceLoss(true);
    REQUIRE(p.getStopOnDeviceLoss());
    p.setStopOnDeviceLoss(false);
    REQUIRE_FALSE(p.getStopOnDeviceLoss());
}
