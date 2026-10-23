#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>

// internal nodes
#include <depthai/pipeline/node/internal/XLinkIn.hpp>
#include <depthai/pipeline/node/internal/XLinkOut.hpp>

TEST_CASE("Pipeline link and remove") {
    dai::Pipeline p;
    auto x_in = p.create<dai::node::internal::XLinkIn>();
    auto x_out = p.create<dai::node::internal::XLinkOut>();
    x_in->out.link(x_out->input);
    p.remove(x_in);
}

TEST_CASE("Pipeline node creation, link, unlink and removal") {
    dai::Pipeline p;
    auto cam = p.create<dai::node::ImageManip>();
    auto xlink = p.create<dai::node::internal::XLinkOut>();

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 2);

    cam->out.link(xlink->input);

    REQUIRE(p.getConnections().size() == 1);

    p.remove(xlink);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 1);

    p.remove(cam);

    REQUIRE(p.getConnections().size() == 0);
    REQUIRE(p.getAllNodes().size() == 0);
}

TEST_CASE("Pipeline closes devices it constructs on stop", "[device-ownership]") {
    std::shared_ptr<dai::Device> device;
    SECTION("Implicit device") {
        dai::Pipeline pipeline;
        device = pipeline.getDefaultDevice();
        pipeline.start();
        pipeline.stop();
        pipeline.wait();
        REQUIRE(device->isClosed());
    }
    SECTION("Device constructed by addDevice") {
        dai::Pipeline pipeline(false);
        const auto info = dai::Device::getAllAvailableDevices().at(0);
        SECTION("DeviceInfo") {
            device = pipeline.addDevice(info);
        }
        SECTION("Device id") {
            device = pipeline.addDevice(info.getDeviceId());
        }
        // Re-registering an owned device must not transfer ownership.
        REQUIRE(pipeline.addDevice(device) == device);
        pipeline.start();
        pipeline.stop();
        pipeline.wait();
        REQUIRE(device->isClosed());
    }
}

TEST_CASE("Pipeline leaves caller-supplied devices open", "[device-ownership]") {
    const auto device = std::make_shared<dai::Device>();
    const auto connection = device->getConnection();
    SECTION("Constructor") {
        dai::Pipeline pipeline(device);
        pipeline.start();
        pipeline.stop();
        pipeline.wait();
        REQUIRE_FALSE(device->isClosed());
    }
    SECTION("addDevice") {
        dai::Pipeline pipeline(false);
        REQUIRE(pipeline.addDevice(device) == device);
        pipeline.start();
        pipeline.stop();
        pipeline.wait();
        REQUIRE_FALSE(device->isClosed());
    }
    SECTION("Device passed to a node") {
        dai::Pipeline pipeline(false);
        pipeline.create<dai::node::ImageManip>(device);
        if(device->getPlatform() != dai::Platform::RVC4) {
            REQUIRE_THROWS_WITH(pipeline.start(), Catch::Matchers::ContainsSubstring("requires RVC4 firmware"));
        } else {
            pipeline.start();
            pipeline.stop();
            pipeline.wait();
            REQUIRE_FALSE(device->isPipelineRunning());
        }
    }
    REQUIRE_FALSE(device->isClosed());
    REQUIRE(device->getConnection() == connection);
}
