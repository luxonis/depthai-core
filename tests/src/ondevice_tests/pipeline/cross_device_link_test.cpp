// Links between nodes on two devices, end to end: direct (PeerLinkOut -> PeerLinkIn) between two RVC4 devices
// on one network, relayed through the host otherwise. Needs two RVC4 devices, or an RVC2 and an RVC4.

#include <catch2/catch_all.hpp>
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/internal/PeerLinkIn.hpp"
#include "depthai/pipeline/node/internal/PeerLinkOut.hpp"

using namespace std::chrono_literals;

namespace {

constexpr std::uint32_t WIDTH = 640;
constexpr std::uint32_t HEIGHT = 400;
constexpr float FPS = 30.0f;

std::vector<dai::DeviceInfo> availableDevices(dai::Platform platform) {
    std::vector<dai::DeviceInfo> result;
    for(const auto& info : dai::Device::getAllAvailableDevices()) {
        if(info.platform == (platform == dai::Platform::RVC4 ? X_LINK_RVC4 : X_LINK_MYRIAD_X)) result.push_back(info);
    }
    return result;
}

// Producer and consumer devices, or std::nullopt when the setup is not available
std::optional<std::pair<dai::DeviceInfo, dai::DeviceInfo>> pickDevices(dai::Platform producer, dai::Platform consumer) {
    auto producers = availableDevices(producer);
    auto consumers = availableDevices(consumer);
    if(producer == consumer) {
        if(producers.size() < 2) return std::nullopt;
        return std::make_pair(producers[0], producers[1]);
    }
    if(producers.empty() || consumers.empty()) return std::nullopt;
    return std::make_pair(producers[0], consumers[0]);
}

struct CrossDevicePipeline {
    dai::Pipeline pipeline{false};
    std::shared_ptr<dai::Device> producerDevice;
    std::shared_ptr<dai::Device> consumerDevice;
    dai::Node::Output* cameraOut = nullptr;
    std::shared_ptr<dai::node::ImageManip> manip;
    std::shared_ptr<dai::MessageQueue> output;
};

// Camera on the producer -> ImageManip on the consumer -> host
std::unique_ptr<CrossDevicePipeline> buildCameraToManip(const dai::DeviceInfo& producer, const dai::DeviceInfo& consumer, dai::CrossDeviceLinkMode mode) {
    auto setup = std::make_unique<CrossDevicePipeline>();
    setup->producerDevice = setup->pipeline.addDevice(producer);
    setup->consumerDevice = setup->pipeline.addDevice(consumer);
    setup->pipeline.setCrossDeviceLinkMode(mode);
    auto camera = setup->pipeline.create<dai::node::Camera>(setup->producerDevice)->build();
    setup->cameraOut = camera->requestOutput({WIDTH, HEIGHT}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS);
    setup->manip = setup->pipeline.create<dai::node::ImageManip>(setup->consumerDevice);
    setup->manip->initialConfig->setOutputSize(WIDTH / 2, HEIGHT / 2);
    setup->manip->setMaxOutputFrameSize(WIDTH * HEIGHT * 3);
    setup->cameraOut->link(setup->manip->inputImage);
    setup->output = setup->manip->out.createOutputQueue(8, false);
    return setup;
}

int countNodes(const dai::PipelineSchema& schema, const std::string& name) {
    int count = 0;
    for(const auto& node : schema.nodes) {
        if(node.second.name == name) count++;
    }
    return count;
}

// Receives frames for a while and returns the rate
double receiveFrames(const std::shared_ptr<dai::MessageQueue>& output, int frames) {
    bool timedOut = false;
    auto first = output->get<dai::ImgFrame>(20s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(first != nullptr);
    const auto start = std::chrono::steady_clock::now();
    for(int i = 0; i < frames; i++) {
        auto frame = output->get<dai::ImgFrame>(3s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame->getWidth() == WIDTH / 2);
        REQUIRE(frame->getHeight() == HEIGHT / 2);
    }
    return frames / std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
}

}  // namespace

TEST_CASE("Two RVC4 devices link directly, without the host") {
    const auto devices = pickDevices(dai::Platform::RVC4, dai::Platform::RVC4);
    if(!devices) SKIP("Needs two RVC4 devices");
    auto mode = GENERATE(dai::CrossDeviceLinkMode::AUTO, dai::CrossDeviceLinkMode::DIRECT);
    CAPTURE(static_cast<int>(mode));

    auto setup = buildCameraToManip(devices->first, devices->second, mode);
    setup->pipeline.build();

    // The link is a PeerLinkOut on the producer and a PeerLinkIn on the consumer, not a host relay
    const auto bridges = setup->cameraOut->getPeerLinkBridges();
    REQUIRE(bridges.size() == 1);
    REQUIRE(bridges[0]->peerLinkOut->getDevice() == setup->producerDevice);
    REQUIRE(bridges[0]->peerLinkIn->getDevice() == setup->consumerDevice);
    REQUIRE_FALSE(bridges[0]->peerLinkIn->getProducerEndpoints().empty());
    REQUIRE(setup->cameraOut->getXLinkBridge() == nullptr);
    const auto schema = setup->pipeline.getPipelineSchema();
    REQUIRE(countNodes(schema, "PeerLinkOut") == 1);
    REQUIRE(countNodes(schema, "PeerLinkIn") == 1);
    // The only host bridge left carries ImageManip's output to the host
    REQUIRE(countNodes(schema, "XLinkInHost") == 1);
    REQUIRE(countNodes(schema, "XLinkOutHost") == 0);

    setup->pipeline.start();
    const double fps = receiveFrames(setup->output, 90);
    CAPTURE(fps);
    REQUIRE(fps > FPS * 0.8);
    setup->pipeline.stop();
}

TEST_CASE("Two RVC4 devices still link through the host relay when asked to") {
    const auto devices = pickDevices(dai::Platform::RVC4, dai::Platform::RVC4);
    if(!devices) SKIP("Needs two RVC4 devices");

    auto setup = buildCameraToManip(devices->first, devices->second, dai::CrossDeviceLinkMode::HOST_RELAY);
    setup->pipeline.build();
    REQUIRE(setup->cameraOut->getPeerLinkBridges().empty());
    REQUIRE(setup->cameraOut->getXLinkBridge() != nullptr);
    const auto schema = setup->pipeline.getPipelineSchema();
    REQUIRE(countNodes(schema, "PeerLinkOut") == 0);
    REQUIRE(countNodes(schema, "XLinkOutHost") == 1);

    setup->pipeline.start();
    const double fps = receiveFrames(setup->output, 90);
    CAPTURE(fps);
    REQUIRE(fps > FPS * 0.8);
    setup->pipeline.stop();
}

TEST_CASE("Direct links carry a MessageGroup, one link per consuming input") {
    const auto devices = pickDevices(dai::Platform::RVC4, dai::Platform::RVC4);
    if(!devices) SKIP("Needs two RVC4 devices");

    dai::Pipeline pipeline(false);
    auto producer = pipeline.addDevice(devices->first);
    auto consumer = pipeline.addDevice(devices->second);
    auto camera = pipeline.create<dai::node::Camera>(producer)->build();
    auto* small = camera->requestOutput({WIDTH / 2, HEIGHT / 2}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS);
    auto* large = camera->requestOutput({WIDTH, HEIGHT}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS);
    auto sync = pipeline.create<dai::node::Sync>(producer);
    small->link(sync->inputs["small"]);
    large->link(sync->inputs["large"]);
    // Two consumers of one output on the consuming device get a link each
    auto demux = pipeline.create<dai::node::MessageDemux>(consumer);
    auto demuxSecond = pipeline.create<dai::node::MessageDemux>(consumer);
    sync->out.link(demux->input);
    sync->out.link(demuxSecond->input);
    auto smallOut = demux->outputs["small"].createOutputQueue(8, false);
    auto largeOut = demux->outputs["large"].createOutputQueue(8, false);
    auto secondLargeOut = demuxSecond->outputs["large"].createOutputQueue(8, false);
    pipeline.build();
    REQUIRE(sync->out.getPeerLinkBridges().size() == 2);

    pipeline.start();
    bool timedOut = false;
    for(int i = 0; i < 30; i++) {
        auto smallFrame = smallOut->get<dai::ImgFrame>(20s, timedOut);
        REQUIRE_FALSE(timedOut);
        auto largeFrame = largeOut->get<dai::ImgFrame>(3s, timedOut);
        REQUIRE_FALSE(timedOut);
        auto secondLargeFrame = secondLargeOut->get<dai::ImgFrame>(3s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(smallFrame->getWidth() == WIDTH / 2);
        REQUIRE(largeFrame->getWidth() == WIDTH);
        REQUIRE(secondLargeFrame->getWidth() == WIDTH);
    }
    pipeline.stop();
}

TEST_CASE("An RVC2 device links through the host relay") {
    auto direction = GENERATE(0, 1);
    const auto devices = direction == 0 ? pickDevices(dai::Platform::RVC2, dai::Platform::RVC4) : pickDevices(dai::Platform::RVC4, dai::Platform::RVC2);
    if(!devices) SKIP("Needs an RVC2 and an RVC4 device");
    CAPTURE(direction);

    SECTION("AUTO falls back to the relay") {
        auto setup = buildCameraToManip(devices->first, devices->second, dai::CrossDeviceLinkMode::AUTO);
        setup->pipeline.build();
        REQUIRE(setup->cameraOut->getPeerLinkBridges().empty());
        REQUIRE(setup->cameraOut->getXLinkBridge() != nullptr);
        REQUIRE(countNodes(setup->pipeline.getPipelineSchema(), "PeerLinkOut") == 0);
        setup->pipeline.start();
        const double fps = receiveFrames(setup->output, 60);
        CAPTURE(fps);
        REQUIRE(fps > FPS * 0.5);
        setup->pipeline.stop();
    }
    SECTION("DIRECT fails the build") {
        auto setup = buildCameraToManip(devices->first, devices->second, dai::CrossDeviceLinkMode::DIRECT);
        REQUIRE_THROWS_WITH(setup->pipeline.build(), Catch::Matchers::ContainsSubstring("RVC2"));
    }
}
