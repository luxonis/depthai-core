#include <algorithm>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "buffer_test_utils.hpp"
#include "camera_test_utils.hpp"
#include "depthai/depthai.hpp"

using buffer_test::requireNoMessage;
using camera_test::drain;
using camera_test::nextFrame;
using camera_test::send;

TEST_CASE("Camera discovery APIs list the same cameras", "[camera-hil]") {
    dai::Device device;
    const auto features = device.getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());

    std::set<dai::CameraBoardSocket> sockets;
    std::unordered_map<dai::CameraBoardSocket, std::string> names;
    for(const auto& feature : features) {
        CAPTURE(feature.socket, feature.sensorName);
        REQUIRE(sockets.insert(feature.socket).second);
        REQUIRE_FALSE(feature.sensorName.empty());
        names[feature.socket] = feature.sensorName;

        // Every config is one of the sensor's types, and the sensor's size is its largest config
        REQUIRE_FALSE(feature.supportedTypes.empty());
        REQUIRE_FALSE(feature.configs.empty());
        int32_t width = 0;
        int32_t height = 0;
        for(const auto& config : feature.configs) {
            CAPTURE(config.width, config.height);
            REQUIRE(std::find(feature.supportedTypes.begin(), feature.supportedTypes.end(), config.type) != feature.supportedTypes.end());
            width = std::max(width, config.width);
            height = std::max(height, config.height);
        }
        REQUIRE(feature.width == width);
        REQUIRE(feature.height == height);
    }

    const auto connected = device.getConnectedCameras();
    REQUIRE(connected.size() == features.size());
    REQUIRE(std::set<dai::CameraBoardSocket>(connected.begin(), connected.end()) == sockets);
    REQUIRE(device.getCameraSensorNames() == names);
}

TEST_CASE("Camera picks the platform's default output type", "[camera-hil]") {
    dai::Pipeline p;
    const auto device = p.getDefaultDevice();
    const auto platform = device->getPlatform();
    const auto features = device->getConnectedCameraFeatures();
    REQUIRE_FALSE(features.empty());

    struct Output {
        dai::CameraBoardSocket socket;
        dai::ImgFrame::Type expected;
        std::shared_ptr<dai::MessageQueue> queue;
    };
    std::vector<Output> outputs;
    for(const auto& feature : features) {
        dai::ImgFrame::Type expected{};
        if(platform == dai::Platform::RVC2NG) {
            const bool mono = std::all_of(feature.configs.begin(), feature.configs.end(), [](const auto& c) { return c.type == dai::CameraSensorType::MONO; });
            expected = mono ? dai::ImgFrame::Type::GRAY8 : dai::ImgFrame::Type::NV16;
        } else if(platform == dai::Platform::RVC4) {
            expected = dai::ImgFrame::Type::NV12;
        } else {
            FAIL("No expected default output type for platform " << dai::platform2string(platform));
        }
        auto camera = p.create<dai::node::Camera>()->build(feature.socket);
        outputs.push_back({feature.socket, expected, camera->requestOutput({320, 240})->createOutputQueue(4, false)});
    }
    p.start();
    for(const auto& output : outputs) {
        CAPTURE(output.socket);
        const auto frame = nextFrame(output.queue);
        REQUIRE(frame->getWidth() == 320);
        REQUIRE(frame->getHeight() == 240);
        REQUIRE(frame->getType() == output.expected);
    }
}

TEST_CASE("Every connected camera pauses, resumes and keeps the frames it delivered", "[camera-hil]") {
    using namespace std::chrono_literals;
    constexpr unsigned int WIDTH = 320;
    constexpr unsigned int HEIGHT = 240;

    struct Stream {
        dai::CameraBoardSocket socket;
        std::shared_ptr<dai::MessageQueue> frames;
        std::shared_ptr<dai::InputQueue> control;
        std::shared_ptr<dai::ImgFrame> last;
    };
    std::vector<camera_test::HeldFrame> held;
    {
        dai::Pipeline p;
        const auto features = p.getDefaultDevice()->getConnectedCameraFeatures();
        REQUIRE_FALSE(features.empty());
        std::vector<Stream> streams;
        for(const auto& feature : features) {
            auto camera = p.create<dai::node::Camera>()->build(feature.socket);
            camera->initialControl.setStopStreaming();
            auto* output = camera->requestOutput({WIDTH, HEIGHT}, dai::ImgFrame::Type::GRAY8);
            REQUIRE(output != nullptr);
            streams.push_back({feature.socket, output->createOutputQueue(4, false), camera->inputControl.createInputQueue(), nullptr});
        }
        p.start();

        {
            INFO("The initial stop control keeps every camera paused");
            for(auto& stream : streams) {
                CAPTURE(stream.socket);
                requireNoMessage(stream.frames);
            }
        }
        {
            INFO("A start control starts every camera, and device timestamps rise from frame to frame");
            for(auto& stream : streams) send(stream.control, dai::CameraControl().setStartStreaming());
            for(auto& stream : streams) {
                CAPTURE(stream.socket);
                auto frame = nextFrame(stream.frames);
                REQUIRE(frame->getWidth() == WIDTH);
                REQUIRE(frame->getHeight() == HEIGHT);
                REQUIRE(frame->getType() == dai::ImgFrame::Type::GRAY8);
                held.push_back(camera_test::hold(frame));
                for(int i = 0; i < 8; ++i) {
                    auto next = nextFrame(stream.frames);
                    REQUIRE(next->getTimestampDevice() > frame->getTimestampDevice());
                    REQUIRE(next->getSequenceNum() > frame->getSequenceNum());
                    frame = next;
                }
                stream.last = frame;
            }
        }
        {
            INFO("Picture controls keep frames coming");
            for(auto& stream : streams) send(stream.control, dai::CameraControl().setBrightness(0).setContrast(0).setSaturation(0).setSharpness(2));
            for(auto& stream : streams) {
                CAPTURE(stream.socket);
                for(int i = 0; i < 5; ++i) {
                    auto frame = nextFrame(stream.frames);
                    REQUIRE(frame->getSequenceNum() > stream.last->getSequenceNum());
                    stream.last = frame;
                }
            }
        }
        {
            INFO("A stop control pauses every camera");
            for(auto& stream : streams) send(stream.control, dai::CameraControl().setStopStreaming());
            std::this_thread::sleep_for(300ms);
            for(auto& stream : streams) drain(stream.frames);
            for(auto& stream : streams) {
                CAPTURE(stream.socket);
                requireNoMessage(stream.frames);
            }
        }
        {
            INFO("A start control resumes every camera with new frames");
            for(auto& stream : streams) send(stream.control, dai::CameraControl().setStartStreaming());
            for(auto& stream : streams) {
                CAPTURE(stream.socket);
                auto frame = nextFrame(stream.frames);
                REQUIRE(frame->getTimestampDevice() > stream.last->getTimestampDevice());
            }
        }

        p.stop();
        p.wait();
        INFO("Frames keep their payload after the pipeline stops");
        camera_test::requireUnchanged(held);
    }
    INFO("Frames keep their payload after the device closes");
    camera_test::requireUnchanged(held);
}
