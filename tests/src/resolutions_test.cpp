#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <stdexcept>

// Libraries
#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <tuple>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

std::vector<dai::CameraSensorConfig> getColorCameraConfigs(dai::Device& device) {
    dai::ColorCameraProperties sss;
    const auto& cameraFeatures = device.getConnectedCameraFeatures();
    const auto& camera = std::find_if(
        cameraFeatures.begin(), cameraFeatures.end(), [](const dai::CameraFeatures& itr) -> bool { return itr.socket == dai::CameraBoardSocket::CAM_A; });
    if(camera == cameraFeatures.end()) {
        throw std::runtime_error("Device doesn't support ColorCamera");
    }
    std::vector<dai::CameraSensorConfig> colorCameraModes;
    std::copy_if(camera->configs.begin(), camera->configs.end(), std::back_inserter(colorCameraModes), [](const auto& itr) {
        return itr.type == dai::CameraSensorType::COLOR;
    });
    return colorCameraModes;
}

class ScopeHelper {
   public:
    ScopeHelper(const std::function<void()>& init,  // NOLINT(bugprone-easily-swappable-parameters)
                std::function<void()> cleanup       // NOLINT(bugprone-easily-swappable-parameters)
                )
        : cleanup(std::move(cleanup)) {
        init();
    }
    ~ScopeHelper() {
        cleanup();
    }
    ScopeHelper(const ScopeHelper&) = delete;
    ScopeHelper(ScopeHelper&&) = delete;
    ScopeHelper& operator=(const ScopeHelper&) = delete;
    ScopeHelper& operator=(ScopeHelper&&) = delete;

   private:
    std::function<void()> init;
    std::function<void()> cleanup;
};

std::tuple<uint32_t, uint32_t> getRandomResolution(dai::Pipeline& pipeline) {
    const auto device = pipeline.getDefaultDevice();
    if(!device) {
        throw std::runtime_error("The pipeline was without device");
    }
    int maxWidth = 0;
    int maxHeight = 0;
    if(device->getDeviceInfo().platform == X_LINK_RVC4) {
        // TODO(jakgra) remove this if statement for RVC4 once RVC4 getConnectedCameraFetaures() works
        maxWidth = 4000;
        maxHeight = 3000;
    } else {
        const auto& colorCameraConfigs = getColorCameraConfigs(*device);
        const auto maxMode = std::max_element(colorCameraConfigs.begin(), colorCameraConfigs.end(), [](const auto& first, const auto& second) {
            return first.width * first.height < second.width * second.height;
        });
        if(maxMode == colorCameraConfigs.end()) {
            throw std::runtime_error("No color camera sensor configs found");
        }
        maxWidth = (*maxMode).width;
        maxHeight = (*maxMode).height;
    }
    std::srand(std::time(nullptr));
    int width = std::abs(std::rand()) % maxWidth;
    int height = std::abs(std::rand()) % maxHeight;
    width = width - width % 2;
    height = height - height % 2;
    // TODO(jakgra) remove this when odd-sized resolutions are supported
    // INFO TAKEN FROM RVC2 FW: width & height must be greater or equal to 32 pixels (empirical tests)
    // TODO(jakgra) also handle this in camera node / throw error there
    // TODO(jakgra) check if needed on RVC4 and RVC3
    width = width < 32 ? 32 : width;
    height = height < 32 ? 32 : height;
    return {width, height};
}

void testResolution(std::optional<std::tuple<uint32_t, uint32_t>> wantedSize = std::nullopt) {
    dai::DeviceInfo info("10.12.110.219");
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);

    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::Camera>();

    const auto size = wantedSize ? *wantedSize : getRandomResolution(pipeline);
    std::cout << "TESTING RESOLUTION: " << std::get<0>(size) << "x" << std::get<1>(size) << "\n" << std::flush;
    // TODO(jakgra) add this limitation on RVC2 devices but not RVC3 devices and not on RVC4 devices
    // int maxIspVideoWidth = std::min(width, 3840);
    // int maxIspVideoHeight = std::min(height, 2160);

    dai::ImgFrameCapability cap;
    cap.size.value = std::make_pair(std::get<0>(size), std::get<1>(size));
    // cap.encoding = dai::ImgFrame::Type::RGB888i;
    // cap.encoding = dai::ImgFrame::Type::BGR888i;
    // cap.encoding = dai::ImgFrame::Type::NV12;
    cap.encoding = dai::ImgFrame::Type::BGR888i;
    auto* output = camRgb->requestOutput(cap, true);
    const auto queueFrames = output->createQueue();

    const auto& qRgb = queueFrames;
    ScopeHelper scopeHelper([&pipeline]() { pipeline.start(); },
                            [&pipeline]() {
                                pipeline.stop();
                                pipeline.wait();
                                std::this_thread::sleep_for(std::chrono::seconds(10));
                            });
    for(int counter = 0; counter < 10; ++counter) {
        const auto inRgb = qRgb->get<dai::ImgFrame>();
        if(inRgb) {
            REQUIRE(inRgb->getWidth() == std::get<0>(size));
            REQUIRE(inRgb->getHeight() == std::get<1>(size));
        }
        counter++;
    }
}

struct MultipleResHelper {
    std::tuple<uint32_t, uint32_t> size;
    std::shared_ptr<dai::MessageQueue> queue;
    uint32_t count{0};
};

void testMultipleResolutions(const std::vector<std::tuple<uint32_t, uint32_t>>& wantedSizes) {  // NOLINT(readability-function-cognitive-complexity)
    dai::DeviceInfo info("10.12.110.219");
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    const auto device = std::make_shared<dai::Device>(info);
    dai::Pipeline pipeline(device);
    auto camRgb = pipeline.create<dai::node::Camera>();
    std::vector<MultipleResHelper> helpers;
    std::cout << "TESTING MULTIPLE RESOLUTIONS: ";
    for(const auto& wantedSize : wantedSizes) {
        std::cout << std::get<0>(wantedSize) << "x" << std::get<1>(wantedSize) << " ";
        auto& helper = helpers.emplace_back(MultipleResHelper());
        helper.count = 0;
        helper.size = wantedSize;
        dai::ImgFrameCapability cap;
        cap.size.value = std::make_pair(std::get<0>(wantedSize), std::get<1>(wantedSize));
        // cap.encoding = dai::ImgFrame::Type::RGB888i;
        // cap.encoding = dai::ImgFrame::Type::BGR888i;
        // cap.encoding = dai::ImgFrame::Type::NV12;
        cap.encoding = dai::ImgFrame::Type::BGR888i;
        auto* output = camRgb->requestOutput(cap, true);
        helper.queue = output->createQueue();
    }
    std::cout << "\n" << std::flush;
    ScopeHelper scopeHelper([&pipeline]() { pipeline.start(); },
                            [&pipeline]() {
                                pipeline.stop();
                                pipeline.wait();
                                std::this_thread::sleep_for(std::chrono::seconds(10));
                            });
    while(true) {
        bool allFramesReceived = true;
        for(auto& helper : helpers) {
            const auto inRgb = helper.queue->tryGet<dai::ImgFrame>();
            if(inRgb) {
                std::cout << "Got frame[" << helper.count << "] for size " << std::get<0>(helper.size) << "x" << std::get<1>(helper.size) << "\n" << std::flush;
                REQUIRE(inRgb->getWidth() == std::get<0>(helper.size));
                REQUIRE(inRgb->getHeight() == std::get<1>(helper.size));
                ++helper.count;
            }
            if(helper.count < 10) {
                allFramesReceived = false;
            }
        }
        if(allFramesReceived) {
            std::cout << "Got all frames\n" << std::flush;
            break;
        }
    }
}

TEST_CASE("prev_broken_resolutions") {
    // TODO(jakgra) fix odd-sized resolutions
    // const auto resolution = GENERATE(table<uint32_t, uint32_t>({{3860, 2587}, {3951, 1576}, {909, 909}, {444, 888}}));
    const auto resolution = GENERATE(table<uint32_t, uint32_t>({{444, 888}}));
    testResolution(resolution);
}

TEST_CASE("common_resolutions") {
    const auto resolution = GENERATE(table<uint32_t, uint32_t>({{1920, 1080}, {300, 300}, {640, 640}, {800, 600}, {640, 480}}));
    testResolution(resolution);
}

TEST_CASE("random_resolutions") {
    (void)GENERATE(repeat(20, value(0)));
    testResolution();
}

TEST_CASE("multiple_resolutions") {
    const std::vector<std::vector<std::tuple<uint32_t, uint32_t>>> resolutionsToTest{
        {{300, 300}, {640, 640}, {1920, 1080}},  // standard
        {{4000, 3000}, {1000, 1000}}             // a bit more high res
    };
    for(const auto& wantedSizes : resolutionsToTest) {
        testMultipleResolutions(wantedSizes);
    }
}
