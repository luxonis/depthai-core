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
    ScopeHelper(const std::function<void()>& init,    // NOLINT(bugprone-easily-swappable-parameters)
                const std::function<void()>& cleanup  // NOLINT(bugprone-easily-swappable-parameters)
                )
        : cleanup(cleanup) {
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
    const auto& colorCameraConfigs = getColorCameraConfigs(*device);

    const auto maxMode = std::max_element(colorCameraConfigs.begin(), colorCameraConfigs.end(), [](const auto& first, const auto& second) {
        return first.width * first.height < second.width * second.height;
    });
    if(maxMode == colorCameraConfigs.end()) {
        throw std::runtime_error("No color camera sensor configs found");
    }
    std::srand(std::time(nullptr));
    int width = std::abs(std::rand()) % (*maxMode).width;
    int height = std::abs(std::rand()) % (*maxMode).height;
    // INFO TAKEN FROM RVC2 FW: width & height must be greater or equal to 32 pixels (empirical tests)
    // TODO(jakgra) also handle this in camera node / throw error there
    width = width < 32 ? 32 : width;
    height = height < 32 ? 32 : height;
    return {width, height};
}

void testResolution(std::optional<std::tuple<uint32_t, uint32_t>> wantedSize = std::nullopt) {
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::Camera>();

    const auto size = wantedSize ? *wantedSize : getRandomResolution(pipeline);
    std::cout << "TESTING RESOLUTION: " << std::get<0>(size) << "x" << std::get<1>(size) << "\n" << std::flush;
    // TODO(jakgra) add this limitation on RVC2 devices but not RVC3 devices
    // int maxIspVideoWidth = std::min(width, 3840);
    // int maxIspVideoHeight = std::min(height, 2160);

    dai::ImgFrameCapability cap;
    cap.size.value = size;
    // cap.encoding = dai::ImgFrame::Type::RGB888p;
    cap.encoding = dai::ImgFrame::Type::RAW10;
    auto* output = camRgb->requestNewOutput(cap);
    const auto queueFrames = output->getQueue();

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

/*
TEST_CASE("prev_broken_resolutions") {
    const auto resolution = GENERATE(table<uint32_t, uint32_t>({{3860, 2587} , {3951, 1576}, {909, 909}, {444, 888}}));
    testResolution(resolution);
}
*/

TEST_CASE("common_resolutions") {
    const auto resolution = GENERATE(table<uint32_t, uint32_t>({{1920, 1080}, {300, 300}, {640, 640}, {800, 600}}));
    testResolution(resolution);
}

/*
TEST_CASE("random_resolutions") {
    (void)GENERATE(repeat(20, value(0)));
    testResolution();
}
*/

// TEST_CASE("multiple_resolutions") {}
