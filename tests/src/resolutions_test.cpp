#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <opencv2/imgproc.hpp>
#include <stdexcept>

// Libraries
#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <opencv2/core.hpp>
#include <tuple>

// Includes common necessary includes for development using depthai library
#include "depthai/capabilities/ImgFrameCapability.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/depthai.hpp"

// Test helpers
#include "image_comparator.hpp"

bool getBoolOption(const std::string& key) {
    const char* valueCStr = std::getenv(key.c_str());
    if(valueCStr == nullptr) {
        return false;
    }
    const std::string value(valueCStr);
    return value == "1" || value == "ON";
}

bool isDebug() {
    return getBoolOption("DEPTHAI_TESTS_DEBUG");
}

dai::DeviceInfo getDeviceInfo() {
    const char* valueCStr = std::getenv("DEPTHAI_TESTS_IP");
    if(valueCStr == nullptr) {
        throw std::runtime_error("Please set device ip using environment variable DEPTHAI_TESTS_IP");
    }
    dai::DeviceInfo info(valueCStr);
    info.protocol = X_LINK_TCP_IP;
    info.state = X_LINK_GATE;
    info.platform = X_LINK_RVC4;
    return info;
}

dai::CameraBoardSocket getBoardSocket() {
    const char* valueCStr = std::getenv("DEPTHAI_TESTS_IP");
    if(valueCStr == nullptr) {
        throw std::runtime_error("Please set camera board socket using environment variable DEPTHAI_TESTS_CAM");
    }
    std::string value(valueCStr);
    using E = dai::CameraBoardSocket;
    auto ret = E::CAM_A;
    if(value == "CAM_A") {
        ret = E::CAM_A;
    } else if(value == "CAM_B") {
        ret = E::CAM_B;
    } else if(value == "CAM_C") {
        ret = E::CAM_C;
    } else {
        throw std::runtime_error("Wrong camera board socket in environment variable DEPTHAI_TESTS_CAM. Supported sockets are CAM_A, CAM_B, CAM_C");
    }
    return ret;
}

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
    std::cout << "TESTING SOME RESOLUTION\n" << std::flush;
    const auto device = std::make_shared<dai::Device>(getDeviceInfo());
    std::cout << "TESTING SOME RESOLUTION 2\n" << std::flush;

    dai::Pipeline pipeline(device);
    std::cout << "TESTING SOME RESOLUTION 3\n" << std::flush;

    auto camRgb = pipeline.create<dai::node::Camera>();
    std::cout << "TESTING SOME RESOLUTION 4\n" << std::flush;
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    const auto size = wantedSize ? *wantedSize : getRandomResolution(pipeline);
    std::cout << "TESTING RESOLUTION: " << std::get<0>(size) << "x" << std::get<1>(size) << "\n" << std::flush;
    // TODO(jakgra) add this limitation on RVC2 devices but not RVC3 devices and not on RVC4 devices
    // int maxIspVideoWidth = std::min(width, 3840);
    // int maxIspVideoHeight = std::min(height, 2160);

    std::cout << "TESTING SOME RESOLUTION 5\n" << std::flush;
    dai::ImgFrameCapability cap;
    cap.size.value = std::make_pair(std::get<0>(size), std::get<1>(size));
    // cap.encoding = dai::ImgFrame::Type::RGB888i;
    // cap.encoding = dai::ImgFrame::Type::BGR888i;
    // cap.encoding = dai::ImgFrame::Type::NV12;
    cap.encoding = dai::ImgFrame::Type::BGR888i;
    auto* output = camRgb->requestOutput(cap, true);
    const auto queueFrames = output->createQueue();

    std::cout << "TESTING SOME RESOLUTION 6\n" << std::flush;
    const auto& qRgb = queueFrames;
    ScopeHelper scopeHelper([&pipeline]() { pipeline.start(); },
                            [&pipeline]() {
                                pipeline.stop();
                                pipeline.wait();
                                std::this_thread::sleep_for(std::chrono::seconds(10));
                            });
    int framesCount = 0;
    int framesWantedCount = 10;
    std::cout << "TESTING SOME RESOLUTION 7\n" << std::flush;
    for(int counter = 0; counter < framesWantedCount; ++counter) {
        const auto inRgb = qRgb->get<dai::ImgFrame>();
        if(inRgb) {
            REQUIRE(inRgb->getWidth() == std::get<0>(size));
            REQUIRE(inRgb->getHeight() == std::get<1>(size));
            ++framesCount;
        }
    }
    REQUIRE(framesCount == framesWantedCount);
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void getImages(std::list<std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>>& imgComparesOut,
               std::tuple<uint32_t, uint32_t>& sizeOut,
               dai::ImgResizeMode resizeMode,
               std::optional<std::tuple<uint32_t, uint32_t>> wantedSize = std::nullopt) {
    const auto device = std::make_shared<dai::Device>(getDeviceInfo());

    dai::Pipeline pipeline(device);

    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);

    const auto size = wantedSize ? *wantedSize : getRandomResolution(pipeline);
    sizeOut = size;
    std::cout << "TESTING RESOLUTION WITH CONTENT: " << std::get<0>(size) << "x" << std::get<1>(size) << "\n" << std::flush;
    // TODO(jakgra) add this limitation on RVC2 devices but not RVC3 devices and not on RVC4 devices
    // int maxIspVideoWidth = std::min(width, 3840);
    // int maxIspVideoHeight = std::min(height, 2160);

    dai::ImgFrameCapability capOrig;
    // capOrig.size.value = std::make_pair(960, 720);  // TODO(jakgra) get this from device when supported on rvc4
    capOrig.size.value = std::make_pair(1920, 1440);  // TODO(jakgra) get this from device when supported on rvc4
    capOrig.encoding = dai::ImgFrame::Type::BGR888i;
    const auto queueFramesOrig = camRgb->requestOutput(capOrig, true)->createQueue();

    dai::ImgFrameCapability cap;
    cap.size.value = std::make_pair(std::get<0>(size), std::get<1>(size));
    cap.encoding = dai::ImgFrame::Type::BGR888i;
    // cap.resizeMode = resizeMode;
    cap.resizeMode = dai::ImgResizeMode::CROP;
    auto* output = camRgb->requestOutput(cap, true);
    const auto queueFrames = camRgb->requestOutput(cap, true)->createQueue();

    ScopeHelper scopeHelper([&pipeline]() { pipeline.start(); },
                            [&pipeline]() {
                                pipeline.stop();
                                // pipeline.wait();
                                std::this_thread::sleep_for(std::chrono::seconds(10));
                            });
    std::list<std::shared_ptr<dai::ImgFrame>> origFramesBuffer;
    size_t maxBufferSize = 300;  // This has to be big enough to catch all the same timestamps.
                                 // But eats up RAM: maxBufferSize * 3 * 12MB
    int framesCount = 0;
    int framesWantedCount = 10;
    int origFramesCount = 0;
    while(true) {
        const auto inRgb = queueFrames->tryGet<dai::ImgFrame>();
        const auto inRgbOrig = queueFramesOrig->tryGet<dai::ImgFrame>();
        if(inRgbOrig) {
            std::cout << "Got origi[" << origFramesCount << "] of size " << inRgbOrig->getWidth() << "x" << inRgbOrig->getHeight()
                      << " timestamp: " << inRgbOrig->tsDevice.get().time_since_epoch().count() << "\n"
                      << std::flush;
            ++origFramesCount;
            if(origFramesBuffer.size() == maxBufferSize) {
                origFramesBuffer.pop_front();
                std::cout << "Poping origFramesBuffer\n" << std::flush;
            }
            origFramesBuffer.push_back(inRgbOrig);
            std::cout << "Adding origFramesBuffer size was: " << origFramesBuffer.size() << "\n" << std::flush;
            for(auto& compare : imgComparesOut) {
                if(compare.second == nullptr && compare.first->tsDevice.get() == inRgbOrig->tsDevice.get()) {
                    std::cout << "Found compare.second...\n" << std::flush;
                    compare.second = inRgbOrig;
                }
            }
        }
        if(inRgb) {
            std::cout << "Got frame[" << framesCount << "] of size " << std::get<0>(size) << "x" << std::get<1>(size)
                      << " timestamp: " << inRgb->tsDevice.get().time_since_epoch().count() << "\n"
                      << std::flush;
            ++framesCount;
            if(imgComparesOut.size() == framesWantedCount) {
                std::cout << "Ignore this frame\n" << std::flush;
            } else {
                REQUIRE(inRgb->getWidth() == std::get<0>(size));
                REQUIRE(inRgb->getHeight() == std::get<1>(size));
                std::shared_ptr<dai::ImgFrame> origFrame;
                for(const auto& frame : origFramesBuffer) {
                    if(frame->tsDevice.get() == inRgb->tsDevice.get()) {
                        origFrame = frame;
                    }
                }
                // origFrame can be nullptr here
                imgComparesOut.emplace_back(std::make_pair(inRgb, origFrame));
            }
        }
        if(imgComparesOut.size() == framesWantedCount) {
            if(std::find_if(imgComparesOut.begin(), imgComparesOut.end(), [](const auto& iter) { return iter.second == nullptr; }) == imgComparesOut.end()
               || (inRgbOrig && inRgbOrig->tsDevice.get() > imgComparesOut.back().first->tsDevice.get())) {
                std::cout << "Got all frames\n" << std::flush;
                break;
            }
        }
    }
    // TODO(jakgra) be more strict here and remove the 2 lines below once things work on rvc4
    imgComparesOut.remove_if([](const auto& iter) { return iter.second == nullptr; });
    REQUIRE(imgComparesOut.size() > framesWantedCount / 2);
    for(const auto& compare : imgComparesOut) {
        REQUIRE((compare.first && compare.second));
    }
}

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void compareImages(bool debugOn,
                   std::tuple<uint32_t, uint32_t> size,
                   dai::ImgResizeMode resizeMode,
                   const std::list<std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>>& imgCompares) {
    int count = 0;
    for(const auto& compare : imgCompares) {
        cv::Mat resizedDown;
        using E = dai::ImgResizeMode;
        switch(resizeMode) {
            case E::STRETCH:
                resize(compare.second->getFrame(),
                       resizedDown,
                       cv::Size(static_cast<int>(std::get<0>(size)), static_cast<int>(std::get<1>(size))),
                       cv::INTER_AREA);
                // cv::INTER_CUBIC); // has artifacts???
                break;
            case E::CROP:
                // TODO(jakgra) implement
                REQUIRE(false);
                break;
            case E::LETTERBOX:
                // TODO(jakgra) implement
                REQUIRE(false);
                break;
            default:
                REQUIRE(false);
                break;
        }
        REQUIRE((resizedDown.cols == compare.first->getWidth() && resizedDown.rows == compare.first->getHeight()));
        const double diffNorm = cv::norm(compare.first->getFrame(), resizedDown);
        const uint32_t area = compare.first->getWidth() * compare.first->getHeight();
        const double diffNormArea = diffNorm / area;
        std::cout << "FRAME " << count << " DIFF was: " << diffNormArea << "\n";
        double maxDiff = 0.013;
        if(debugOn && diffNormArea > maxDiff) {
            ImageComparator comparator;
            comparator.run(compare.first->getFrame(), resizedDown);
        }
        REQUIRE(diffNormArea < maxDiff);
        ++count;
    }
}

void testResolutionWithContent(bool debugOn, dai::ImgResizeMode resizeMode, std::optional<std::tuple<uint32_t, uint32_t>> wantedSize = std::nullopt) {
    std::list<std::pair<std::shared_ptr<dai::ImgFrame>, std::shared_ptr<dai::ImgFrame>>> imgCompares;
    std::tuple<uint32_t, uint32_t> size;
    getImages(imgCompares, size, resizeMode, wantedSize);
    compareImages(debugOn, size, resizeMode, imgCompares);
}

struct MultipleResHelper {
    std::tuple<uint32_t, uint32_t> size;
    std::shared_ptr<dai::MessageQueue> queue;
    uint32_t count{0};
};

// NOLINTNEXTLINE(readability-function-cognitive-complexity)
void testMultipleResolutions(const std::vector<std::tuple<uint32_t, uint32_t>>& wantedSizes) {
    const auto device = std::make_shared<dai::Device>(getDeviceInfo());
    dai::Pipeline pipeline(device);
    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_C);

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
    // const auto resolution = GENERATE(table<uint32_t, uint32_t>({{700, 700}}));
    testResolutionWithContent(isDebug(), dai::ImgResizeMode::STRETCH, resolution);
}

TEST_CASE("common_resolutions") {
    const auto resolution = GENERATE(table<uint32_t, uint32_t>({{1920, 1080}, {300, 300}, {640, 640}, {800, 600}, {640, 480}}));
    testResolutionWithContent(isDebug(), dai::ImgResizeMode::STRETCH, resolution);
}

TEST_CASE("random_resolutions") {
    (void)GENERATE(repeat(20, value(0)));
    testResolutionWithContent(isDebug(), dai::ImgResizeMode::STRETCH);
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
