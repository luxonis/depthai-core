#include <algorithm>
#include <chrono>
#include <iostream>
#include <iterator>
#include <limits>
#include <stdexcept>

// Libraries
#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <tuple>

// Includes common necessary includes for development using depthai library
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraFeatures.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/properties/ColorCameraProperties.hpp"

/*
The code is the same as for Tiny-yolo-V3, the only difference is the blob file.
The blob was compiled following this tutorial: https://github.com/TNTWEN/OpenVINO-YOLOV4
*/

// TODO fix so all resolutions work
struct Res {
    int32_t width;
    int32_t height;
};
static const std::vector<Res> notWorkingResolutions = {
    {.width = 1352, .height = 1012}  // dai::ColorCameraProperties::SensorResolution::THE_1352X1012
};

dai::ColorCameraProperties::SensorResolution resolutionFromSensorConfig(const dai::CameraSensorConfig& config) {
    const auto width = config.width;
    const auto height = config.height;
    using R = dai::ColorCameraProperties::SensorResolution;
    if(width == 1920 && height == 1080) {
        return R::THE_1080_P;
    } else if(width == 3840 && height == 2160) {
        return R::THE_4_K;
    } else if(width == 4056 && height == 3040) {
        return R::THE_12_MP;
    } else if(width == 4208 && height == 3120) {
        return R::THE_13_MP;
    } else if(width == 1280 && height == 720) {
        return R::THE_720_P;
    } else if(width == 1280 && height == 800) {
        return R::THE_800_P;
    } else if(width == 1920 && height == 1200) {
        return R::THE_1200_P;
    } else if(width == 2592 && height == 1944) {
        return R::THE_5_MP;
    } else if(width == 4000 && height == 3000) {
        return R::THE_4000X3000;
    } else if(width == 5312 && height == 6000) {
        return R::THE_5312X6000;
    } else if(width == 8000 && height == 6000) {
        return R::THE_48_MP;
    } else if(width == 240 && height == 180) {
        return R::THE_240X180;
    } else if(width == 1280 && height == 962) {
        return R::THE_1280X962;
    } else if(width == 2000 && height == 1500) {
        return R::THE_2000X1500;
    } else if(width == 2028 && height == 1520) {
        return R::THE_2028X1520;
    } else if(width == 2104 && height == 1560) {
        return R::THE_2104X1560;
    } else if(width == 1440 && height == 1080) {
        return R::THE_1440X1080;
    } else if(width == 1352 && height == 1012) {
        return R::THE_1352X1012;
    } else if(width == 2024 && height == 1520) {
        return R::THE_2024X1520;
    } else {
        throw std::runtime_error("Unknown resolution " + std::to_string(width) + "x" + std::to_string(height) + " requested");
    }
}

std::vector<dai::CameraSensorConfig> getColorCameraConfigs(dai::Device& device) {
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

dai::CameraSensorConfig getClosestCameraConfig(const std::vector<dai::CameraSensorConfig>& colorCameraModes, int64_t width, int64_t height) {
    int64_t minAdditionalPixels = -1;
    ssize_t foundIndex = -1;
    ssize_t index = 0;
    for(const auto& mode : colorCameraModes) {
        if(mode.width >= width && mode.height >= height) {
            if(std::find_if(notWorkingResolutions.begin(),
                            notWorkingResolutions.end(),
                            [&mode = std::as_const(mode)](const auto& itr) { return itr.width == mode.width && itr.height == mode.height; })
               != notWorkingResolutions.end()) {
                std::cout << "Warning: ignoring possible best resolution " << mode.width << "x" << mode.height << " because of possible firmware bugs"
                          << std::endl;
            } else {
                int64_t additionalPixels = (mode.width - width) * mode.height + (mode.height - height) * mode.width;
                if(minAdditionalPixels == -1 || additionalPixels < minAdditionalPixels) {
                    foundIndex = index;
                    minAdditionalPixels = additionalPixels;
                }
            }
        }
        ++index;
    }
    if(minAdditionalPixels == -1 || foundIndex == -1) {
        throw std::runtime_error("This camera can't provide the wanted resolution " + std::to_string(width) + "x" + std::to_string(height));
    }
    return colorCameraModes[foundIndex];
}

std::set<std::tuple<double, int, int>> validIspScales() {
    std::set<std::tuple<double, int, int>> result;
    for(int numerator = 1; numerator < 16 + 1; ++numerator) {
        for(int denominator = 1; denominator < 63 + 1; ++denominator) {
            // Chroma needs 2x extra downscaling
            if(denominator < 32 || numerator % 2 == 0) {
                // Only if irreducible
                if(std::gcd(numerator, denominator) == 1) {
                    result.insert(std::make_tuple((double)numerator / (double)denominator, numerator, denominator));
                }
            }
        }
    }
    return result;
}

std::tuple<int, int> findClosestIspScale(int width, int height, const dai::CameraSensorConfig& mode) {
    const static auto validScales = validIspScales();
    const auto useWidth = (double)width / (double)mode.width > (double)height / (double)mode.height;
    int numerator = useWidth ? width : height;
    int denominator = useWidth ? mode.width : mode.height;
    const auto div = std::gcd(numerator, denominator);
    numerator = numerator / div;
    denominator = denominator / div;
    const auto foundScale = std::find_if(validScales.begin(), validScales.end(), [numerator, denominator](const auto& itr) {
        return std::get<1>(itr) == numerator && std::get<2>(itr) == denominator;
    });
    if(foundScale != validScales.end()) {
        return std::make_tuple(std::get<1>(*foundScale), std::get<2>(*foundScale));
    }
    const double wantedScale = (double)numerator / (double)denominator;
    double bestDiff = std::numeric_limits<double>::max();
    auto bestScale = std::make_tuple<int, int>(-1, -1);
    for(const auto& validScale : validScales) {
        const auto scale = std::get<0>(validScale);
        if(scale >= wantedScale) {
            const auto diff = scale - wantedScale;
            if(diff < bestDiff) {
                bestDiff = diff;
                bestScale = std::make_tuple(std::get<1>(validScale), std::get<2>(validScale));
            }
        }
    }
    if(std::get<0>(bestScale) == -1) {
        throw std::runtime_error("Couldn't find correct scale");
    }
    return bestScale;
}

void testResolution(std::optional<std::tuple<int, int>> wantedSize = std::nullopt) {
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();

    camRgb->setInterleaved(false);                                       // NOLINT
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);  // NOLINT
    camRgb->setFps(40);

    const auto device = pipeline.getDefaultDevice();
    if(!device) {
        throw std::runtime_error("The pipeline was without device");
    }
    const auto& colorCameraConfigs = getColorCameraConfigs(*device);
    int width;
    int height;
    if(wantedSize) {
        width = std::get<0>(*wantedSize);
        height = std::get<1>(*wantedSize);
    } else {
        const auto maxMode = std::max_element(colorCameraConfigs.begin(), colorCameraConfigs.end(), [](const auto& first, const auto& second) {
            return first.width * first.height < second.width * second.height;
        });
        if(maxMode == colorCameraConfigs.end()) {
            throw std::runtime_error("No color camera sensor configs found");
        }
        std::srand(std::time(nullptr));
        width = std::abs(std::rand()) % (*maxMode).width;
        height = std::abs(std::rand()) % (*maxMode).height;
        // INFO TAKEN FROM FW: width & height must be greater or equal to 32 pixels (empirical tests)
        width = width < 32 ? 32 : width;
        height = height < 32 ? 32 : height;
    }
    std::cout << "TESTING RESOLUTION: " << width << "x" << height << std::endl;
    const auto& mode = getClosestCameraConfig(colorCameraConfigs, width, height);
    std::cout << "FOUND CLOSEST RESOLUTION: " << mode.width << "x" << mode.height << std::endl;
    camRgb->setResolution(resolutionFromSensorConfig(mode));
    const auto ispScale = findClosestIspScale(width, height, mode);
    std::cout << "USING ISP SCALE " << std::get<0>(ispScale) << "/" << std::get<1>(ispScale) << std::endl;
    camRgb->setIspScale(ispScale);
    std::cout << "SETTING PREVIEW SIZE TO: " << width << "x" << height << std::endl;
    camRgb->setPreviewSize(static_cast<int>(width), static_cast<int>(height));

    const auto queueFrames = camRgb->preview.getQueue();

    const auto& qRgb = queueFrames;
    pipeline.start();

    for(int counter = 0; counter < 10; ++counter) {
        const auto inRgb = qRgb->get<dai::ImgFrame>();
        if(inRgb) {
            REQUIRE(inRgb->getWidth() == width);
            REQUIRE(inRgb->getHeight() == height);
        }
        counter++;
    }
    pipeline.stop();
    pipeline.wait();
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

TEST_CASE("prev_broken_resolutions") {
    const auto resolution = GENERATE(table<int, int>({{3860, 2587}, {3951, 1576}, {909, 909}, {444, 888}}));
    testResolution(resolution);
}

TEST_CASE("common_resolutions") {
    const auto resolution = GENERATE(table<int, int>({{1920, 1080}, {300, 300}, {640, 640}, {800, 600}}));
    testResolution(resolution);
}

TEST_CASE("random_resolutions") {
    (void)GENERATE(repeat(20, value(0)));
    testResolution();
}
