#include "depthai/pipeline/node/DepthEncoder.hpp"

// standard
#include <fstream>

#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

static constexpr auto maxHueValue = 1529;
std::tuple<int, int, int> toRgbHue(int x) {
    if(x < 0 || x > 1529) throw std::runtime_error("Invalid input value for hue LUT conversion. Must be between 0 and 1529 but got " + std::to_string(x));
    int R, G, B = 0;
    if ((0 <= x && x <= 255) || (1275 < x && x <= 1529)) {
        R = 255;
    } else if (255 < x && x <= 510) {
        R = 510 - x;
    } else if (510 < x && x <= 1020) {
        R = 0;
    } else if (1020 < x && x <= 1275) {
        R = x - 1020;
    }

    if (0 <= x && x <= 255) {
        G = x;
    } else if (255 < x && x <= 765) {
        G = 255;
    } else if (765 < x && x <= 1020) {
        G = 1020 - x;
    } else if (1020 < x && x <= 1529) {
        G = 0;
    }

    if (0 <= x && x <= 510) {
        B = 0;
    } else if (510 < x && x <= 765) {
        B = x - 510;
    } else if (765 < x && x <= 1275) {
        B = 255;
    } else if (1275 < x && x <= 1529) {
        B = 1529 - x;
    }
    return std::make_tuple(R, G, B);
}

namespace dai {
namespace node {

void DepthEncoder::build() {

}

DepthEncoder::DepthEncoder()
    : NodeCRTP<DeviceNode, DepthEncoder, DepthEncoderProperties>() {}

DepthEncoder::DepthEncoder(std::unique_ptr<Properties> props)
    : NodeCRTP<DeviceNode, DepthEncoder, DepthEncoderProperties>(std::move(props)) {}

DepthEncoder::Properties& DepthEncoder::getProperties() {
    return properties;
}


void DepthEncoder::setLut(std::vector<uint8_t> lutR, std::vector<uint8_t> lutG, std::vector<uint8_t> lutB) {
    properties.lutR = std::move(lutR);
    properties.lutG = std::move(lutG);
    properties.lutB = std::move(lutB);
}


void DepthEncoder::setHueLutGeneric(uint16_t minDepthIn,
                                    uint16_t maxDepthIn,
                                    float bufferAmount,
                                    const std::function<uint16_t(uint16_t, uint16_t)>& getMinDisparity,
                                    const std::function<uint16_t(uint16_t, uint16_t, uint16_t)>& getMaxDisparity,
                                    const std::function<uint16_t(uint16_t, uint16_t, uint16_t, uint16_t)>& getHueValueFromDisparity) {
    // First check the input for validity
    if(minDepthIn >= maxDepthIn) throw std::runtime_error("Invalid input for setHueLut: minIn must be smaller than maxIn");
    if(bufferAmount < 0 || bufferAmount > 0.5f) throw std::runtime_error("Invalid input for setHueLut: bufferAmount must be between 0 and 0.5");
    constexpr uint16_t LUT_SIZE = 96 * 32 * 2;  // Must match the maximum size on FW
    std::vector<uint8_t> lutR(LUT_SIZE);
    std::vector<uint8_t> lutG(LUT_SIZE);
    std::vector<uint8_t> lutB(LUT_SIZE);

    uint16_t minInDisparity = getMinDisparity(minDepthIn, maxDepthIn);
    uint16_t maxInDisparity = getMaxDisparity(minDepthIn, maxDepthIn, LUT_SIZE - 1);

    for(int i = 0; i < LUT_SIZE; i++) {
        // First handle the case when the input is outside of the range
        if(i < minInDisparity || i > maxInDisparity) {
            lutR[i] = 0;
            lutG[i] = 0;
            lutB[i] = 0;
            continue;
        }
        // Calculate the hue value
        auto hueIn = getHueValueFromDisparity(i, minDepthIn, maxDepthIn, maxHueValue);

        // Handle the buffering to avoid minimum and maximum colors being too similar
        auto hueInBuffered = static_cast<uint16_t>(std::round(static_cast<float>(hueIn) * (1.0f - (2 * bufferAmount))) + (maxHueValue * bufferAmount));
        auto color = toRgbHue(hueInBuffered);
        lutR[i] = std::get<0>(color);
        lutG[i] = std::get<1>(color);
        lutB[i] = std::get<2>(color);
    }
    // Override that in all cases the invalid depth is mapped to black
    lutR[0] = 0;
    lutG[0] = 0;
    lutB[0] = 0;
    setLut(lutR, lutG, lutB);
}

void DepthEncoder::setHueLutDisparity(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount) {
    // Check scale for validity
    if(scale <= 0) throw std::runtime_error("Invalid input for setHueLutDepth: scale must be positive");

    auto getMinDisparity = [scale](uint16_t minDepth, uint16_t maxDepth) -> uint16_t {
        (void)minDepth;
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(maxDepth)));
    };
    auto getMaxDisparity = [scale](uint16_t minDepth, uint16_t maxDepth, uint16_t valueForMaxDepth) -> uint16_t {
        (void)maxDepth;
        if(minDepth == 0) {
            return valueForMaxDepth;
        }
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(minDepth)));
    };
    auto getHueValueFromDisparity = [scale](uint16_t disparity, uint16_t minDepth, uint16_t maxDepth, int maxHueValue) -> int {
        (void)minDepth;
        (void)maxDepth;
        uint16_t maxDisparity = 0;
        if(minDepth == 0) {
            throw std::runtime_error("Invalid input for setHueLutDepth: minDepth must not be zero");
        } else {
            maxDisparity = static_cast<uint16_t>(std::round(scale / minDepth));
        }
        return static_cast<int>(std::round(static_cast<float>(disparity) * static_cast<float>(maxHueValue) / static_cast<float>(maxDisparity)));
    };

    setHueLutGeneric(minInDepth, maxInDepth, bufferAmount, getMinDisparity, getMaxDisparity, getHueValueFromDisparity);
};

void DepthEncoder::setHueLutDepth(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount) {
    // Check scale for validity
    if(scale <= 0) throw std::runtime_error("Invalid input for setHueLutDepth: scale must be positive");

    auto getMinDisparity = [scale](uint16_t minDepth, uint16_t maxDepth) -> uint16_t {
        (void)minDepth;
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(maxDepth)));
    };
    auto getMaxDisparity = [scale](uint16_t minDepth, uint16_t maxDepth, uint16_t valueForMaxDepth) -> uint16_t {
        (void)maxDepth;
        if(minDepth == 0) {
            return valueForMaxDepth;
        }
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(minDepth)));
    };
    auto getHueValueFromDisparity = [scale](uint16_t disparity, uint16_t minDepth, uint16_t maxDepth, int maxHueValue) -> int {
        float depth = scale / static_cast<float>(disparity);
        return static_cast<int>(std::round(static_cast<float>((depth - static_cast<float>(minDepth)) * static_cast<float>(maxHueValue))
                                           / static_cast<float>((maxDepth - minDepth))));
    };

    setHueLutGeneric(minInDepth, maxInDepth, bufferAmount, getMinDisparity, getMaxDisparity, getHueValueFromDisparity);
}

std::tuple<double, double> DepthEncoder::setHueLutDepthNormalized(uint16_t minInDepth, uint16_t maxInDepth, float scale, float bufferAmount) {
    // Check scale for validity
    if(scale <= 0) throw std::runtime_error("Invalid input for setHueLutDepth: scale must be positive");

    auto getMinDisparity = [scale](uint16_t minDepth, uint16_t maxDepth) -> uint16_t {
        (void)minDepth;
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(maxDepth)));
    };
    auto getMaxDisparity = [scale](uint16_t minDepth, uint16_t maxDepth, uint16_t valueForMaxDepth) -> uint16_t {
        (void)maxDepth;
        if(minDepth == 0) {
            return valueForMaxDepth;
        }
        return static_cast<std::uint16_t>(std::round(scale / static_cast<float>(minDepth)));
    };

    // Push the depth through a logarithmic function to make the expected relative error constant
    // depth = a * ln(depth) + d
    // To get a and d we solve the system of equations:
    // a * ln(minDepth) + d = minHue
    // a * ln(maxDepth) + d = maxHue
    // Then we solve for a and d
    // a = maxHue - minHue / (ln(maxDepth) - ln(minDepth))
    // d = ln(maxDepth) * minHue - maxHue * ln(minDepth) / (ln(maxDepth) - ln(minDepth))
    // Considering minHue = 0 and maxHue = maxHueValue
    auto a = static_cast<double>(maxHueValue) / (std::log(maxInDepth) - std::log(minInDepth));
    auto d = - maxHueValue * std::log(minInDepth) / (std::log(maxInDepth) - std::log(minInDepth));
    // std::cout << "a: " << a << " d: " << d << std::endl;
    auto getHueValueFromDisparity = [scale, a, d](uint16_t disparity, uint16_t minDepth, uint16_t maxDepth, int maxHueValue) -> int {
        (void)maxHueValue;
        (void)minDepth;
        (void)maxDepth;
        double depth = scale / disparity;
        return static_cast<int>(std::round(a * std::log(depth) + d));
    };

    setHueLutGeneric(minInDepth, maxInDepth, bufferAmount, getMinDisparity, getMaxDisparity, getHueValueFromDisparity);
    return std::make_tuple(a, d);
};

void DepthEncoder::setOutputType(RawImgFrame::Type outputType) {
    properties.outputType = outputType;
}

void DepthEncoder::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void DepthEncoder::setNumShaves(int numShaves) {
    properties.numShaves = numShaves;
}

}  // namespace node
}  // namespace dai
