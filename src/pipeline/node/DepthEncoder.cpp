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

void DepthEncoder::setHueLut(uint16_t minIn, uint16_t maxIn, float scaleFactor, float bufferAmount) {
    // First check the input for validity
    if(minIn >= maxIn) throw std::runtime_error("Invalid input for setHueLut: minIn must be smaller than maxIn");
    if(scaleFactor <= 0) throw std::runtime_error("Invalid input for setHueLut: scaleFactor must be positive");
    if(bufferAmount < 0 || bufferAmount > 1) throw std::runtime_error("Invalid input for setHueLut: bufferAmount must be between 0 and 1");
    constexpr uint16_t LUT_SIZE = 96 * 32 * 2; // Must match the maximum size on FW
    std::vector<uint8_t> lutR(LUT_SIZE);
    std::vector<uint8_t> lutG(LUT_SIZE);
    std::vector<uint8_t> lutB(LUT_SIZE);
    uint16_t minInDisparity = scaleFactor / maxIn;  // Transform depth to disparity
    uint16_t maxInDisparity;
    if(minIn == 0) {
        maxInDisparity = LUT_SIZE - 1;
    } else {
        maxInDisparity = scaleFactor / minIn;  // Transform depth to disparity
    }
    for(int i = 0; i < LUT_SIZE; i++) {
        // First handle the case when the input is outside of the range
        if(i < minInDisparity || i > maxInDisparity) {
            lutR[i] = 0;
            lutG[i] = 0;
            lutB[i] = 0;
            continue;
        }
        // Calculate the hue value
        // Convert disparity to depth for the hue calculation
        float depth = scaleFactor / i;
        int hueIn = std::round(static_cast<float>((depth - minIn) * 1529) / (maxIn - minIn));
        // Handle the buffering to avoid minimum and maximum colors being too similar
        int hueInBuffered = (hueIn * (1.0f - (2 * bufferAmount))) + (maxHueValue * bufferAmount);
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
