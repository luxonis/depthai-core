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
    if((0 <= x && x <= 255) || (1275 < x && x <= 1529)) {
        R = 255;
    } else if(255 < x && x <= 510) {
        R = 510 - x;
    } else if(510 < x && x <= 1020) {
        R = 0;
    } else if(1020 < x && x <= 1275) {
        R = x - 1020;
    }

    if(0 <= x && x <= 255) {
        G = x;
    } else if(255 < x && x <= 765) {
        G = 255;
    } else if(765 < x && x <= 1020) {
        G = 1020 - x;
    } else if(1020 < x && x <= 1529) {
        G = 0;
    }

    if(0 <= x && x <= 510) {
        B = 0;
    } else if(510 < x && x <= 765) {
        B = x - 510;
    } else if(765 < x && x <= 1275) {
        B = 255;
    } else if(1275 < x && x <= 1529) {
        B = 1529 - x;
    }
    return std::make_tuple(R, G, B);
}

namespace dai {
namespace node {

void DepthEncoder::build() {}

DepthEncoder::DepthEncoder() : NodeCRTP<DeviceNode, DepthEncoder, DepthEncoderProperties>() {}

DepthEncoder::DepthEncoder(std::unique_ptr<Properties> props) : NodeCRTP<DeviceNode, DepthEncoder, DepthEncoderProperties>(std::move(props)) {}

DepthEncoder::Properties& DepthEncoder::getProperties() {
    return properties;
}

void DepthEncoder::setLut(std::vector<uint8_t> lutR, std::vector<uint8_t> lutG, std::vector<uint8_t> lutB) {
    properties.lutR = std::move(lutR);
    properties.lutG = std::move(lutG);
    properties.lutB = std::move(lutB);
}

void DepthEncoder::setHueEncoding(HueEncodingType hueEncodingType, int32_t minDepth, int32_t maxDepth) {
    properties.useHueEncoding = true;
    properties.hueEncodingType = hueEncodingType;
    properties.minDepth = minDepth;
    properties.maxDepth = maxDepth;
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
