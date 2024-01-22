#include "depthai/pipeline/node/DepthEncoder.hpp"

// standard
#include <fstream>

#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

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

void DepthEncoder::setOutputType(RawImgFrame::Type outputType) {
    properties.outputType = outputType;
}

void DepthEncoder::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

}  // namespace node
}  // namespace dai
