//
// Created by thwdpc on 7/25/25.
//

#pragma once
#include "BaseParser.hpp"

namespace dai::node {

enum class ValuesPerKeypoint: uint8_t {
    Two = 2,
    Three = 3
};

class KeypointParser : virtual public CustomParser<KeypointParser> {
public:
    constexpr static const char* NAME = "KeypointParser";

protected:
    void buildImpl(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) override;
    void run() override;

    std::vector<nn_archive::v1::Output> keypointsOutputs{};
    uint16_t nKeypoints = 17;
    // dimensionality: 2D or 3D
    ValuesPerKeypoint valuesPerKeypoint = ValuesPerKeypoint::Two;
    std::vector<std::string> keypointNames{};
    std::vector<std::pair<uint8_t, uint8_t>> skeletonEdges{};
};
}