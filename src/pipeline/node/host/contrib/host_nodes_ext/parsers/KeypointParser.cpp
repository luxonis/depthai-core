//
// Created by thwdpc on 7/25/25.
//

#include "parsers/KeypointParser.hpp"

#include <spdlog/spdlog.h>

#include "utility/ErrorMacros.hpp"

namespace dai::node {
void KeypointParser::buildImpl(const nn_archive::v1::Head& head, const nn_archive::v1::Model& model) {
    bool fallback = false;
    if(const auto layers = head.metadata.keypointsOutputs) {
        for(auto& layerName : *layers) {
            auto output = std::find_if(model.outputs.begin(), model.outputs.end(), [&](const auto& o) { return o.name == layerName; });
            DAI_CHECK_V(output != model.outputs.end(), "{}: keypoint output {} not found in model", getName(), layerName);
            keypointsOutputs.push_back(*output);
        }
    } else {
        spdlog::trace("KeypointParser(or subclass) did not receive keypoints_outputs, fallback to using all outputs");
        for(auto& output : model.outputs) {
            keypointsOutputs.push_back(output);
        };
        fallback = true;
    }

    const uint8_t ko_sz = keypointsOutputs.size();
    if(ko_sz < 1 || ko_sz > 3) {
        const std::string where = fallback ? "During fallback to use all outputs" : "Configured keypoints_outputs";
        throw std::runtime_error(fmt::format("{w}: size {sz} must satisfy 1 <= {sz} <= 3 ", fmt::arg("w", where), fmt::arg("sz", ko_sz)));
    }

    // take outputs size if it makes sense else default
    switch(head.metadata.extraParams.value("values_per_keypoint", ko_sz > 1 ? ko_sz : static_cast<uint8_t>(valuesPerKeypoint))) {
        case 2:
            valuesPerKeypoint = ValuesPerKeypoint::Two;
            break;
        case 3:
            valuesPerKeypoint = ValuesPerKeypoint::Three;
            break;
        default:
            DAI_CHECK_IN(false);
            break;
    }

    DAI_CHECK_V(ko_sz == 1 || ko_sz == static_cast<uint8_t>(valuesPerKeypoint),
                "Expected one output per keypoint dimension, or one output that contains all keypoints, got {} layers vs dimensionality {}.",
                ko_sz,
                static_cast<uint8_t>(valuesPerKeypoint));

    if(const auto n = head.metadata.nKeypoints) {
        nKeypoints = *n;
    } else {
        spdlog::warn("SimCCKeypointParser did not receive n_keypoints, defaulting to standard COCO 17. Populating this field is strongly encouraged");
    }

    keypointNames = head.metadata.extraParams.value("keypoint_names", keypointNames);
    skeletonEdges = head.metadata.extraParams.value("skeleton_edges", skeletonEdges);
}
void KeypointParser::run() {
    assert(false); //TODO KeypointParser::run
}

}  // namespace dai::node