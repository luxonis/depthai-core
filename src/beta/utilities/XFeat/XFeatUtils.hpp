#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "depthai/nn_archive/v1/Input.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "depthai/pipeline/datatype/TrackedFeatures.hpp"

namespace dai {
namespace beta {
namespace utilities {
namespace XFeatUtils {

struct XFeatTensor {
    std::vector<float> values;
    std::vector<std::size_t> dims;  // NCHW order
};

struct XFeatTensors {
    XFeatTensor feats;
    XFeatTensor keypoints;
    XFeatTensor heatmaps;
};

struct XFeatResult {
    std::vector<std::array<float, 2>> keypoints;  // (x, y) in original image pixels, descending score order
    std::vector<float> scores;
    std::vector<double> descriptors;  // L2-normalized, row-major (keypoints.size() x descriptorSize); float64 like the source
    std::size_t descriptorSize = 0;
};

struct MatchedPoints {
    std::vector<std::array<float, 2>> referencePoints;
    std::vector<std::array<float, 2>> targetPoints;
};

XFeatTensors extractXFeatTensors(dai::NNData& nnData, const std::string& featsName, const std::string& keypointsName, const std::string& heatmapsName);

std::optional<XFeatResult> detectAndCompute(const XFeatTensor& feats,  // Port of the source detect_and_compute; nullopt when no candidate is above 0.05
                                            const XFeatTensor& keypoints,
                                            const XFeatTensor& heatmaps,
                                            double resizeRateW,
                                            double resizeRateH,
                                            std::uint32_t inputWidth,
                                            std::uint32_t inputHeight,
                                            int topK);

MatchedPoints matchResults(const XFeatResult& reference, const XFeatResult& target, double minCossim = -1.0);  // _match_mkpts port; minCossim <= 0: no filter

std::shared_ptr<dai::TrackedFeatures> createTrackedFeaturesMessage(const MatchedPoints& matchedPoints);  // Match i: id i, reference age 0, then target age 1

std::optional<std::pair<std::uint32_t, std::uint32_t>> readSizeParam(const nlohmann::json& extraParams, const char* key);

std::optional<int> readMaxKeypointsParam(const nlohmann::json& extraParams);

std::pair<std::uint32_t, std::uint32_t> getModelInputSize(const std::vector<dai::nn_archive::v1::Input>& inputs, const char* parserName);

}  // namespace XFeatUtils
}  // namespace utilities
}  // namespace beta
}  // namespace dai
