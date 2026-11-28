#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace impl {
struct MaskRemapCache {  // One per linked key; owned by the processing thread, reused between rounds
    std::array<std::vector<std::uint8_t>, 2> transformations;
    std::array<std::size_t, 4> dimensions{};
    std::vector<std::size_t> sourceIndices;
};

std::shared_ptr<ImgDetections> filterDetectionRound(const std::vector<std::shared_ptr<ImgDetections>>& messages,  // Inputs in key order; throws runtime_error
                                                    const ImgDetectionsFilterConfig& config,
                                                    const std::optional<ImgTransformation>& reference,
                                                    const std::vector<std::shared_ptr<ImgFrame>>& sourceMasks = {},
                                                    std::vector<MaskRemapCache>* maskRemaps = nullptr);
}  // namespace impl
}  // namespace dai
