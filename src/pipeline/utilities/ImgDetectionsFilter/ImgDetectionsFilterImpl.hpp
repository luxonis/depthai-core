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
// One entry per linked key, owned by the processing thread and reused between rounds.
struct MaskRemapCache {
    std::array<std::vector<std::uint8_t>, 2> transformations;
    std::array<std::size_t, 4> dimensions{};
    std::vector<std::size_t> sourceIndices;
};

// Inputs must be in key order; each vector entry represents a different linked key. Invalid message data throws std::runtime_error.
std::shared_ptr<ImgDetections> filterDetectionRound(const std::vector<std::shared_ptr<ImgDetections>>& messages,
                                                    const ImgDetectionsFilterConfig& config,
                                                    const std::optional<ImgTransformation>& reference,
                                                    const std::vector<std::shared_ptr<ImgFrame>>& sourceMasks = {},
                                                    std::vector<MaskRemapCache>* maskRemaps = nullptr);
}  // namespace impl
}  // namespace dai
