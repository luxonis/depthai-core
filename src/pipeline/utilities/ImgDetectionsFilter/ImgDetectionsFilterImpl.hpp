#pragma once
#include <memory>
#include <optional>
#include <vector>

#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsFilterConfig.hpp"

namespace dai {
namespace impl {
// Inputs must be in key order; each vector entry represents a different linked key.
std::shared_ptr<ImgDetections> filterDetectionRound(const std::vector<std::shared_ptr<ImgDetections>>& messages,
                                                    const ImgDetectionsFilterConfig& config,
                                                    const std::optional<ImgTransformation>& reference);
}  // namespace impl
}  // namespace dai
