#pragma once
#include <spdlog/async_logger.h>

#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp>
#include <depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp>

#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {
namespace utilities {
namespace SpatialUtils {

struct DepthStats {
    std::uint32_t sum = 0;
    std::uint32_t counter = 0;
    std::uint16_t max = std::numeric_limits<std::uint16_t>::min();
    std::uint16_t min = std::numeric_limits<std::uint16_t>::max();
    std::uint32_t lowerThreshold = 0;
    std::uint32_t upperThreshold = std::numeric_limits<std::uint16_t>::max();
    std::vector<uint16_t> validPixels;

    DepthStats(std::uint32_t lowerThreshold, std::uint32_t upperThreshold, std::uint32_t maxNumPixels);

    void addPixel(uint16_t px);

    float calculateDepth(SpatialLocationCalculatorAlgorithm algo);
};

void computeSpatialData(std::shared_ptr<dai::ImgFrame> depthFrame,
                        const std::vector<dai::SpatialLocationCalculatorConfigData>& configDataVec,
                        std::vector<dai::SpatialLocations>& spatialLocations,
                        std::shared_ptr<spdlog::async_logger> logger);

void computeSpatialDetections(const dai::ImgFrame& depthFrame,
                              const SpatialLocationCalculatorConfig& config,
                              const dai::ImgDetections& imgDetections,
                              dai::SpatialImgDetections& spatialDetections,
                              std::shared_ptr<spdlog::async_logger> logger);
}  // namespace SpatialUtils
}  // namespace utilities
}  // namespace dai