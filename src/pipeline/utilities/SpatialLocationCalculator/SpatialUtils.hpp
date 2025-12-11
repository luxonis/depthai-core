#pragma once
#include <spdlog/async_logger.h>

#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/datatype/SpatialLocationCalculatorConfig.hpp>
#include <depthai/pipeline/datatype/SpatialLocationCalculatorData.hpp>

#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"

namespace dai {
namespace utilities {
namespace SpatialUtils {

void computeSpatialData(std::shared_ptr<dai::ImgFrame> depthFrame,
                        const std::vector<dai::SpatialLocationCalculatorConfigData>& configDataVec,
                        std::vector<dai::SpatialLocations>& spatialLocations,
                        std::shared_ptr<spdlog::async_logger> logger);

void computeSpatialDetections(std::shared_ptr<dai::ImgFrame> depthFrame,
                              const std::shared_ptr<SpatialLocationCalculatorConfig> config,
                              dai::ImgDetections& imgDetections,
                              dai::SpatialImgDetections& spatialDetections,
                              std::shared_ptr<spdlog::async_logger> logger);
}  // namespace SpatialUtils
}  // namespace utilities
}  // namespace dai