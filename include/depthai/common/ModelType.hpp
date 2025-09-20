#pragma once

#include <filesystem>

#include "depthai/utility/export.hpp"

namespace dai {
namespace model {

/**
 * @brief Neural network model type
 */
enum class ModelType { BLOB, SUPERBLOB, DLC, NNARCHIVE, OTHER };

/**
 * @brief Read model type from model path
 *
 * @param modelPath Path to model
 * @return ModelType
 */
ModelType DEPTHAI_API readModelType(const std::filesystem::path& modelPath);

}  // namespace model
}  // namespace dai