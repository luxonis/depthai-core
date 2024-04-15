#pragma once

#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Data type of the input data (e.g., 'float32').
 *
 * Represents all existing data types used in i/o streams of the model.
 *
 * Data type of the output data (e.g., 'float32').
 */

/**
 * Data type of the input data (e.g., 'float32').
 *
 * Represents all existing data types used in i/o streams of the model.
 *
 * Data type of the output data (e.g., 'float32').
 */
enum class DataType : int { FLOAT16, FLOAT32, INT8, NV12, UINT8 };
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
