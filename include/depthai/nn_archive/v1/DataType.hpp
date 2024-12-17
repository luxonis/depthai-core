#pragma once

#include <optional>

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Data type of the input data (e.g., 'float32').
 *
 * Represents all existing data types used in i/o streams of the
 * model.
 *
 * Precision of the model weights.
 *
 * Data type of the output data (e.g., 'float32').
 */

/**
 * Data type of the input data (e.g., 'float32').
 *
 * Represents all existing data types used in i/o streams of the
 * model.
 *
 * Precision of the model weights.
 *
 * Data type of the output data (e.g., 'float32').
 */
enum class DataType : int { BOOLEAN, FLOAT16, FLOAT32, FLOAT64, INT16, INT32, INT4, INT64, INT8, STRING, UINT16, UINT32, UINT4, UINT64, UINT8 };
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
