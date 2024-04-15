#pragma once

#include <optional>
#include <string>

namespace dai {
namespace nn_archive {
namespace v1 {
enum class DataType : int;
}
}  // namespace nn_archive
}  // namespace dai

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Represents output stream of a model.
 *
 * @type name: str
 * @ivar name: Name of the output layer.
 * @type dtype: DataType
 * @ivar dtype: Data type of the output data (e.g., 'float32').
 */

/**
 * Represents output stream of a model.
 *
 * @type name: str
 * @ivar name: Name of the output layer.
 * @type dtype: DataType
 * @ivar dtype: Data type of the output data (e.g., 'float32').
 */
struct Output {
    /**
     * Data type of the output data (e.g., 'float32').
     */
    DataType dtype;
    /**
     * Name of the output layer.
     */
    std::string name;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
