#pragma once

#include <optional>
#include <string>
#include <vector>

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
     * List of letters describing the output layout (e.g. 'NC').
     */
    std::optional<std::string> layout;
    /**
     * Name of the output layer.
     */
    std::string name;
    /**
     * Shape of the output as a list of integers (e.g. [1, 1000]).
     */
    std::optional<std::vector<int64_t>> shape;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
