#pragma once

#include <optional>
#include <string>
#include <vector>

#include "PreprocessingBlock.hpp"

namespace dai {
namespace nn_archive {
namespace v1 {
enum class DataType : int;
enum class InputType : int;
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai

namespace dai {
namespace nn_archive {
namespace v1 {
/**
 * Represents input stream of a model.
 *
 * @type name: str
 * @ivar name: Name of the input layer.
 *
 * @type dtype: DataType
 * @ivar dtype: Data type of the input data (e.g., 'float32').
 *
 * @type input_type: InputType
 * @ivar input_type: Type of input data (e.g., 'image').
 *
 * @type shape: list
 * @ivar shape: Shape of the input data as a list of integers (e.g. [H,W], [H,W,C],
 * [N,H,W,C], ...).
 *
 * @type layout: str
 * @ivar layout: Lettercode interpretation of the input data dimensions (e.g., 'NCHW').
 *
 * @type preprocessing: PreprocessingBlock
 * @ivar preprocessing: Preprocessing steps applied to the input data.
 */

/**
 * Represents input stream of a model.
 *
 * @type name: str
 * @ivar name: Name of the input layer.
 *
 * @type dtype: DataType
 * @ivar dtype: Data type of the input data (e.g., 'float32').
 *
 * @type input_type: InputType
 * @ivar input_type: Type of input data (e.g., 'image').
 *
 * @type shape: list
 * @ivar shape: Shape of the input data as a list of integers (e.g. [H,W], [H,W,C],
 * [N,H,W,C], ...).
 *
 * @type layout: str
 * @ivar layout: Lettercode interpretation of the input data dimensions (e.g., 'NCHW').
 *
 * @type preprocessing: PreprocessingBlock
 * @ivar preprocessing: Preprocessing steps applied to the input data.
 */
struct Input {
    /**
     * Data type of the input data (e.g., 'float32').
     */
    DataType dtype;
    /**
     * Type of input data (e.g., 'image').
     */
    InputType inputType;
    /**
     * Lettercode interpretation of the input layout (e.g., 'NCHW').
     */
    std::optional<std::string> layout;
    /**
     * Name of the input layer.
     */
    std::string name;
    /**
     * Preprocessing steps applied to the input data.
     */
    PreprocessingBlock preprocessing;
    /**
     * Shape of the input data as a list of integers (e.g. [H,W], [H,W,C], [N,H,W,C], ...).
     */
    std::vector<int64_t> shape;
};
}  // namespace v1
}  // namespace nn_archive
}  // namespace dai
