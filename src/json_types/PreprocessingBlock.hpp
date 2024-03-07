//  To parse this JSON data, first install
//
//      json.hpp  https://github.com/nlohmann/json
//
//  Then include this file, and then do
//
//     PreprocessingBlock.hpp data = nlohmann::json::parse(jsonString);

#pragma once

#include <optional>
#include <nlohmann/json.hpp>
#include "helper.hpp"

namespace dai {
namespace json_types {
    /**
     * Preprocessing steps applied to the input data.
     *
     * Represents preprocessing operations applied to the input data.
     *
     * @type mean: list
     * @ivar mean: Mean values in channel order. Typically, this is BGR order.
     * @type scale: list
     * @ivar scale: Standardization values in channel order. Typically, this is BGR order.
     * @type reverse_channels: bool
     * @ivar reverse_channels: If True, color channels are reversed (e.g. BGR to RGB or
     * vice versa).
     * @type interleaved_to_planar: bool
     * @ivar interleaved_to_planar: If True, format is changed from interleaved to planar.
     */

    using nlohmann::json;

    /**
     * Preprocessing steps applied to the input data.
     *
     * Represents preprocessing operations applied to the input data.
     *
     * @type mean: list
     * @ivar mean: Mean values in channel order. Typically, this is BGR order.
     * @type scale: list
     * @ivar scale: Standardization values in channel order. Typically, this is BGR order.
     * @type reverse_channels: bool
     * @ivar reverse_channels: If True, color channels are reversed (e.g. BGR to RGB or
     * vice versa).
     * @type interleaved_to_planar: bool
     * @ivar interleaved_to_planar: If True, format is changed from interleaved to planar.
     */
    struct PreprocessingBlock {
        /**
         * If True, format is changed from interleaved to planar.
         */
        std::optional<bool> interleavedToPlanar;
        /**
         * Mean values in channel order. Typically, this is BGR order.
         */
        std::optional<std::vector<double>> mean;
        /**
         * If True, color channels are reversed (e.g. BGR to RGB or vice versa).
         */
        std::optional<bool> reverseChannels;
        /**
         * Standardization values in channel order. Typically, this is BGR order.
         */
        std::optional<std::vector<double>> scale;
    };
}
}
