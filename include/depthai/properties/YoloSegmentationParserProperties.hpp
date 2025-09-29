#pragma once

#include <vector>

#include "depthai/common/TensorInfo.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for DetectionParser
 */
struct YoloSegmentationParserProperties : PropertiesSerializable<Properties, YoloSegmentationParserProperties> {
    /// Num frames in output pool
    int numFramesPool = 8;

    /// Network inputs
    std::unordered_map<std::string, TensorInfo> networkInputs;
    int imgWidth = 512;
    int imgHeight = 288;

    // // Find outputs by name (exact names from your JSON).
    // const auto* out1m = findOutput(cfg, "output1_masks");
    // const auto* out2m = findOutput(cfg, "output2_masks");
    // const auto* out3m = findOutput(cfg, "output3_masks");
    // const auto* proto = findOutput(cfg, "protos_output");

    // if (!out1m || !out2m || !out3m || !proto) {
    //     throw std::runtime_error("Config: required outputs not found (output{1,2,3}_masks, protos_output)");
    // }

    // // Shapes are NHWC (N may be present as leading dim).
    // std::array<int,4> s1 = shape4(*out1m);
    // std::array<int,4> s2 = shape4(*out2m);
    // std::array<int,4> s3 = shape4(*out3m);
    // std::array<int,4> sp = shape4(*proto);

    // // Expect C == 32 everywhere (mask coeffs and protos)
    // if (s1[3] != 32 || s2[3] != 32 || s3[3] != 32 || sp[3] != 32) {
    //     throw std::runtime_error("Config: expected 32 channels for mask heads and protos");
    // }

    // heads_[0] = { s1[1], s1[2], s1[3] }; // H,W,C
    // heads_[1] = { s2[1], s2[2], s2[3] };
    // heads_[2] = { s3[1], s3[2], s3[3] };
    // Hp_ = sp[1]; Wp_ = sp[2]; Cp_ = sp[3];

    // // Precompute scaling from image to protos
    // scale_y_to_proto_ = static_cast<float>(Hp_) / std::max(1, img_h_);
    // scale_x_to_proto_ = static_cast<float>(Wp_) / std::max(1, img_w_);

    std::string out1m = "output1_masks";
    std::string out2m = "output2_masks";
    std::string out3m = "output3_masks";
    std::string proto = "output_proto";

    float scale_y_to_proto = static_cast<float>(72) / static_cast<float>(imgHeight);
    float scale_x_to_proto = static_cast<float>(128) / static_cast<float>(imgWidth);
};

DEPTHAI_SERIALIZE_EXT(
    YoloSegmentationParserProperties, numFramesPool, networkInputs, imgWidth, imgHeight, out1m, out2m, out3m, proto, scale_y_to_proto, scale_x_to_proto);

}  // namespace dai
