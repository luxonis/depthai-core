#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * TofFusionConfig message. Carries runtime-tunable parameters for the TofFusion node.
 */
class TofFusionConfig : public Buffer {
   public:
    TofFusionConfig() = default;
    virtual ~TofFusionConfig();

    // --- Fusion weight parameters ---
    float tof_base_weight = 3.0f;
    float tof_conf_power = 1.0f;
    float tof_range_decay_start = 4.0f;
    float tof_range_decay_rate = 0.3f;

    float neural_base_weight = 1.0f;
    float neural_conf_power = 0.5f;

    float texture_tof_boost = 0.5f;
    float texture_neural_boost = 0.3f;

    float agree_thresh_rel = 0.05f;
    float agree_tof_bonus = 2.0f;
    float disagree_neural_penalty = 0.5f;

    float neural_depth_scale = 0.955f;

    // --- Pre-projection knobs ---
    float tof_preproj_bias_m = 0.0f;
    float tof_preproj_scale = 1.0f;
    float tof_preproj_upscale = 1.0f;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::TofFusionConfig;
    }

    DEPTHAI_SERIALIZE(TofFusionConfig,
                      tof_base_weight,
                      tof_conf_power,
                      tof_range_decay_start,
                      tof_range_decay_rate,
                      neural_base_weight,
                      neural_conf_power,
                      texture_tof_boost,
                      texture_neural_boost,
                      agree_thresh_rel,
                      agree_tof_bonus,
                      disagree_neural_penalty,
                      neural_depth_scale,
                      tof_preproj_bias_m,
                      tof_preproj_scale,
                      tof_preproj_upscale);
};

}  // namespace dai
