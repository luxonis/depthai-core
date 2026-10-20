#pragma once

#include "depthai/common/Interpolation.hpp"
#include "depthai/pipeline/datatype/AlignConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai {

/**
 * Specify properties for Align
 */
struct AlignProperties : PropertiesSerializable<Properties, AlignProperties> {
    AlignConfig initialConfig;

    /// Num frames in output pool
    int numFramesPool = 4;

    /// Image interpolation. AUTO preserves the existing backend default.
    Interpolation interpolation = Interpolation::AUTO;

    ~AlignProperties() override;
};

// Accept pipeline schemas from SDKs predating interpolation selection.
inline void to_json(nlohmann::json& j, const AlignProperties& properties) {
    j = nlohmann::json{{"initialConfig", properties.initialConfig}, {"numFramesPool", properties.numFramesPool}, {"interpolation", properties.interpolation}};
}
inline void from_json(const nlohmann::json& j, AlignProperties& properties) {
    j.at("initialConfig").get_to(properties.initialConfig);
    j.at("numFramesPool").get_to(properties.numFramesPool);
    properties.interpolation = j.value("interpolation", Interpolation::AUTO);
}
NOP_EXTERNAL_STRUCTURE(AlignProperties, initialConfig, numFramesPool, interpolation);

}  // namespace dai
