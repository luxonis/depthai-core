#pragma once

#include <cstdint>

#include "depthai/beta/datatype/ToFStereoFusionConfig.hpp"
#include "depthai/properties/Properties.hpp"

namespace dai::beta {

struct ToFStereoFusionProperties : PropertiesSerializable<Properties, ToFStereoFusionProperties> {
    /** Fusion model. */
    enum class Model : int32_t {
        /** Blend ToF and NeuralDepth with a per-pixel weight from a small network. Output in the ToF view. */
        ALPHA_BLEND = 0,
        /** One network takes the rectified stereo pair and ToF planes and outputs depth in the rectified-left view. */
        STEREO_NETWORK = 1,
    };

    /** Where the STEREO_NETWORK pre- and post-processing runs. */
    enum class Backend : int32_t {
        /** GPU when the device has one and OpenCL initializes, else CPU. */
        AUTO = 0,
        CPU = 1,
        /** GPU only; the node fails instead of falling back to the CPU. */
        GPU = 2,
    };

    /** Scale of the ToF confidence the STEREO_NETWORK model receives. */
    enum class ToFConfidenceScale : int32_t {
        /** Confidence from the ToF fusion depth mode, used as is. */
        FUSION = 0,
        /** Vendor ToF confidence, mapped to the fusion scale with a fixed table. */
        VENDOR = 1,
    };

    ToFStereoFusionConfig initialConfig;
    Model model = Model::ALPHA_BLEND;
    Backend backend = Backend::AUTO;
    ToFConfidenceScale tofConfidenceScale = ToFConfidenceScale::FUSION;

    ~ToFStereoFusionProperties() override;
};

DEPTHAI_SERIALIZE_EXT(ToFStereoFusionProperties, initialConfig, model, backend, tofConfidenceScale);

}  // namespace dai::beta
