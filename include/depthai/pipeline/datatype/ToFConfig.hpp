#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

/**
 * ToFConfig message. Carries config for feature tracking algorithm
 */
class ToFConfig : public Buffer {
   public:
    struct DepthParams {
        /**
         * Enable feature maintaining or not.
         */
        bool enable = true;

        /**
         * Enable averaging between phases with same modulation frequency(e.g. for ToF cameras with phase shuffle).
         * The depth frame rate will be half if this is enabled
         */
        bool avgPhaseShuffle = false;

        /**
         * Perform depth calculation only for pixels with amplitude greater than provided value
         */
        float minimumAmplitude = 5.0;

        /**
         * Frequency modulation frames used for depth calculation. If the ToF sensor supports multiple modulation frequencies,
         * all will be used for depth calculation.
         */
        enum class TypeFMod : std::int32_t { F_MOD_ALL, F_MOD_MIN, F_MOD_MAX };

        TypeFMod freqModUsed = TypeFMod::F_MOD_MIN;

        DEPTHAI_SERIALIZE(DepthParams, enable, avgPhaseShuffle, minimumAmplitude, freqModUsed);
    };

    DepthParams depthParams;
    /**
     * Construct ToFConfig message.
     */
    ToFConfig() = default;
    virtual ~ToFConfig() = default;

    // TODO(before mainline) - API not supported on RVC3
    ToFConfig& setDepthParams(dai::ToFConfig::DepthParams config);
    ToFConfig& setFreqModUsed(dai::ToFConfig::DepthParams::TypeFMod fmod);
    ToFConfig& setAvgPhaseShuffle(bool enable);
    ToFConfig& setMinAmplitude(float minamp);
};

}  // namespace dai
