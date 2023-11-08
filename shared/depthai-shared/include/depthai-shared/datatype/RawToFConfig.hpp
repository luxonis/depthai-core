#pragma once
#include <cstdint>
#include <vector>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawBuffer.hpp"
#include "depthai-shared/utility/Serialization.hpp"

namespace dai {

/// RawToFConfig configuration structure
struct RawToFConfig : public RawBuffer {
    /**
     * DepthParams configuration structure.
     */
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

    /**
     * DepthParams configuration.
     * Used for configuring the ToF.
     */
    DepthParams depthParams;

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::ToFConfig;
    };

    DEPTHAI_SERIALIZE(RawToFConfig, depthParams);
};

}  // namespace dai
